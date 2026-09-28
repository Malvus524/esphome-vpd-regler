"""Causal replay of real sensor publications through the production C++ core."""
import argparse
import csv
from contextlib import closing, ExitStack
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import tempfile
import sqlite3

SENSORS = {"temperature", "humidity", "room_temperature", "room_humidity", "leaf_temperature", "recorded_fan"}
BOOLS = {"auto_on", "night", "tent_open", "force_fallback", "leaf_switch", "safety_in_manual", "external_selected"}
RANGES = {
    "target_day": (0.3, 2.5), "target_night": (0.3, 2.5), "deadband": (0.01, 0.5),
    "sacrifice": (0, 0.3), "speed": (0.05, 1), "transition": (0, 180),
    "manual_speed": (1, 100), "fan_min": (1, 100), "fan_max": (1, 100),
    "emergency": (1, 100), "temp_max": (20, 40), "temp_band": (0.5, 10),
    "rh_max": (40, 95), "rh_band": (1, 30), "open_max": (5, 240),
    "leaf_offset_day": (-10, 5), "leaf_offset_night": (-10, 5),
    "control_smoothing": (0, 120), "trend_horizon": (0, 120),
    "fan_increase_rate": (1, 600), "protection_release_rate": (1, 100),
    "fallback_time_constant": (0.5, 10), "fallback_rate": (1, 100),
    "fallback_smoothing": (10, 300), "limit_finder_cost": (0, 20),
    "limit_finder_step": (5, 50), "limit_test_margin": (0, 0.1),
    "limit_test_max_pause": (5, 480), "sensor_noise": (0.001, 0.1),
    "load_change_per_hour": (1, 200), "sensible_max_rate": (0.1, 100),
}
DEFAULTS = {"auto_on": 1, "leaf_switch": 1, "safety_in_manual": 1,
            "target_day": 1.2, "target_night": 1, "transition": 45,
            "leaf_offset_day": -2, "leaf_offset_night": -1}
INVALID = {"nan", "unknown", "unavailable", "null", "none"}


def timestamp(value):
    try:
        result = float(value)
    except ValueError:
        dt = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if dt.tzinfo is None:
            raise ValueError("ISO timestamps must include a timezone")
        result = dt.timestamp()
    if not math.isfinite(result):
        raise ValueError("Timestamp must be finite")
    return result


def number(key, value):
    text = str(value).strip().lower()
    if key in SENSORS and text in INVALID:
        return float("nan")
    if key in BOOLS:
        if text in {"true", "on", "1"}: return 1
        if text in {"false", "off", "0"}: return 0
        raise ValueError(f"{key}: expected true/false, on/off or 1/0")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{key}: non-finite value")
    if key in RANGES and not RANGES[key][0] <= result <= RANGES[key][1]:
        raise ValueError(f"{key}: outside {RANGES[key]}")
    if key == "recorded_fan" and not 0 <= result <= 100:
        raise ValueError("recorded_fan must be 0..100 percent")
    return result


def records(stream, columns):
    reader = csv.DictReader(stream)
    headers = reader.fieldnames or []
    for key in ("timestamp", "temperature", "humidity"):
        if columns.get(key, key) not in headers:
            raise ValueError(f"Missing CSV column: {columns.get(key, key)}")
    for key, source in columns.items():
        if source not in headers: raise ValueError(f"Missing mapped column: {source}")
        if key not in SENSORS | BOOLS | RANGES.keys() | {"timestamp"}:
            raise ValueError(f"Unknown canonical column: {key}")
    previous = -math.inf
    for row_number, row in enumerate(reader, 2):
        try:
            if None in row: raise ValueError("Too many CSV fields")
            when = timestamp(row[columns.get("timestamp", "timestamp")])
            if when <= previous: raise ValueError("Timestamps must be strictly increasing; combine simultaneous publications")
            previous = when
            values = {}
            for key in SENSORS | BOOLS | RANGES.keys():
                value = row.get(columns.get(key, key))
                if value is not None and value.strip(): values[key] = number(key, value)
            yield when, values
        except (ValueError, TypeError) as error:
            raise ValueError(f"CSV row {row_number}: {error}") from error


def history_records(stream, entities, database, diagnostics):
    """Sort HA state-change exports on disk; merge simultaneous changes causally."""
    if not entities: raise ValueError("HA history requires an entities mapping")
    if not {"temperature", "humidity"} <= entities.keys(): raise ValueError("Map tent temperature and humidity")
    if entities.keys() - (SENSORS | BOOLS | RANGES.keys()): raise ValueError("Unknown entity mapping")
    if len(set(entities.values())) != len(entities): raise ValueError("Map each entity only once")
    reverse = {entity: key for key, entity in entities.items()}
    reader = csv.DictReader(stream)
    if not {"entity_id", "state", "last_changed"} <= set(reader.fieldnames or []):
        raise ValueError("Expected HA columns entity_id,state,last_changed")
    with closing(sqlite3.connect(database)) as db:
        db.execute("CREATE TABLE samples (time REAL, key TEXT, value TEXT)")
        found = set()
        for row in reader:
            key = reverse.get(row["entity_id"])
            if key is None: continue
            found.add(key)
            db.execute("INSERT INTO samples VALUES (?, ?, ?)",
                       (timestamp(row["last_changed"]), key, row["state"]))
        if entities.keys() - found: raise ValueError(f"Entities absent: {entities.keys() - found}")
        db.commit()
        unknown = set(entities) - SENSORS
        when = None
        values = {}
        for time, key, text in db.execute("SELECT time,key,value FROM samples ORDER BY time,rowid"):
            if when is not None and time != when:
                values["context_valid"] = int(not unknown)
                yield when, values
                values = {}
            when = time
            try:
                value = number(key, text or "unknown")
            except ValueError:
                diagnostics["invalid_setting_events"] = diagnostics.get("invalid_setting_events", 0) + 1
                unknown.add(key)
                continue
            values[key] = value
            unknown.discard(key)
        if when is not None:
            values["context_valid"] = int(not unknown)
            yield when, values


def ticks(rows, settings, timeout):
    """Forward-fill publications only until their own timeout; never interpolate."""
    pending = next(rows, None)
    if pending is None: raise ValueError("CSV has no samples")
    current = dict(DEFAULTS, **settings)
    ages = {}
    tick = pending[0]
    while pending is not None:
        while pending is not None and pending[0] <= tick:
            when, values = pending
            current.update(values)
            for key in values:
                if key in SENSORS: ages[key] = when
            pending = next(rows, None)
        values = dict(current)
        for key in SENSORS:
            if key not in ages or tick - ages[key] >= timeout:
                values[key] = float("nan")
        values["night_has_state"] = int("night" in current)
        yield tick, values
        tick += 10
        # Do not fabricate a final tick beyond the end of the recording.
        if pending is not None and pending[0] < tick:
            # The next iteration can consume it only if there are later records.
            while pending is not None and pending[0] < tick:
                when, updates = pending
                current.update(updates)
                for key in updates:
                    if key in SENSORS: ages[key] = when
                pending = next(rows, None)


def replay(input_path, output_path, config, mode="shadow", core_path=None):
    allowed = {"columns", "entities", "settings", "room_configured", "sensor_timeout_s", "initial_output", "start", "end", "required_external_source"}
    if config.keys() - allowed: raise ValueError(f"Unknown config keys: {config.keys() - allowed}")
    settings = config.get("settings", {})
    if settings.keys() - (RANGES.keys() | BOOLS): raise ValueError("Unsupported setting")
    settings = {key: number(key, value) for key, value in settings.items()}
    timeout = float(config.get("sensor_timeout_s", 120))
    initial = float(config.get("initial_output", 30))
    if not math.isfinite(timeout) or not 10 <= timeout <= 86400: raise ValueError("Invalid sensor_timeout_s")
    if not math.isfinite(initial) or not 1 <= initial <= 100: raise ValueError("Invalid initial_output")
    room = config.get("room_configured", False)
    if "required_external_source" in config and not isinstance(config["required_external_source"], bool):
        raise ValueError("required_external_source must be boolean")
    start = timestamp(config["start"]) if "start" in config else -math.inf
    end = timestamp(config["end"]) if "end" in config else math.inf
    if end < start: raise ValueError("end must not precede start")
    if not isinstance(room, bool): raise ValueError("room_configured must be boolean")
    if input_path.resolve() == output_path.resolve(): raise ValueError("Output must differ from input")
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="vpd-replay-") as tmp:
        tmp = Path(tmp)
        if core_path is None:
            core_path = tmp / "replay-core.exe"
            subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(root),
                            str(root / "tools/replay_core.cpp"), "-o", str(core_path)], check=True)
        count = 0
        missing = 0
        valid_context = 0
        diagnostics = {}
        with input_path.open(encoding="utf-8-sig", newline="") as source, \
                (tmp / "input").open("w") as protocol, (tmp / "metadata").open("w", newline="") as meta, ExitStack() as cleanup:
            protocol.write(f"room_configured {int(room)} initial_output {initial}\n")
            writer = csv.writer(meta)
            history = "entities" in config
            rows = history_records(source, config["entities"], tmp / "history.db", diagnostics) if history else records(source, config.get("columns", {}))
            cleanup.callback(rows.close)
            for when, values in ticks(iter(rows), settings, math.inf if history else timeout):
                if when < start: continue
                if when > end: break
                context_valid = values.pop("context_valid", 1)
                source_selected = values.pop("external_selected", None)
                if "required_external_source" in config and source_selected != int(config["required_external_source"]):
                    context_valid = 0
                    for key in ("temperature", "humidity", "leaf_temperature"): values[key] = float("nan")
                    diagnostics["unrepresented_source_ticks"] = diagnostics.get("unrepresented_source_ticks", 0) + 1
                recorded = values.pop("recorded_fan")
                if mode == "observed":
                    if math.isfinite(recorded): values["observed_fan"] = recorded
                    else:
                        context_valid = 0
                        diagnostics["missing_observed_fan_ticks"] = diagnostics.get("missing_observed_fan_ticks", 0) + 1
                protocol.write(" ".join(f"{key} {value}" for key, value in values.items()) + "\n")
                writer.writerow([datetime.fromtimestamp(when, timezone.utc).isoformat(), recorded, context_valid])
                count += 1
                valid_context += context_valid
                missing += not (math.isfinite(values["temperature"]) and math.isfinite(values["humidity"]))
        if count == 0: raise ValueError("No ticks in selected interval")
        with (tmp / "input").open() as source, (tmp / "result").open("w") as result:
            subprocess.run([str(core_path.resolve())], stdin=source, stdout=result, check=True)
        travel = deviation = 0.0
        compared = 0
        previous = None
        with (tmp / "result").open() as result, (tmp / "metadata").open(newline="") as meta, \
                (tmp / "output").open("w", newline="") as destination:
            reader = csv.DictReader(result, delimiter="\t")
            writer = csv.writer(destination)
            writer.writerow(["timestamp", "recorded_fan", "context_valid", *reader.fieldnames])
            for row, metadata in zip(reader, csv.reader(meta), strict=True):
                fan = float(row["fan_output"])
                if previous is not None: travel += abs(fan - previous)
                previous = fan
                actual = float(metadata[1])
                if math.isfinite(actual) and metadata[2] == "1" and math.isfinite(float(row["measured_vpd"])):
                    deviation += abs(fan - actual)
                    compared += 1
                writer.writerow([*metadata, *row.values()])
        output_path.write_bytes((tmp / "output").read_bytes())
        return {"mode": mode, "ticks": count, "missing_tent_ticks": missing,
                "valid_setting_context_ticks": valid_context, **diagnostics,
                "freshness": "State-change history: publication timeouts cannot be reconstructed" if history else "Per-sensor publication timeout",
                "fan_travel_percentage_points": travel,
                "mean_difference_from_recorded_fan": deviation / compared if compared else None,
                "note": "Decision replay only. Recorded climate is not the counterfactual response to proposed fan output.",
                "config": config}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mode", choices=["shadow", "observed"], default="shadow")
    parser.add_argument("--core", type=Path, help="Use an already compiled replay_core executable")
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text(encoding="utf-8")) if args.config else {}
        print(json.dumps(replay(args.input, args.output, config, args.mode, args.core), indent=2, allow_nan=False))
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Replay failed: {error}\n")


if __name__ == "__main__":
    main()
