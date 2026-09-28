# Replay real sensor history

`replay.py` runs recorded measurements through the same C++ controller headers
as the firmware. Requires Python 3.10+ and `g++`; no Python packages are needed.
Nothing is sent to a device or to Home Assistant.

```sh
python tools/replay.py history.csv replay.csv --config replay-config.json --mode observed
```

The output CSV contains raw/filtered VPD, active target, proposed fan output,
economic ceiling, benefit estimate, fallback diagnostics and state text. A JSON
summary is printed to stdout. Save it with shell redirection if desired.
`--core PATH` selects an already compiled adapter, useful for replaying different
revisions against identical data:

```sh
g++ -std=c++17 -O2 -I . tools/replay_core.cpp -o replay-core
```

## Two distinct modes

- `shadow`: feed recorded climate into a separately evolving controller.
- `observed`: also feed the last recorded fan output into the climate estimator
  each tick. The controller still proposes its own next output. Missing recorded
  output marks the tick's context invalid and is counted in the summary.

Neither mode predicts what the tent **would have done** with a different fan
output. Lower fan travel or closer agreement with historical output alone does
not establish better climate regulation or energy savings. The model uses the
default airflow curve and 21 s time constant; experimental fan learning is off.
Internal persisted state is not reconstructed. Set `initial_output` and allow
a warm-up period when comparing decisions.

## Publication CSV

Use comma-separated columns with a header:

```csv
timestamp,temperature,humidity,room_temperature,room_humidity,recorded_fan,night
2026-09-27T12:00:00Z,25,60,22,50,30,off
2026-09-27T12:00:10Z,25,61,22,50,31,off
```

Timestamps must be strictly increasing Unix seconds or ISO timestamps including
timezone. Values are degrees Celsius, %RH, kPa and fan percentage (0–100).
Boolean values accept on/off, true/false or 1/0. Blank cells mean **no new
publication**. `unknown`, `unavailable` and `nan` invalidate a sensor immediately.
Values are carried forward only until their individual `sensor_timeout_s`
(default 120 s). Replay ticks every 10 s without future interpolation.

Optional columns include `leaf_temperature`, `leaf_switch`, `auto_on`,
`tent_open`, `force_fallback`, `target_day`, `target_night`, `sacrifice`,
`deadband`, `speed`, `transition`, protection thresholds and fan limits.
See `RANGES` and `BOOLS` in the script for the supported settings. Missing
night state stays unknown. Settings can be fixed in JSON; recorded changes
override them. Names can be mapped with `columns`:

```json
{
  "room_configured": true,
  "initial_output": 30,
  "sensor_timeout_s": 120,
  "settings": {"target_day": 1.3, "target_night": 1.1},
  "columns": {"temperature": "tent_temperature"}
}
```

## Home Assistant history export

The importer also supports `entity_id,state,last_changed` exports. Set
`entities` instead of `columns`, mapping canonical names to the actual entity
IDs. Rows may be grouped by entity: sorting uses a temporary SQLite database.
Simultaneous updates are merged before replay.

```json
{
  "room_configured": true,
  "start": "2026-09-27T00:00:00Z",
  "end": "2026-09-28T00:00:00Z",
  "entities": {
    "temperature": "sensor.tent_temperature",
    "humidity": "sensor.tent_humidity",
    "room_temperature": "sensor.room_temperature",
    "room_humidity": "sensor.room_humidity",
    "recorded_fan": "sensor.controller_fan_output",
    "auto_on": "switch.controller_enabled",
    "night": "binary_sensor.night",
    "target_day": "number.target_day",
    "target_night": "number.target_night"
  }
}
```

HA history records state changes, not every sensor publication. Unchanged values
therefore remain valid until explicitly invalidated; **sensor publication
timeouts cannot be verified from this format**. Missing/invalid mapped settings
mark `context_valid=0` until all mapped settings are known. The last valid setting
is retained internally during those gaps; do not treat the resulting proposals
as a faithful reconstruction. Invalid-setting event counts include imported
history before the selected replay window.

Map the **actually selected** climate source and effective leaf temperature.
If only an external source was exported, map its selection switch as
`external_selected` and set `required_external_source: true`. Other source
intervals are marked unrepresented and their tent measurements invalidated.
This does not emulate an external-source failure switching to an unrecorded
local sensor. Source selection, leaf mode, restarts and historic firmware
changes must be accounted for before drawing conclusions.

## Collecting useful data

Record the selected tent and room temperature/humidity, effective leaf temperature
or leaf mode/offsets, raw and filtered VPD, day/night state, configured and active
targets, automatic/manual mode, tent-open state, applied fan output, controller
output, economic ceiling, benefit estimate and protection states. Keep tuning
values and firmware revision with the recording. Publish timestamps/heartbeats
are needed to test freshness; HA state-change timestamps are insufficient.

Use `measured_vpd`, `control_vpd`, `target_active`, `fan_output`,
`controller_output`, `sensible_max`, `next_step_benefit`, `state`,
`temperature_protection` and `humidity_protection` as optional component entities.
The raw sensor history is private input and is not needed by CI; importer tests
use small invented fixtures.
