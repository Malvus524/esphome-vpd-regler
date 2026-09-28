"""Importer tests use invented measurements, never private sensor history."""
import csv
import importlib.util
import io
from pathlib import Path
import shutil
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("replay", Path(__file__).resolve().parents[1] / "tools/replay.py")
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


class ReplayTests(unittest.TestCase):
    def test_causal_ticks_and_independent_expiry(self):
        data = io.StringIO("timestamp,temperature,humidity\n0,25,60\n15,,61\n40,26,\n")
        result = list(replay.ticks(iter(replay.records(data, {})), {}, 20))
        self.assertEqual([t for t, _ in result], [0, 10, 20, 30, 40])
        self.assertEqual(result[1][1]["humidity"], 60)  # no future interpolation
        self.assertTrue(replay.math.isnan(result[2][1]["temperature"]))
        self.assertEqual(result[2][1]["humidity"], 61)
        self.assertTrue(replay.math.isnan(result[-1][1]["humidity"]))

    def test_bad_order_and_timezone_rejected(self):
        with self.assertRaises(ValueError):
            list(replay.records(io.StringIO("timestamp,temperature,humidity\n10,25,60\n9,25,60\n"), {}))
        with self.assertRaises(ValueError): replay.timestamp("2026-09-27T12:00:00")

    def test_history_sort_merge_and_unknown_settings(self):
        data = io.StringIO("entity_id,state,last_changed\nt,25,0\nt,unavailable,30\nh,60,0\na,on,0\na,unavailable,20\na,on,40\n")
        with tempfile.TemporaryDirectory() as directory:
            diagnostics = {}
            result = list(replay.history_records(data, {"temperature": "t", "humidity": "h", "auto_on": "a"},
                                                Path(directory) / "history.db", diagnostics))
        self.assertEqual([t for t, _ in result], [0, 20, 30, 40])
        self.assertEqual(result[0][1]["humidity"], 60)
        self.assertEqual(result[1][1]["context_valid"], 0)
        self.assertTrue(replay.math.isnan(result[2][1]["temperature"]))
        self.assertEqual(result[3][1]["context_valid"], 1)

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler required")
    def test_production_core_stream(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.csv"
            destination = Path(directory) / "result.csv"
            source.write_text("timestamp,temperature,humidity,recorded_fan\n0,25,60,30\n10,35,60,100\n20,25,60,80\n")
            summary = replay.replay(source, destination, {}, "observed")
            with destination.open() as stream: rows = list(csv.DictReader(stream))
            self.assertEqual(summary["ticks"], 3)
            self.assertEqual(float(rows[1]["fan_output"]), 100)  # immediate safety
            self.assertGreater(float(rows[2]["fan_output"]), 90)  # gradual recovery


if __name__ == "__main__": unittest.main()
