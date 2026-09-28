"""Schema regression tests; run with python -m unittest discover -s tests."""

import importlib.util
from pathlib import Path
import unittest

from esphome.core import CORE
from esphome import config_validation as cv

PATH = Path(__file__).resolve().parents[1] / "components/vpd_kalman/__init__.py"
SPEC = importlib.util.spec_from_file_location("vpd_component", PATH)
COMPONENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COMPONENT)


class ConfigTests(unittest.TestCase):
    def config(self, **overrides):
        CORE.reset()
        return COMPONENT.CONFIG_SCHEMA({
            "output": "fan_pwm", "temperature": "tent_t", "humidity": "tent_rh", **overrides
        })

    def test_minimal_modes(self):
        config = self.config()
        self.assertIn("target", config)
        self.assertTrue(config["safety_in_manual"])
        self.assertFalse(config["learn_fan_curve"])
        config = self.config(night="night_phase", room_temperature="room_t", room_humidity="room_rh")
        self.assertIn("target_day", config)
        self.assertIn("target_night", config)

    def test_number_ranges(self):
        for options in [
            {"min_value": 90, "max_value": 10},
            {"initial_value": 101},
            {"initial_value": float("nan")},
            {"step": float("inf")},
            {"min_value": 0},
            {"min_value": 60, "initial_value": 50},
        ]:
            with self.subTest(options=options), self.assertRaises(cv.Invalid):
                self.config(manual_speed={"name": "Manual", **options})

    def test_tuning_scalar_and_entity_share_limits(self):
        for value in [0, float("inf"), {"initial_value": 0}, {"min_value": 0}]:
            with self.subTest(value=value), self.assertRaises(cv.Invalid):
                self.config(tuning={"fallback_rate": value})

    def test_fan_limits(self):
        with self.assertRaises(cv.Invalid):
            self.config(fan_min={"name": "Min", "initial_value": 80},
                        fan_max={"name": "Max", "initial_value": 20})

    def test_control_strategy_tuning(self):
        room = {"room_temperature": "room_t", "room_humidity": "room_rh"}
        self.config(**room, tuning={"control_smoothing": "0s", "trend_horizon": "0s",
                                   "fan_increase_rate": 30, "protection_release_rate": 20},
                    measured_vpd={"name": "Raw VPD"})
        for key, value in [("control_smoothing", 121), ("trend_horizon", -1),
                           ("fan_increase_rate", 0), ("protection_release_rate", 101)]:
            with self.subTest(key=key), self.assertRaises(cv.Invalid):
                self.config(**room, tuning={key: value})
        with self.assertRaises(cv.Invalid):
            self.config(tuning={"control_smoothing": 10})

    def test_learning_requires_room_and_explicit_opt_in(self):
        with self.assertRaises(cv.Invalid):
            self.config(learn_fan_curve=True)
        with self.assertRaises(cv.Invalid):
            self.config(room_temperature="room_t", room_humidity="room_rh",
                        learned_airflow_50={"name": "Learned airflow"})
        self.config(room_temperature="room_t", room_humidity="room_rh", learn_fan_curve=True)

    def test_time_limits_and_finite_parameters(self):
        for value in ["0s", "1ms", "9s", "25h"]:
            with self.subTest(value=value), self.assertRaises(cv.Invalid):
                self.config(sensor_timeout=value)
        with self.assertRaises(cv.Invalid):
            self.config(time_constant="1ms")
        with self.assertRaises(cv.Invalid):
            self.config(airflow_at_zero=float("nan"))


if __name__ == "__main__":
    unittest.main()
