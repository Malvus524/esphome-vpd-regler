"""VPD controller for grow tents: excess-humidity controller with Kalman filter.

Drives one exhaust fan so that the tent reaches a VPD target for day and
night. All settings are created as number/switch entities, all diagnostics
are optional sensors. See README.md.

With room_temperature/room_humidity the Kalman controller runs and the
fallback controller (PI on a log fan scale with limit finder,
vpd_fallback_core.h) takes over while the room sensor fails. Without them only
the fallback controller runs. Optional tuning parameters (tuning:) are fixed
values, or number entities if they get a name.
"""

import hashlib

from esphome import automation
import esphome.codegen as cg
from esphome.components import binary_sensor, number, output, sensor, switch, text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_INITIAL_VALUE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_MODE,
    CONF_NAME,
    CONF_OUTPUT,
    CONF_STEP,
    DEVICE_CLASS_HEAT,
    DEVICE_CLASS_MOISTURE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)

CODEOWNERS = ["@Malvus524"]
AUTO_LOAD = ["sensor", "number", "switch", "text_sensor", "binary_sensor"]
DEPENDENCIES = ["output"]

vpd_kalman_ns = cg.esphome_ns.namespace("vpd_kalman")
VpdKalman = vpd_kalman_ns.class_("VpdKalman", cg.PollingComponent)
VpdNumber = vpd_kalman_ns.class_("VpdNumber", number.Number, cg.Component)
VpdSwitch = vpd_kalman_ns.class_("VpdSwitch", switch.Switch, cg.Component)
ExternalClimate = vpd_kalman_ns.struct("ExternalClimate")

LANGUAGES = {
    "en": vpd_kalman_ns.LANGUAGE_EN,
    "de": vpd_kalman_ns.LANGUAGE_DE,
}

CONF_NIGHT = "night"
CONF_TEMPERATURE = "temperature"
CONF_HUMIDITY = "humidity"
CONF_LEAF_TEMPERATURE = "leaf_temperature"
CONF_ROOM_TEMPERATURE = "room_temperature"
CONF_ROOM_HUMIDITY = "room_humidity"
CONF_EXTERNAL_CLIMATE = "external_climate"
CONF_LANGUAGE = "language"
CONF_AIRFLOW_AT_ZERO = "airflow_at_zero"
CONF_TIME_CONSTANT = "time_constant"
CONF_LEAF_MAX_DEVIATION = "leaf_max_deviation"
CONF_STORAGE_KEYS = "storage_keys"
CONF_ON_MESSAGE = "on_message"
CONF_CONTROLLER_OUTPUT = "controller_output"
CONF_DAY = "day"
CONF_NIGHT_KEY = "night"
CONF_TUNING = "tuning"

# key: (default name, icon, unit, min, max, step, initial, entity category)
NUMBERS = {
    "manual_speed": ("Fan manual speed", "mdi:fan", "%", 1, 100, 1, 50, None),
    "target_day": ("VPD target day", "mdi:target", "kPa", 0.3, 2.5, 0.01, 1.1, ENTITY_CATEGORY_CONFIG),
    "target_night": ("VPD target night", "mdi:target", "kPa", 0.3, 2.5, 0.01, 0.9, ENTITY_CATEGORY_CONFIG),
    "deadband": ("VPD deadband", "mdi:arrow-expand-horizontal", "kPa", 0.01, 0.5, 0.01, 0.05,
                 ENTITY_CATEGORY_CONFIG),
    "vpd_sacrifice": ("Allowed VPD sacrifice", "mdi:scale-balance", "kPa", 0, 0.3, 0.005, 0.03,
                      ENTITY_CATEGORY_CONFIG),
    "speed": ("Controller speed", "mdi:speedometer", None, 0.05, 1.0, 0.05, 0.25, ENTITY_CATEGORY_CONFIG),
    "transition": ("Setpoint transition", "mdi:chart-line-variant", "min", 0, 180, 1, 45,
                   ENTITY_CATEGORY_CONFIG),
    "tent_open_max": ("Tent open max. duration", "mdi:timer-lock-open-outline", "min", 5, 240, 1, 30,
                      ENTITY_CATEGORY_CONFIG),
    "fan_min": ("Fan minimum automatic", "mdi:fan-chevron-down", "%", 1, 100, 1, 5, ENTITY_CATEGORY_CONFIG),
    "fan_max": ("Fan maximum automatic", "mdi:fan-chevron-up", "%", 1, 100, 1, 100, ENTITY_CATEGORY_CONFIG),
    "fan_emergency": ("Fan emergency speed", "mdi:fan-alert", "%", 1, 100, 1, 40, ENTITY_CATEGORY_CONFIG),
    "temperature_max": ("Safety temperature max", "mdi:thermometer-high", "°C", 20, 40, 0.5, 28,
                        ENTITY_CATEGORY_CONFIG),
    "temperature_band": ("Safety temperature P-band", "mdi:thermometer-lines", "K", 0.5, 10, 0.5, 3,
                         ENTITY_CATEGORY_CONFIG),
    "humidity_max": ("Safety humidity max", "mdi:water-percent-alert", "%", 40, 95, 1, 75,
                     ENTITY_CATEGORY_CONFIG),
    "humidity_band": ("Safety humidity P-band", "mdi:water-percent", "%", 1, 30, 1, 10, ENTITY_CATEGORY_CONFIG),
    "leaf_offset_day": ("Leaf offset day", "mdi:white-balance-sunny", "°C", -10, 5, 0.1, -2.0,
                        ENTITY_CATEGORY_CONFIG),
    "leaf_offset_night": ("Leaf offset night", "mdi:weather-night", "°C", -10, 5, 0.1, -1.0,
                          ENTITY_CATEGORY_CONFIG),
}

# Optional tuning parameters (tuning:). Not set = the built-in default, no
# entity. A plain value (for s/min also a time like "2min") fixes it, a block
# with name makes it a number entity that can be changed live.
# key: (default name, icon, unit, min, max, step, default)
TUNING = {
    # Switching Kalman <-> fallback when a configured room sensor fails
    "room_fallback_delay": ("Room sensor fallback delay", "mdi:timer-alert-outline", "min", 0.5, 60, 0.5, 2),
    "room_return_delay": ("Room sensor return delay", "mdi:timer-check-outline", "min", 0.5, 60, 0.5, 1),
    # Remembering the fan per light phase (Kalman)
    "light_memory_after": ("Light phase memory after", "mdi:timer-sand", "min", 0, 240, 1, 20),
    "light_memory_delay": ("Light phase memory delay", "mdi:history", "min", 0.5, 60, 0.5, 5),
    # Start
    "boot_wait_tent": ("Boot wait tent sensor", "mdi:timer-outline", "s", 10, 600, 10, 60),
    "boot_wait_night": ("Boot wait day/night", "mdi:timer-outline", "s", 10, 1800, 10, 180),
    # Kalman filter and sensible maximum
    "sensible_max_rate": ("Sensible maximum rate", "mdi:speedometer-slow", "%/min", 0.1, 100, 0.1, 5),
    "sensor_noise": ("Kalman sensor noise", "mdi:chart-bell-curve", "kPa", 0.001, 0.1, 0.0001, 0.0063),
    "load_change_per_hour": ("Kalman load change", "mdi:sprout", "%/h", 1, 200, 1, 10),
    # Safety
    "temperature_hysteresis": ("Safety temperature hysteresis", "mdi:thermometer-lines", "K", 0, 5, 0.1, 0.5),
    "humidity_hysteresis": ("Safety humidity hysteresis", "mdi:water-percent", "%", 0, 20, 0.5, 3),
    "all_clear_after": ("Safety all clear after", "mdi:bell-check-outline", "min", 1, 1440, 1, 60),
    # Fallback controller
    "fallback_time_constant": ("Fallback time constant", "mdi:timer-outline", "min", 0.5, 10, 0.5, 2),
    "fallback_rate": ("Fallback controller rate", "mdi:speedometer", "%/min", 1, 100, 1, 10),
    "limit_finder_cost": ("Limit finder fan cost", "mdi:scale-balance", None, 0, 20, 0.1, 4),
    "limit_finder_step": ("Limit finder test step", "mdi:stairs", "%", 5, 50, 1, 15),
    "fallback_smoothing": ("Fallback VPD smoothing", "mdi:chart-bell-curve-cumulative", "s", 10, 300, 10, 30),
    "limit_test_margin": ("Limit test margin", "mdi:plus-minus-variant", "kPa", 0, 0.1, 0.001, 0.01),
    "limit_test_max_pause": ("Limit test max. pause", "mdi:timer-pause-outline", "min", 5, 480, 5, 60),
}

# key: (default name, icon, restore mode, entity category)
SWITCHES = {
    "control": ("VPD control", "mdi:fan-auto", "RESTORE_DEFAULT_OFF", None),
    "tent_open": ("Tent open", "mdi:door-open", "ALWAYS_OFF", None),
    "leaf_sensor": ("Leaf temperature from sensor", "mdi:leaf", "RESTORE_DEFAULT_ON", ENTITY_CATEGORY_CONFIG),
}

# Optional switch "use fallback controller": only created if configured
CONF_FORCE_FALLBACK = "force_fallback"

# key: (icon, unit, accuracy, state class, entity category) - all optional
SENSORS = {
    "control_vpd": ("mdi:target-variant", "kPa", 3, None, None),
    "target_active": ("mdi:target", "kPa", 2, None, ENTITY_CATEGORY_DIAGNOSTIC),
    "controller_output": ("mdi:fan", "%", 1, STATE_CLASS_MEASUREMENT, ENTITY_CATEGORY_DIAGNOSTIC),
    "sensible_max": ("mdi:fan-chevron-up", "%", 1, STATE_CLASS_MEASUREMENT, ENTITY_CATEGORY_DIAGNOSTIC),
    "fan_output": ("mdi:fan", "%", 1, STATE_CLASS_MEASUREMENT, None),
    "excess": ("mdi:water-plus", "kPa", 4, STATE_CLASS_MEASUREMENT, ENTITY_CATEGORY_DIAGNOSTIC),
    "excess_target": ("mdi:water-check", "kPa", 4, STATE_CLASS_MEASUREMENT, ENTITY_CATEGORY_DIAGNOSTIC),
    "moisture_load": ("mdi:sprout", "kPa", 4, STATE_CLASS_MEASUREMENT, ENTITY_CATEGORY_DIAGNOSTIC),
    "next_step_benefit": ("mdi:trending-up", "kPa", 4, STATE_CLASS_MEASUREMENT, ENTITY_CATEGORY_DIAGNOSTIC),
    "vpd_at_max": ("mdi:arrow-collapse-up", "kPa", 3, STATE_CLASS_MEASUREMENT, ENTITY_CATEGORY_DIAGNOSTIC),
    # Fallback controller
    "limit_finder_drift": ("mdi:chart-line-variant", "kPa/min", 4, None, ENTITY_CATEGORY_DIAGNOSTIC),
    "limit_finder_vpd_change": ("mdi:delta", "kPa", 3, None, ENTITY_CATEGORY_DIAGNOSTIC),
    "limit_finder_cost_before": ("mdi:scale-balance", None, 2, None, ENTITY_CATEGORY_DIAGNOSTIC),
    "limit_finder_cost_after": ("mdi:scale-balance", None, 2, None, ENTITY_CATEGORY_DIAGNOSTIC),
}

# Only used by the Kalman controller: not created (numbers) or rejected
# (sensors, tuning) when no room sensor is configured
KALMAN_ONLY = {
    "vpd_sacrifice", "speed",
    "excess", "excess_target", "moisture_load", "next_step_benefit", "vpd_at_max",
    "room_fallback_delay", "room_return_delay", "light_memory_after", "light_memory_delay",
    "sensible_max_rate", "sensor_noise", "load_change_per_hour",
    CONF_FORCE_FALLBACK,
}

# key: (device class, icon) - all optional
BINARY_SENSORS = {
    "temperature_protection": (DEVICE_CLASS_HEAT, None),
    "humidity_protection": (DEVICE_CLASS_MOISTURE, None),
    "fallback_active": (None, "mdi:swap-horizontal"),
}


def _number_schema(key):
    name, icon, unit, lo, hi, step, initial, category = NUMBERS[key]
    kwargs = {"icon": icon}
    if unit is not None:
        kwargs["unit_of_measurement"] = unit
    if category is not None:
        kwargs["entity_category"] = category
    schema = number.number_schema(VpdNumber, **kwargs).extend(
        {
            cv.Optional(CONF_MIN_VALUE, default=lo): cv.float_,
            cv.Optional(CONF_MAX_VALUE, default=hi): cv.float_,
            cv.Optional(CONF_STEP, default=step): cv.positive_float,
            cv.Optional(CONF_INITIAL_VALUE, default=initial): cv.float_,
            cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
        }
    )
    return cv.Optional(key, default={CONF_NAME: name}), schema


def _switch_schema(key):
    name, icon, restore, category = SWITCHES[key]
    kwargs = {"icon": icon, "default_restore_mode": restore}
    if category is not None:
        kwargs["entity_category"] = category
    return cv.Optional(key, default={CONF_NAME: name}), switch.switch_schema(VpdSwitch, **kwargs)


def _sensor_schema(key):
    icon, unit, accuracy, state_class, category = SENSORS[key]
    kwargs = {"icon": icon, "accuracy_decimals": accuracy}
    if unit is not None:
        kwargs["unit_of_measurement"] = unit
    if state_class is not None:
        kwargs["state_class"] = state_class
    if category is not None:
        kwargs["entity_category"] = category
    return cv.Optional(key), sensor.sensor_schema(**kwargs)


def _tuning_schema(key):
    name, icon, unit, lo, hi, step, default = TUNING[key]
    kwargs = {"icon": icon, "entity_category": ENTITY_CATEGORY_CONFIG}
    if unit is not None:
        kwargs["unit_of_measurement"] = unit
    entity = number.number_schema(VpdNumber, **kwargs).extend(
        {
            cv.Optional(CONF_MIN_VALUE, default=lo): cv.float_,
            cv.Optional(CONF_MAX_VALUE, default=hi): cv.float_,
            cv.Optional(CONF_STEP, default=step): cv.positive_float,
            cv.Optional(CONF_INITIAL_VALUE, default=default): cv.float_,
            cv.Optional(CONF_MODE, default="BOX"): cv.enum(number.NUMBER_MODES, upper=True),
        }
    )
    in_range = cv.float_range(min=lo, max=hi)

    def validate(value):
        if isinstance(value, dict):
            # Entity: the default name unless one is given
            return entity({CONF_NAME: name, **value})
        if unit in ("s", "min") and isinstance(value, str):
            try:
                value = float(value)
            except ValueError:
                ms = cv.positive_time_period_milliseconds(value).total_milliseconds
                value = ms / (60000.0 if unit == "min" else 1000.0)
        return in_range(value)

    return cv.Optional(key), validate


def _storage_key(name):
    # Same key as a "globals:" entry with restore_value: true and this id
    return int(hashlib.md5(name.encode()).hexdigest()[:8], 16) ^ 1944399030


STORAGE_KEYS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_CONTROLLER_OUTPUT): cv.string_strict,
        cv.Optional(CONF_DAY): cv.string_strict,
        cv.Optional(CONF_NIGHT_KEY): cv.string_strict,
    }
)

_schema = {
    cv.GenerateID(): cv.declare_id(VpdKalman),
    cv.Required(CONF_OUTPUT): cv.use_id(output.FloatOutput),
    cv.Required(CONF_NIGHT): cv.use_id(binary_sensor.BinarySensor),
    cv.Required(CONF_TEMPERATURE): cv.use_id(sensor.Sensor),
    cv.Required(CONF_HUMIDITY): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_LEAF_TEMPERATURE): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_ROOM_TEMPERATURE): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_ROOM_HUMIDITY): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_EXTERNAL_CLIMATE): cv.returning_lambda,
    cv.Optional(CONF_LANGUAGE, default="en"): cv.enum(LANGUAGES, lower=True),
    cv.Optional(CONF_AIRFLOW_AT_ZERO, default=0.2): cv.float_range(min=0.0, max=0.95),
    cv.Optional(CONF_TIME_CONSTANT, default="21s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_LEAF_MAX_DEVIATION, default=6.0): cv.positive_float,
    cv.Optional(CONF_STORAGE_KEYS, default={}): STORAGE_KEYS_SCHEMA,
    cv.Optional(CONF_TUNING, default={}): cv.Schema(dict(_tuning_schema(k) for k in TUNING)),
    cv.Optional(CONF_ON_MESSAGE): automation.validate_automation({}),
}
for _key in NUMBERS:
    _k, _v = _number_schema(_key)
    _schema[_k] = _v
for _key in SWITCHES:
    _k, _v = _switch_schema(_key)
    _schema[_k] = _v
for _key in SENSORS:
    _k, _v = _sensor_schema(_key)
    _schema[_k] = _v
_schema[cv.Optional(CONF_FORCE_FALLBACK)] = switch.switch_schema(
    VpdSwitch,
    icon="mdi:swap-horizontal",
    default_restore_mode="RESTORE_DEFAULT_OFF",
    entity_category=ENTITY_CATEGORY_CONFIG,
)
_schema[cv.Optional("state")] = text_sensor.text_sensor_schema(
    icon="mdi:state-machine", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
)
for _key, (_cls, _icon) in BINARY_SENSORS.items():
    _kwargs = {"entity_category": ENTITY_CATEGORY_DIAGNOSTIC}
    if _cls is not None:
        _kwargs["device_class"] = _cls
    if _icon is not None:
        _kwargs["icon"] = _icon
    _schema[cv.Optional(_key)] = binary_sensor.binary_sensor_schema(**_kwargs)


def _is_fallback(config):
    return CONF_ROOM_TEMPERATURE not in config


def _check_mode(config):
    # Without room sensor only the fallback controller runs. Numbers have
    # defaults and are simply not created, diagnostic sensors and tuning
    # parameters are only there if the user added them, so they are most
    # likely a mistake.
    if not _is_fallback(config):
        return config
    for key in [*SENSORS, CONF_FORCE_FALLBACK]:
        if key in config and key in KALMAN_ONLY:
            raise cv.Invalid(
                f"'{key}' needs room_temperature and room_humidity (Kalman controller)", path=[key]
            )
    for key in config[CONF_TUNING]:
        if key in KALMAN_ONLY:
            raise cv.Invalid(
                f"'{key}' needs room_temperature and room_humidity (Kalman controller)", path=[CONF_TUNING, key]
            )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(_schema).extend(cv.COMPONENT_SCHEMA),
    cv.has_none_or_all_keys(CONF_ROOM_TEMPERATURE, CONF_ROOM_HUMIDITY),
    _check_mode,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_output(await cg.get_variable(config[CONF_OUTPUT])))
    cg.add(var.set_night(await cg.get_variable(config[CONF_NIGHT])))
    cg.add(var.set_temperature(await cg.get_variable(config[CONF_TEMPERATURE])))
    cg.add(var.set_humidity(await cg.get_variable(config[CONF_HUMIDITY])))
    if CONF_LEAF_TEMPERATURE in config:
        cg.add(var.set_leaf_temperature(await cg.get_variable(config[CONF_LEAF_TEMPERATURE])))
    fallback = _is_fallback(config)
    # With room sensor: Kalman controller, fallback while it fails.
    # Without: fallback controller only (see vpd_controller_core.h)
    if not fallback:
        cg.add(var.set_room_temperature(await cg.get_variable(config[CONF_ROOM_TEMPERATURE])))
        cg.add(var.set_room_humidity(await cg.get_variable(config[CONF_ROOM_HUMIDITY])))
    if CONF_EXTERNAL_CLIMATE in config:
        lam = await cg.process_lambda(config[CONF_EXTERNAL_CLIMATE], [], return_type=ExternalClimate)
        cg.add(var.set_external_climate(lam))

    cg.add(var.set_language(config[CONF_LANGUAGE]))
    cg.add(var.set_airflow_at_zero(config[CONF_AIRFLOW_AT_ZERO]))
    if not fallback:
        cg.add(var.set_time_constant(config[CONF_TIME_CONSTANT]))
    cg.add(var.set_leaf_max_deviation(config[CONF_LEAF_MAX_DEVIATION]))

    keys = config[CONF_STORAGE_KEYS]
    base = config[CONF_ID].id
    cg.add(
        var.set_storage_keys(
            _storage_key(keys.get(CONF_CONTROLLER_OUTPUT, f"{base}_controller_output")),
            _storage_key(keys.get(CONF_DAY, f"{base}_day")),
            _storage_key(keys.get(CONF_NIGHT_KEY, f"{base}_night")),
        )
    )

    for key, conf in config[CONF_TUNING].items():
        index = getattr(vpd_kalman_ns, f"TUNING_{key.upper()}")
        if isinstance(conf, dict):
            num = await number.new_number(
                conf, min_value=conf[CONF_MIN_VALUE], max_value=conf[CONF_MAX_VALUE], step=conf[CONF_STEP]
            )
            await cg.register_component(num, conf)
            cg.add(num.set_initial_value(conf[CONF_INITIAL_VALUE]))
            cg.add(var.set_tuning_number(index, num))
        else:
            cg.add(var.set_tuning(index, conf))

    skip = KALMAN_ONLY if fallback else set()
    for key in NUMBERS:
        if key in skip:
            continue
        conf = config[key]
        num = await number.new_number(
            conf, min_value=conf[CONF_MIN_VALUE], max_value=conf[CONF_MAX_VALUE], step=conf[CONF_STEP]
        )
        await cg.register_component(num, conf)
        cg.add(num.set_initial_value(conf[CONF_INITIAL_VALUE]))
        cg.add(getattr(var, f"set_{key}_number")(num))

    for key in SWITCHES:
        if key in skip:
            continue
        conf = config[key]
        sw = await switch.new_switch(conf)
        await cg.register_component(sw, conf)
        cg.add(getattr(var, f"set_{key}_switch")(sw))

    if conf := config.get(CONF_FORCE_FALLBACK):
        sw = await switch.new_switch(conf)
        await cg.register_component(sw, conf)
        cg.add(var.set_force_fallback_switch(sw))

    for key in SENSORS:
        if conf := config.get(key):
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(var, f"set_{key}_sensor")(sens))

    if conf := config.get("state"):
        ts = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_state_text_sensor(ts))

    for key in BINARY_SENSORS:
        if conf := config.get(key):
            bs = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(var, f"set_{key}_binary_sensor")(bs))

    for conf in config.get(CONF_ON_MESSAGE, []):
        await automation.build_callback_automation(
            var,
            "add_on_message_callback",
            [(cg.std_string, "title"), (cg.std_string, "message")],
            conf,
        )
