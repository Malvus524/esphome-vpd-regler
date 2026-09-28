# esphome-vpd-regler

> [!WARNING]
> **This repository was created entirely with AI.** The code, the
> documentation and the tests were written by an AI assistant.
>
> I have tested it myself in my own grow tent, but my tests cannot cover
> every setup, fan, sensor and failure case. Use it with care: watch the
> controller closely at the beginning, keep the safety limits set sensibly
> and do not leave it unsupervised where a wrong fan setting could harm your
> plants or equipment. It comes without any warranty (see [LICENSE](LICENSE)).

An [ESPHome](https://esphome.io) external component that controls the
**VPD (vapour pressure deficit)** of a grow tent with a single exhaust fan.

- Model-based controller with a **Kalman filter** that estimates how much
  moisture the plants add, instead of a PID
- Finds the **sensible maximum**: it stops speeding up the fan once more air
  would gain less VPD than you are willing to give up (saves energy and noise,
  especially in winter)
- Separate **day and night targets**, smooth **setpoint transition** after
  light changes, remembered fan speed per light phase
- **Temperature and humidity safety** that overrides the controller, with
  one notification per episode
- **Tent open** switch that pauses the controller while you work in the tent
- Leaf temperature from an IR sensor (e.g. MLX90614) or a fixed day/night offset
- Optional second tent sensor (e.g. over ESP-NOW) via a lambda
- Every setting is a Home Assistant entity, every internal value can be
  exposed as a diagnostic sensor
- State texts and notifications in English or German

The fan is the only thing it drives. Humidifiers, heaters or lamp dimming
belong in their own components and can read this one's entities.

---

## How it works

The tent trails the room air. The controller works with vapour pressure
(kPa), which stays the same when the intake air warms up:

```
e_tent = e_room + E           E  = vapour pressure excess of the tent (kPa)
E      = L / q(u)             L  = moisture load (transpiration)
q(u)   = q0 + (1 - q0) * u    q  = relative airflow, q0 = airflow at 0 %
```

Room changes pass straight into the tent, the fan divides the excess `E` by
the airflow. The tent follows with the time constant `tau / q`.

Every 10 s the controller

1. measures tent, room and leaf temperature,
2. runs a Kalman filter on `E` that estimates the load `L`,
3. derives the **sensible maximum** from `L`: the fan setting at which full
   fan would only gain the *allowed VPD sacrifice* more,
4. moves the fan on a log scale towards the band edge
   (`ln q += k * dt * ln(E / E_edge)`, with `k = speed * q / tau`) and does
   nothing inside the deadband,
5. applies the safety: `output = max(controller, temperature P, humidity P)`.

After a light change and after closing the tent, the effective target starts
at the measured VPD and moves to the target within the *setpoint transition*
time, so the fan does not fight a gap that heating up or cooling down closes
on its own.

The direction is fixed: more exhaust means drier air and a higher VPD. If
your room air is wetter than the tent air, this controller is not for you.

---

## Installation

```yaml
external_components:
  - source: github://Malvus524/esphome-vpd-regler@v1.0.0
    components: [vpd_kalman]
```

Pin a tag (`@v1.0.0`) so updates only arrive when you choose.

## Minimal configuration

```yaml
vpd_kalman:
  output: fan_pwm              # float output driving the fan
  night: lights_off            # binary sensor, ON = night
  temperature: tent_temperature
  humidity: tent_humidity
  room_temperature: room_temperature
  room_humidity: room_humidity
```

This creates all settings and switches with English names. The controller
starts in **manual mode**: turn on the switch *VPD control* to let it drive
the fan. A complete example with sensors, diagnostics and notifications is in
[example.yaml](example.yaml).

---

## Configuration reference

### Inputs

| Key | Required | Description |
|---|---|---|
| `output` | yes | Float output of the fan (e.g. `ledc`). The controller writes 1-100 %, it never switches the fan off. Use `min_power`/`max_power` of the output to map that to your fan. |
| `night` | yes | Binary sensor, ON = night (lights off). Day/night selects the target, the leaf offset and the remembered fan speed. Until it has a state after boot, the controller holds the fan (max. 3 min). |
| `temperature`, `humidity` | yes | Tent air sensor. |
| `leaf_temperature` | no | Leaf temperature sensor (IR). Used while the switch *Leaf temperature from sensor* is on and the value is within `leaf_max_deviation` of the air temperature. Otherwise air temperature + leaf offset day/night. |
| `room_temperature`, `room_humidity` | yes | Air the fan draws in. Without it the controller holds the fan. |
| `external_climate` | no | Lambda returning `vpd_kalman::ExternalClimate` with `temperature`, `humidity`, `leaf_temperature` (and `selected` for the log). If all three are valid, they are used instead of the own sensors, e.g. for a second sensor at canopy height. See below. |

### Options

| Key | Default | Description |
|---|---|---|
| `airflow_at_zero` | `0.2` | Airflow of the fan at 0 % relative to 100 % (`q0`). Enters the moisture load and the sensible maximum. |
| `time_constant` | `21s` | Time the tent needs at 100 % fan until 63 % of a humidity change is exhausted (`tau`). Sets the filter dynamics and, together with *Controller speed*, how fast the controller acts. |
| `leaf_max_deviation` | `6.0` | Leaf sensor values further than this (°C) from the air temperature count as measurement errors. |
| `language` | `en` | `en` or `de`: state texts, notifications and log lines. |
| `storage_keys` | - | Only for migrating from a `globals:` setup, see below. |
| `on_message` | - | Automation with `title` and `message` (`std::string`) for notifications. |

The controller runs every 10 s. This is fixed, because the filter and the
control law are tuned for it.

### Settings (number entities)

All are created automatically, stored in flash and shown as boxes. Each
accepts the usual number options (`name`, `id`, `icon`, `entity_category`,
...) plus `min_value`, `max_value`, `step` and `initial_value`.

| Key | Default name | Default | Range | Meaning |
|---|---|---|---|---|
| `manual_speed` | Fan manual speed | 50 % | 1-100 | Fan speed while *VPD control* is off. |
| `target_day` | VPD target day | 1.1 kPa | 0.3-2.5 | |
| `target_night` | VPD target night | 0.9 kPa | 0.3-2.5 | |
| `deadband` | VPD deadband | 0.05 kPa | 0.01-0.5 | Target +/- this counts as reached, the fan stays where it is. |
| `vpd_sacrifice` | Allowed VPD sacrifice | 0.03 kPa | 0-0.3 | How much VPD the controller may give up to save fan. 0 = run up to *Fan maximum automatic*. |
| `speed` | Controller speed | 0.25 | 0.05-1 | 0.25 is critically damped according to the model. Higher = faster, but may oscillate if the tent is slower than `time_constant`. |
| `transition` | Setpoint transition | 45 min | 0-180 | Duration of the setpoint transition. 0 = off. |
| `tent_open_max` | Tent open max. duration | 30 min | 5-240 | *Tent open* switches itself off after this time. |
| `fan_min` | Fan minimum automatic | 5 % | 1-100 | |
| `fan_max` | Fan maximum automatic | 100 % | 1-100 | |
| `fan_emergency` | Fan emergency speed | 40 % | 1-100 | Used when the tent sensor fails, and the minimum at which the safety starts. |
| `temperature_max` | Safety temperature max | 28 °C | 20-40 | Above this at least *Fan emergency speed*, rising linearly to 100 % at max + P-band. |
| `temperature_band` | Safety temperature P-band | 3 K | 0.5-10 | |
| `humidity_max` | Safety humidity max | 75 % | 40-95 | Must be above the humidity your VPD target results in. |
| `humidity_band` | Safety humidity P-band | 10 % | 1-30 | |
| `leaf_offset_day` | Leaf offset day | -2.0 °C | -10-5 | Leaf temperature = air + offset, when no valid leaf sensor value is used. |
| `leaf_offset_night` | Leaf offset night | -1.0 °C | -10-5 | |

### Switches

| Key | Default name | Restore | Meaning |
|---|---|---|---|
| `control` | VPD control | default off | ON = the controller drives the fan (takes over the current speed without a jump). OFF = manual speed. |
| `tent_open` | Tent open | always off | Holds the fan, pauses filter and remembering, safety stays active. |
| `leaf_sensor` | Leaf temperature from sensor | default on | Use `leaf_temperature` instead of the offsets. |

### Diagnostics (optional)

Only created if you add the key (with at least a `name`). All accept the
usual sensor options.

| Key | Unit | Meaning |
|---|---|---|
| `control_vpd` | kPa | VPD the controller works with (unfiltered). |
| `target_active` | kPa | Effective target, including the setpoint transition. |
| `controller_output` | % | What the controller wants (before safety). |
| `sensible_max` | % | Highest useful fan setting, see *Allowed VPD sacrifice*. |
| `fan_output` | % | What is applied to the output. |
| `excess` | kPa | Vapour pressure excess tent - room. |
| `excess_target` | kPa | Excess the target requires. 0 or negative = unreachable with this room air. |
| `moisture_load` | kPa | Kalman estimate of the load, proportional to transpiration. |
| `next_step_benefit` | kPa | VPD gain of the next fan step (+15 % airflow). |
| `vpd_at_max` | kPa | VPD that *Fan maximum automatic* would reach at the same temperature. |
| `state` | text | E.g. *In band*, *Regulating*, *At sensible maximum*, *Target unreachable*, *Safety (temperature)*, *Tent open - paused (5 min)*. |
| `temperature_protection` | binary | ON while the temperature safety raises the fan. |
| `humidity_protection` | binary | ON while the humidity safety raises the fan. |

---

## Notifications

```yaml
vpd_kalman:
  # ...
  on_message:
    - homeassistant.action:
        action: notify.notify
        data:
          title: !lambda 'return title;'
          message: !lambda 'return message;'
```

Sent when the temperature or humidity safety triggers (once per episode,
all clear after 60 min without triggering) and when *Tent open* ends
automatically. For `homeassistant.action`, allow the device to perform Home
Assistant actions in the ESPHome integration options.

## Second tent sensor (`external_climate`)

For example a sensor at canopy height on another ESP, received via
ESP-NOW or `packet_transport`. Return NAN for missing values, the
controller then falls back to its own sensors for that tick. When the source
changes, the filter is re-synchronised and a setpoint transition starts,
because two sensors at different places measure different VPDs.

```yaml
switch:
  - platform: template
    id: use_canopy_sensor
    name: "Control with canopy sensor"
    optimistic: true
    restore_mode: RESTORE_DEFAULT_ON

vpd_kalman:
  # ...
  external_climate: !lambda |-
    vpd_kalman::ExternalClimate c;
    c.selected = id(use_canopy_sensor).state;
    if (c.selected) {
      c.temperature = id(canopy_temperature).state;
      c.humidity = id(canopy_humidity).state;
      c.leaf_temperature = id(canopy_temperature).state - 2.0f;
    }
    return c;
```

## Migrating from a YAML lambda

Entities keep their Home Assistant history and stored values if you give
them the same `name` (the flash key is derived from it) and the same `id`
(for your other lambdas). If your old setup stored the controller values in
restoring `globals:`, point `storage_keys` to their ids:

```yaml
vpd_kalman:
  storage_keys:
    controller_output: u_vpd_gespeichert
    day: u_tag_gespeichert
    night: u_nacht_gespeichert
```

## License

[MIT](LICENSE)
