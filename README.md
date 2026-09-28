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

## Why a dedicated VPD controller?

VPD decides how much water the plants evaporate: too low and they barely
transpire, take up few nutrients and invite mould, too high and they close
their stomata. In a tent the exhaust fan is usually the only thing that moves
it, so the fan controller decides how close you get to the target, and how
much electricity and noise that costs.

A normal PID controller on "VPD -> fan" works on paper, but a grow tent
breaks several of its assumptions:

| Problem in the tent | What a PID does | What this controller does |
|---|---|---|
| **The fan acts very unevenly.** The humidity excess falls with `1 / airflow`: 10 -> 20 % fan changes a lot, 80 -> 90 % hardly anything. The tent also reacts much more slowly at low fan (`tau / airflow`). | Fixed gains fit only one operating point: too aggressive at low speed (oscillates), too sluggish at high speed. | Works on a logarithmic airflow scale with a gain that follows the airflow (`k = speed * q / tau`), so it behaves the same at every speed. |
| **The target is often unreachable**, e.g. when the room air is humid or the plants transpire a lot. | The integral winds up and the fan runs at 100 % for hours, for a few hundredths of a kPa. | Estimates the moisture load and stops at the **sensible maximum**, where more fan would gain less VPD than you are willing to give up. Saves a lot of energy, especially in winter. |
| **The room air enters 1:1.** Every change in the room shows up in the tent. | Only notices it once the VPD has already moved, then over- or undershoots. | Measures the room air and works with the vapour pressure excess `tent - room`, so room changes do not look like controller errors. |
| **Light changes are big, predictable jumps** in temperature and transpiration. | Chases the jump from the old fan speed, overshoots and needs a long time to settle. | Jumps straight to the fan speed it remembered for this light phase and moves the target smoothly (*setpoint transition*) instead of fighting a gap that closes on its own. |
| **Noisy sensors.** | The derivative term amplifies the noise, so it is often switched off. | A Kalman filter separates sensor noise from real load changes and limits outliers. |
| **Tent open, sensor failures.** | Winds up on meaningless readings. | Holds the fan while the tent is open, falls back to a controller without room sensor while the room sensor fails, runs an emergency speed if the tent sensor fails. |
| **Too hot or too humid.** | Has only one goal. | A separate safety layer overrides the controller above temperature and humidity limits and reports it. |

In short: a PID only reacts to the deviation. This controller also knows
*why* the VPD deviates (room air, plant load, light phase) and whether more
fan would actually help. That is what gets the VPD close to the target
without running the fan at full speed for nothing.

## What it is

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
- **Fallback controller** that only needs the tent sensor: it takes over
  automatically while the room sensor fails, or runs alone if you have no
  room sensor, see [Fallback controller](#fallback-controller)
- Every setting is a Home Assistant entity, every internal value can be
  exposed as a diagnostic sensor
- Optional [tuning parameters](#tuning-parameters-optional) for other setups,
  as fixed values or as entities, only if you add them
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
| `room_temperature`, `room_humidity` | no | Air the fan draws in. Both or none. With them the Kalman controller runs, and the [fallback controller](#fallback-controller) takes over while they fail. Without them only the fallback controller runs. |
| `external_climate` | no | Lambda returning `vpd_kalman::ExternalClimate` with `temperature`, `humidity`, `leaf_temperature` (and `selected` for the log). If all three are valid, they are used instead of the own sensors, e.g. for a second sensor at canopy height. See below. |

### Options

| Key | Default | Description |
|---|---|---|
| `airflow_at_zero` | `0.2` | Airflow of the fan at 0 % relative to 100 % (`q0`). Enters the moisture load and the sensible maximum. The fallback controller uses it for its log fan scale. |
| `time_constant` | `21s` | Time the tent needs at 100 % fan until 63 % of a humidity change is exhausted (`tau`). Sets the filter dynamics and, together with *Controller speed*, how fast the controller acts. Kalman controller only. |
| `leaf_max_deviation` | `6.0` | Leaf sensor values further than this (°C) from the air temperature count as measurement errors. |
| `language` | `en` | `en` or `de`: state texts, notifications and log lines. |
| `storage_keys` | - | Only for migrating from a `globals:` setup, see below. |
| `tuning` | - | Optional tuning parameters, see [below](#tuning-parameters-optional). |
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
| `tent_open` | Tent open | always off | Holds the fan, pauses filter, remembering and limit finder, safety stays active. |
| `leaf_sensor` | Leaf temperature from sensor | default on | Use `leaf_temperature` instead of the offsets. |
| `force_fallback` | - (optional, only with `name`) | default off | ON = the [fallback controller](#fallback-controller) runs although the room sensor works, e.g. to compare both controllers. Only with room sensor. |

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
| `fallback_active` | binary | ON while the fallback controller drives the fan. |
| `limit_finder_drift` | kPa/min | Fallback: VPD drift measured before the last test step. |
| `limit_finder_vpd_change` | kPa | Fallback: effect of the last test step, without the drift. |
| `limit_finder_cost_before` | - | Fallback: `J` before the last test step. |
| `limit_finder_cost_after` | - | Fallback: `J` after the last test step (with margin). |

---

## Fallback controller

The Kalman controller needs the room air. For the case that it is missing,
the component contains a second, simpler controller that only needs the tent
sensor. It is always part of the firmware:

- **Room sensor configured:** the Kalman controller runs. If the room sensor
  has no valid value for 2 min, the fallback controller takes over. After
  1 min of valid values again, the Kalman controller takes over again. Both
  switches send a notification (`on_message`), the state text gets
  *(no room sensor)* while the fallback runs. Shorter dropouts only hold the
  fan.
- **Switch `force_fallback` (optional):** ON hands over to the fallback
  controller right away, the room sensor keeps recording, so both
  controllers can be compared on the same data. OFF hands back right away
  (after the return delay if the room sensor is missing just then). No
  notification, the state text gets *(fallback)*.

  ```yaml
  vpd_kalman:
    # ...
    force_fallback:
      name: "Use fallback controller"
  ```
- **No room sensor configured:** only the fallback controller runs.

  ```yaml
  vpd_kalman:
    output: fan_pwm
    night: lights_off
    temperature: tent_temperature
    humidity: tent_humidity
  ```

On a switch the fan is taken over without a jump, safety episodes, *Tent
open* and the remembered events carry on. The fallback starts its upper
limit at the last sensible maximum of the Kalman controller. Back in the
Kalman controller, the filter is re-synchronised to the measurement and keeps
the estimated load.

Without the room air the fallback cannot estimate the moisture load. It
works in three levels, every 10 s:

1. **Base controller**: a PI controller on a log fan scale,
   `x = ln(u + u0)` with `u0 = 100 * q0 / (1 - q0)` from `airflow_at_zero`.
   Equal steps in `x` are equal relative airflow changes. Nothing happens
   inside the deadband.
2. **Limit finder**: if the VPD stays below the band while the fan sits at
   its upper limit for 3 time constants, it tries out whether a lower (or
   higher) limit is better. It measures the VPD drift, moves the limit by one
   test step, waits 3 time constants and compares
   `J = (deviation / 0.1 kPa)^2 + fan cost * (fan / 100 %)^2` before and
   after, with the drift taken out. The step is kept if `J` gets clearly
   smaller (margin 0.01 kPa), otherwise the limit goes back and the next test
   waits twice as long (max. 60 min). This is the fallback's version of the
   sensible maximum. The limit is reset to *Fan maximum automatic* after a
   light change, a target change, a tent sensor change and when the control
   is switched on.
3. **Safety**: the same as in the Kalman controller.

The controlled VPD is the mean of the last 30 s. *Tent open*, the
*setpoint transition* (after light changes, closing the tent and a tent
sensor change), manual mode, notifications, leaf temperature and
`external_climate` work as in the Kalman controller. Its settings (time
constant, rate, fan cost, test step) are [tuning parameters](#tuning-parameters-optional).

Without room sensor the settings *Allowed VPD sacrifice* and *Controller
speed* are not created and `time_constant` is ignored. The Kalman-only
diagnostics (`excess`, `excess_target`, `moisture_load`,
`next_step_benefit`, `vpd_at_max`) and tuning parameters are rejected.

## Tuning parameters (optional)

For setups that differ from the defaults. Leave a parameter out and the
built-in default applies, no entity is created. A plain value fixes it (for
times also `90s`, `2min`), a block with `name` turns it into a number entity
under *Configuration* that you can change live (it accepts the usual number
options, `initial_value` defaults to the default):

```yaml
vpd_kalman:
  # ...
  tuning:
    room_fallback_delay: 5min          # fixed
    sensor_noise: 0.01                 # fixed
    fallback_rate:                     # entity
      name: "Fallback controller rate"
```

| Key | Unit | Default | Range | Controller | Meaning |
|---|---|---|---|---|---|
| `room_fallback_delay` | min | 2 | 0.5-60 | switch | How long the room sensor may be missing before the fallback takes over. |
| `room_return_delay` | min | 1 | 0.5-60 | switch | How long the room sensor must deliver again before the Kalman controller is back. |
| `light_memory_after` | min | 20 | 0-240 | Kalman | Time after a light change before the fan of this light phase is remembered. |
| `light_memory_delay` | min | 5 | 0.5-60 | Kalman | The value remembered is the one from this long ago, so the late end of a light phase does not spoil it. |
| `boot_wait_tent` | s | 60 | 10-600 | both | After boot, hold the fan this long while no tent value has arrived. |
| `boot_wait_night` | s | 180 | 10-1800 | both | After boot, hold the fan this long while day/night is unknown. |
| `sensible_max_rate` | %/min | 5 | 0.1-100 | Kalman | How fast the sensible maximum may change. |
| `sensor_noise` | kPa | 0.0063 | 0.001-0.1 | Kalman | Measurement noise of the vapour pressure excess. Higher = the filter trusts single readings less. |
| `load_change_per_hour` | %/h | 10 | 1-200 | Kalman | How fast the moisture load may change. Higher = the load estimate follows faster but noisier. |
| `temperature_hysteresis` | K | 0.5 | 0-5 | both | The temperature safety switches off only this far below its limit. |
| `humidity_hysteresis` | % | 3 | 0-20 | both | The same for the humidity safety. |
| `all_clear_after` | min | 60 | 1-1440 | both | A safety episode ends (all clear notification) after this long without triggering. |
| `fallback_time_constant` | min | 2 | 0.5-10 | fallback | How fast the VPD reacts to a fan change. Sets the P part and all limit finder times (3 time constants each). |
| `fallback_rate` | %/min | 10 | 1-100 | fallback | Relative fan change per minute at 0.1 kPa outside the deadband. |
| `limit_finder_cost` | - | 4 | 0-20 | fallback | Weight of the fan in `J`. Higher = the limit is lowered more readily. |
| `limit_finder_step` | % | 15 | 5-50 | fallback | Size of one test step, relative to the airflow. |
| `fallback_smoothing` | s | 30 | 10-300 | fallback | Averaging time of the controlled VPD. |
| `limit_test_margin` | kPa | 0.01 | 0-0.1 | fallback | How clearly a test step has to be better to be kept. |
| `limit_test_max_pause` | min | 60 | 5-480 | fallback | Longest waiting time between two tests after discarded steps. |

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
