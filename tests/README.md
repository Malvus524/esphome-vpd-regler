# Controller regression tests

Run from the repository root with Python, ESPHome and a C++17 compiler installed:

```sh
python -m unittest discover -s tests -v
g++ -std=c++17 -O2 -I . tests/test_core.cpp -o /tmp/vpd-core-test
/tmp/vpd-core-test
g++ -std=c++17 -O2 -DUSE_VPD_FAN_CURVE -I . tests/test_core.cpp -o /tmp/vpd-learning-test
/tmp/vpd-learning-test
g++ -std=c++17 -O2 -I . tests/simulate_control.cpp -o /tmp/vpd-simulation
/tmp/vpd-simulation
esphome compile tests/configs/fallback.yaml
esphome compile tests/configs/kalman.yaml
esphome compile tests/configs/learning.yaml
```

On Windows, put the executables in `$env:TEMP` instead of `/tmp`.
The firmware configurations use synthetic sensors and a template output;
they are build fixtures, not configurations to flash to a grow controller.

Coverage includes publication timeout and clock wrap, invalid/implausible
measurements, external sample age, ramps in both directions, manual protection
and its opt-out, delayed fallback/recovery, clearing stale learning diagnostics,
learning persistence validation, YAML parameter constraints, conflicting runtime
fan limits and the memory footprint with learning disabled.

The closed-loop baseline simulates a first-order moisture plant at constant
temperature, with a load step. It checks convergence to the configured deadband
without saturation. A second simulation checks that an unreachable target runs
at 100 % with zero sacrifice, but below 50 % with a 0.03 kPa allowance in the
specified low-load plant. It does not establish robustness against all real sensor
delays, nonlinear fans, coupled temperature changes or disturbances. Hardware
measurements are still needed before claiming general tuning quality.

The additional scenario runner covers both controllers with noise, isolated
spikes, sensor lag, transport delay and a faster main-controller setting.
It reports integrated error, error outside the target band, fan travel, peak
error, final average error and time above 90% fan output. Assertions check
convergence, bounded transients, absence of prolonged saturation, and fan travel
in the noise scenario. These are deterministic model regressions, not hardware
validation or evidence of stability for every installation.

Core tests also cover isolated versus sustained measurement changes, history
reset after invalid input, trend anticipation, bounded feedback increases and
immediate protection followed by gradual release, including manual interruption
and repeated handovers between the main and fallback controllers.
