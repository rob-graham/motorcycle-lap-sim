# Mallala telemetry data contract

## Scope

This is the initial Phase 10 contract for importing the supplied R6 Mallala AiM-derived workbook and preparing a two-dimensional validation dataset. It does not claim that the logger installation, motorcycle, rider, or track georeferencing are fully characterised.

The importer is deliberately separate from the core physics and optimisation modules. Measured data must not become a hidden dependency of the simulator.

## Source workbook used for development

The supplied project workbook identifies:

- session: Mallala;
- vehicle: R6;
- rider/racer label: Coop;
- event: SA State Titles;
- comment: P4 Friday;
- date: 24 May 2024;
- nominal sample rate: 20 Hz; and
- session duration: about 479 s.

The workbook includes an `Updated` sheet containing cached local east/north coordinates plus GPS, IMU and ECU channels, and a `Lap5` sheet containing a selected lap. The project copy also contains `GridConversion` and `References` sheets used to derive local coordinates.

The current import implementation consumes the cached values in `Updated` (or another explicitly selected compatible sheet) with `data_only=True`. It does not recalculate workbook formulas or make the workbook's coordinate conversion a production georeferencing method.

## Canonical imported channels

All numerical simulator/validation channels use SI units internally:

| Canonical field | Source | Internal unit |
| --- | --- | --- |
| `time_s` | Time | s |
| `distance_m` | Distance on GPS Speed | m |
| `east_m`, `north_m` | east, north | m |
| `speed_mps` | GPS Speed | m/s |
| `lateral_acceleration_mps2` | GPS LatAcc | m/s^2 |
| `longitudinal_acceleration_mps2` | GPS LonAcc | m/s^2 |
| `slope_rad` | GPS Slope | rad |
| `heading_rad` | GPS Heading | rad |
| `gps_gyro_radps` | GPS Gyro | rad/s |
| `latitude_deg`, `longitude_deg` | GPS Latitude/Longitude | deg |
| `roll_rate_radps` | RollRate | rad/s |
| `pitch_rate_radps` | PitchRate | rad/s |
| `yaw_rate_radps` | YawRate | rad/s |
| `engine_rpm` | ECU RPM | rpm |
| `gear_number` | ECU GEAR | integer |
| `ecu_throttle_rad` | ECU THROTTLE | rad |
| `hand_throttle_fraction` | ECU TPS HAND | fraction |
| `distance_from_start_m` | Dist from Start | m |
| `marker` | workbook marker column | text/None |
| `lap_id` | workbook lap column | integer |

Acceleration channels expressed in `g` are converted using standard gravity 9.80665 m/s^2. Angle rates expressed in degrees per second are converted to radians per second.

## Lap handling

The initial `lap_slices()` helper reports contiguous positive workbook lap-ID runs. It intentionally does **not** call every run a complete valid lap. Completeness, start/finish consistency, GPS quality and calibration/hold-out selection are Phase 10 validation decisions and must be recorded separately.

For the supplied selected `Lap5` sheet, the cached time range is approximately 377.45 to 449.50 s, i.e. approximately 72.05 s between its first and last samples. This is a useful measured comparison case but remains case-specific.

## Coordinate contract and map matching

The simulator's Mallala reference geometry intentionally omits its original EPSG:7854 fitting coordinates and starts at local `(0, 0)` with heading `0`. The telemetry workbook's east/north frame therefore must **not** be assumed to be the simulator frame.

`Rigid2DTransform` makes the required origin and local-x bearing explicit. Only after that transform is documented should telemetry be passed to `map_match_nearest()`.

The first map matcher is deliberately transparent: it finds the nearest point on a sufficiently fine sampled centreline and reports reference chainage, signed lateral offset, reference distance and sample index. Later interpolation may improve sub-sample precision without changing this coordinate contract.

A verified Mallala world-to-simulator transform is currently a required data item before final measured-line/lateral-error validation can be claimed.

## Roll-response work in Phase 10

The initial roll work begins with source-independent demanded-lean and demanded-roll-rate calculations in `motorcycle.roll`:

- steady planar lean demand: `atan(v^2 * kappa / g)`;
- demanded roll rate: `v * d(phi)/ds`; and
- explicit comparison of demanded rate against a replaceable maximum-roll-rate scenario.

These helpers are not yet a speed-solver constraint. The intended Phase 10 sequence is:

1. import and quality-check the R6 roll-rate channel;
2. check sign, bias, time alignment and filtering assumptions;
3. derive an independent lean/lean-rate estimate from GPS/path quantities;
4. compare logged and reconstructed transition behaviour;
5. introduce a switchable Level-1 roll-rate constraint into the fixed-path solver;
6. verify that disabling the constraint reproduces the frozen Phase 9 baseline; and
7. only then re-optimise the racing line with roll response active.

A lap-time shift toward the measured lap is not sufficient validation. Added time must occur in physically plausible transition zones and improve local speed/line/roll agreement.

## Current limitations to carry explicitly

- rider mass is not known;
- motorcycle modifications are incompletely described;
- logger orientation/axis convention is not fully documented;
- IMU channels require sign/bias/filter/time-alignment checks;
- the Mallala simulator geometry is approximate rather than survey-grade;
- the simulator-local/world transform is not yet recorded in the repository; and
- current work is a Mallala R6 case calibration/validation exercise, not general motorcycle or track validation.
