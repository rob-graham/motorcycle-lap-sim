# Mallala telemetry data contract

## Scope

This is the initial Phase 10 contract for importing the supplied R6 Mallala AiM-derived workbook and preparing a two-dimensional validation dataset. It does not claim that the logger installation, motorcycle, rider, GPS reception, or track georeferencing are fully characterised.

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

All numerical simulator/validation channels use SI units internally. The imported channels include time, GPS-derived distance/east/north/speed/accelerations/slope/heading/gyro/latitude/longitude, IMU roll/pitch/yaw rates, engine RPM, raw ECU gear signal, throttle channels, distance from start, markers and lap identifiers.

Acceleration channels expressed in `g` are converted using standard gravity 9.80665 m/s^2. Angle rates expressed in degrees per second are converted to radians per second. The raw AiM gear channel may contain fractional interpolated values during shifts and is retained as measured rather than forced to integer.

## Lap handling

The initial `lap_slices()` helper reports contiguous positive workbook lap-ID runs. It intentionally does **not** call every run a complete valid lap. Completeness, start/finish consistency, GPS quality and calibration/hold-out selection are Phase 10 validation decisions and must be recorded separately.

For the supplied selected `Lap5` sheet, the cached time range is approximately 377.45 to 449.50 s, i.e. approximately 72.05 s between its first and last samples. This is a useful measured comparison case but remains case-specific.

## GPS-position quality and lap selection

The R6 dataset shows apparent position inconsistencies on some laps/track sectors. These must not automatically be interpreted as errors in the simulator track geometry. A plausible physical explanation is partial GPS antenna masking or degraded satellite geometry at particular motorcycle headings, but that is currently a hypothesis rather than a demonstrated cause.

Phase 10 map matching must therefore treat GPS position as a measured channel with variable quality rather than ground truth. The validation workflow should:

1. compare repeated-lap trajectories sector by sector before fitting the simulator transform;
2. inspect available GPS quality indicators (`GPS Nsat`, `GPS PosAccuracy`, `GPS SpdAccuracy`) together with heading and speed;
3. identify position excursions that are not repeatable between otherwise similar laps;
4. test whether residual magnitude or direction clusters with motorcycle heading, which would support but not prove an antenna-masking explanation;
5. exclude or down-weight demonstrably poor position intervals when fitting the rigid transform and measured racing-line envelope, while keeping those exclusions explicit; and
6. retain speed, IMU and ECU channels from a lap where appropriate even if a local GPS-position segment is unsuitable for line-position validation.

No lap should be labelled globally `good` or `bad` solely from one GPS excursion. Quality flags should preferably be sample- or sector-level so that useful telemetry is not discarded unnecessarily.

## Coordinate contract and map matching

The simulator's Mallala reference geometry intentionally omits its original EPSG:7854 fitting coordinates and starts at local `(0, 0)` with heading `0`. The telemetry workbook's east/north frame therefore must **not** be assumed to be the simulator frame.

`Rigid2DTransform` makes the required origin and local-x bearing explicit. Only after that transform is documented should telemetry be passed to `map_match_nearest()`.

The first map matcher is deliberately transparent: it finds the nearest point on a sufficiently fine sampled centreline and reports reference chainage, signed lateral offset, reference distance and sample index. Later interpolation may improve sub-sample precision without changing this coordinate contract.

The Mallala simulator geometry is approximate rather than survey-grade, but engineering review considers it a good representation for the current work. Consequently, a few-metre local disagreement may contain contributions from both GPS error and reference-track approximation. The fitting process must report residuals and repeated-lap consistency instead of silently warping or scaling the simulator track to make the telemetry fit.

A verified Mallala world-to-simulator transform, with documented fitting samples/exclusions and residuals, is required before final measured-line/lateral-error validation can be claimed.

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
- GPS-position quality varies locally and the cause of apparent anomalies is not yet established;
- the Mallala simulator geometry is approximate rather than survey-grade, although it is considered a good current reference;
- the simulator-local/world transform is not yet recorded in the repository; and
- current work is a Mallala R6 case calibration/validation exercise, not general motorcycle or track validation.
