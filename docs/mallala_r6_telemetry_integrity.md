# Mallala R6 telemetry integrity assessment

## Status

**Classification:** SOURCE-DERIVED / INITIAL PHASE 10 ASSESSMENT  
**Case:** Mallala R6, supplied AiM-derived workbook  
**Purpose:** establish whether the available session is suitable for initial 2D validation and roll-response assessment before parameter calibration.

The raw workbook is not committed. Its identity and SHA-256 hash are recorded in `cases/mallala_r6/raw_data_manifest.yaml`.

## Session structure

The `Updated` sheet contains 9,580 data samples spanning 0.00 to 478.95 s.

Timing quality is unusually clean for the first validation pass:

- every recorded sample interval is 0.05 s;
- nominal sample rate is therefore 20 Hz;
- there are no non-positive time increments;
- there are no gaps above 0.075 s; and
- the key channels inspected have no blank/non-finite samples.

The workbook marks six lap starts. Laps 1-5 are complete startline-to-startline laps; lap 6 is incomplete.

| Lap | Start time (s) | Lap time (s) | Samples | GPS integrated distance (m) |
| --- | ---: | ---: | ---: | ---: |
| 1 | 83.45 | 73.60 | 1472 | 2500.1222 |
| 2 | 157.05 | 74.20 | 1484 | 2497.7690 |
| 3 | 231.25 | 73.55 | 1471 | 2500.4541 |
| 4 | 304.80 | 72.65 | 1453 | 2501.2481 |
| 5 | 377.45 | 72.10 | 1442 | 2503.5127 |

Lap 5 is the fastest complete lap and is the existing selected `Lap5` worksheet. Lap 4 is the next-fastest complete lap. A reasonable **provisional** split is therefore Lap 5 for first calibration/development comparisons and Lap 4 as the first hold-out check, while retaining Laps 1-3 as additional out-of-fit comparisons. This split can be changed if later channel-quality or riding-consistency analysis gives a better reason.

## Initial channel checks

The inspected channels are populated throughout the session, including GPS position/speed/accelerations, GPS heading/gyro, RollRate/PitchRate/YawRate, ECU RPM, ECU GEAR, ECU throttle, hand throttle, and distance-from-start.

### ECU gear

The raw `ECU GEAR` channel is **not purely integer-valued**. It contains many fractional values during transitions (for example values between adjacent gears). The initial importer incorrectly required an integer gear channel; the Phase 10 branch has been corrected to preserve the raw floating-point signal and defer stable-gear classification to telemetry cleaning.

This is a useful example of why the raw workbook must be exercised directly rather than relying only on synthetic importer tests.

### Hand throttle

`ECU TPS HAND` slightly exceeds 100% in a substantial number of samples, with a recorded maximum of about 106.0%. The importer therefore preserves this channel as a raw scaled fraction and does not clip it to [0, 1]. Any normalization should be an explicit later cleaning/calibration decision.

### Roll-rate zero/bias indication

At very low GPS speed (<1 km/h), the raw RollRate channel has approximately:

- mean: +0.10 deg/s;
- median: +0.15 deg/s;
- standard deviation: about 1.02 deg/s; and
- approximate 5th-95th percentile range: -1.51 to +2.11 deg/s.

This suggests no large stationary roll-rate offset in the available data, but it does **not** establish logger orientation, dynamic axis alignment, or a final de-biasing method.

For Lap 5, RollRate spans approximately -94.5 to +75.8 deg/s, with the 99th percentile of absolute roll rate around the mid-50 deg/s range. Peaks should not be used directly as a model parameter without filtering/alignment review.

## Preliminary independent roll/lean-rate cross-check

As an intentionally simple diagnostic, planar lean was estimated from the GPS lateral acceleration using

`lean_est = atan(GPS_LatAcc_g)`

and differentiated with respect to time. This ignores banking, vertical dynamics, GPS acceleration filtering details, and logger attitude, so it is **not** yet a calibration-quality lean estimate.

On Lap 5, the raw derived lean-rate signal has the same preferred sign as RollRate. After progressively smoothing the lean estimate, the correlation with RollRate rises from roughly 0.45 unsmoothed to about 0.59 with a roughly 0.75-1.05 s moving-average window. Reversing the sign produces substantially worse RMS agreement.

Interpretation:

- the RollRate sign is plausibly consistent with the GPS lateral-acceleration convention for this lap;
- the moderate rather than high correlation is expected to require investigation of filtering, timing, axis orientation and the limitations of `atan(a_lat/g)`; and
- this is sufficient evidence to continue the roll-channel assessment, but not sufficient to set `max_roll_rate` yet.

The raw RollRate also correlates moderately with GPS Gyro in this session, which is another reason to inspect channel definitions/orientation rather than assuming a body-axis interpretation from channel names alone.

## Geometry/distance observation

The complete laps have GPS integrated distances of about 2498-2504 m. The approximate simulator Mallala centreline is about 2557 m, while the retained 52-control Phase 8 racing line is about 2516 m. These values are close enough to make rigid 2D registration and map matching worthwhile, but the differences are material enough that **no scale correction should be silently applied**.

The next registration step should fit only rotation/translation (and explicitly test orientation), then report residual position error versus chainage. If the residuals show a systematic scale or shape mismatch, that should be recorded as approximate-track-geometry error rather than absorbed into the telemetry.

## Phase 10 decisions from this assessment

1. The session is suitable for initial 2D validation: sampling is continuous and five complete laps are available.
2. Lap 5 is the first calibration/development candidate; Lap 4 is the first provisional hold-out candidate.
3. No motorcycle performance parameters should be tuned yet.
4. Preserve the 52-control Phase 8 line as the representative ideal-response baseline; use the 96-control line as an optimisation-assurance sensitivity case.
5. Proceed next to rigid 2D telemetry-to-track registration and map matching.
6. In parallel with registration, continue roll-channel sign/bias/filter/time-alignment work before making a finite-roll parameter active in the speed solver.

## Open questions / cautions

- logger physical orientation and body-axis definition remain undocumented;
- GPS LatAcc and GPS Gyro filtering/derivation need to be understood well enough for quantitative roll comparison;
- the approximate simulator track omits its original EPSG:7854 fitting transform;
- fractional ECU gear values require an explicit stable-gear classification rule for gear/RPM validation;
- throttle scaling above 100% must not be silently clipped; and
- all current conclusions are specific to this Mallala R6 session.
