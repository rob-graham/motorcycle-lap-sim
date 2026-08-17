# Coaching event definitions

**Status date:** 2026-08-17  
**Phase:** 12A  
**Status:** INTERNAL PROTOTYPE pending Mallala visual review

## Scope

Phase 12A extracts deterministic rider-facing landmarks from the retained Phase 11 Mallala representative line. It is deliberately narrower than the later run-off interface.

The input is a solved representative trajectory containing line geometry, speed, longitudinal acceleration, curvature, demanded lean, roll-rate demand, gear and RPM. The current Mallala command re-evaluates the retained `reduced_reoptimised_51` controls using the same 0.250 m corridor margin, 0.8 rad/s finite-roll sensitivity scenario, 0.125 m common grid and Python backend used for Phase 11 representative selection.

The 0.8 rad/s value remains an uncalibrated sensitivity parameter, not a measured R6/rider constant. The coaching marks are model-derived reference points, not universally safe physical track markers.

## Corner segmentation

Generic candidate corner regions are derived from demanded lean angle with hysteresis:

- corner-on threshold: absolute lean >= 6 degrees;
- corner-off threshold: absolute lean >= 4 degrees;
- minimum retained corner length: 18 m; and
- same-direction regions separated by no more than 35 m are merged.

The first retained-line Mallala execution produced 12 generic lean regions rather than nine. Inspection showed that the extra regions were not unmerged compound corners: they were sustained low-curvature setup/straight-line lean regions at the lap start, before T5 and across the lap-end/start-finish transition. The nine intended T1-T9 regions were already present among the 12 candidates.

For the current Mallala integration command, generic regions are therefore post-filtered against the current reference-track corner geometry. The nine reference windows are defined from the analytic primitive groups for T1-T9, with T3, T6 and T7 represented by their contiguous compound arc groups. For each reference window, the command selects the unique detected lean region with the largest positive chainage overlap. It fails closed if any T1-T9 window has no detected region or if one detected region would map to more than one reference corner.

This geometry-overlap step is intentionally case-specific. It avoids introducing a generic curvature cutoff that could incorrectly reject a real fast/shallow corner such as Mallala T4, and it does not change the generic lean-hysteresis detector for other circuits. The Mallala command still fails closed unless exactly nine mapped corner regions are supplied to event extraction.

## Event rules

For each accepted corner the initial implementation records:

- `local_max_speed`: maximum speed between the previous corner exit and braking onset, or turn-in if no strong braking episode is detected;
- `braking_onset`: beginning of sustained deceleration before a strong-braking sample;
- `maximum_braking`: minimum longitudinal acceleration in the approach/corner search window;
- `brake_release`: first actual recovery above the release threshold after maximum braking; no release event is emitted merely because the search window ended;
- `turn_in`: start of the sustained lean region after hysteresis;
- `geometric_apex`: maximum absolute racing-line curvature within the corner region;
- `speed_apex`: minimum speed within the corner region, retained separately when it differs from the geometric apex;
- `maximum_lean`: maximum absolute demanded lean, retained separately when it differs from the other apex events;
- `positive_drive_pickup`: first sustained positive longitudinal acceleration after the speed apex;
- `corner_exit`: end of the sustained lean region after hysteresis;
- `roll_transition`: minimum absolute demanded lean between consecutive opposite-direction corner regions; and
- `gear_shift`: solver gear-number change between adjacent samples.

Initial longitudinal thresholds are explicit, replaceable prototype values:

- strong braking <= -1.5 m/s^2;
- braking onset <= -0.35 m/s^2;
- brake release >= -0.20 m/s^2;
- positive drive >= +0.35 m/s^2, sustained over at least 4 m.

These thresholds are event-detection parameters. They are not motorcycle performance limits or safety criteria.

## Event record

Each event records:

- corner identifier;
- event type;
- sample index;
- reference-track chainage;
- racing-line distance;
- local x/y coordinates;
- speed;
- longitudinal acceleration;
- racing-line curvature;
- demanded lean angle;
- Level-1 roll-rate demand when available;
- gear and RPM when available;
- the source rule; and
- a confidence/quality classification.

GIS coordinates are not added in Phase 12A because the current Mallala geometry is still local/approximate. That field belongs in a later georeferenced result/export layer.

## Rider-facing map

The Phase 12A racing-line image is intentionally presentation-focused. It contains:

- physical track edges;
- the retained representative racing line;
- start/finish; and
- rider-facing coaching marks for braking onset, brake release, turn-in, geometric apex, positive-drive pickup and corner exit.

Labels show the corner, event abbreviation, speed, and where useful gear or demanded lean.

The coaching image must not show optimiser control points, optimiser/control-basis spread, margin-corridor lines, centreline diagnostics, convergence information, or other development-only overlays.

Additional extracted events such as maximum braking, speed apex, maximum lean, roll transition and gear shifts remain in the machine-readable event table but are not drawn by default to avoid obscuring the rider-facing map.

## Review gate before run-off interface work

The generated Mallala event CSV and coaching image must be visually reviewed together. Review should check at least:

- one sensible T1-T9 corner sequence;
- braking marks on the expected approaches rather than after the apex;
- turn-in before the geometric apex;
- brake release in a plausible part of the entry phase;
- positive-drive pickup after the speed/apex region;
- exits on the departure side of each corner;
- separate geometric and speed apexes where the solved trajectory supports that distinction; and
- no obvious event duplication caused by compound analytic track primitives or straight/setup lean artefacts.

If the visual review exposes bad segmentation or event rules, Phase 12A should be corrected and rerun before downstream interfaces are frozen.

The simulator-to-run-off export contract is intentionally not defined in this document. It will be designed only after the retained event locations have passed the Mallala visual review.