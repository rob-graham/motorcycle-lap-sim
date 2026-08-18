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

For the current Mallala integration command, generic regions are consolidated against the current reference-track corner geometry. The nine nominal windows are defined from the analytic primitive groups for T1-T9, with T3, T6 and T7 represented by their contiguous compound arc groups. Non-overlapping ownership intervals extend those nominal windows to the midpoints of adjacent straights (and to the lap boundary for T1/T9). Raw regions are assigned by greatest ownership overlap only when their demanded-lean direction matches the nominal primitive direction. Multiple direction-consistent fragments may consolidate into one compound corner. Equal ownership ambiguity and any nominal corner without a direction-valid overlapping region fail closed; rejected/unassigned raw regions remain explicit in `phase12a_corner_regions_review.csv`.

This geometry-overlap step is intentionally case-specific. It avoids introducing a generic curvature cutoff that could incorrectly reject a real fast/shallow corner such as Mallala T4, and it does not change the generic lean-hysteresis detector for other circuits. The Mallala command still fails closed unless exactly nine mapped corner regions are supplied to event extraction.

## Event rules

For each accepted nominal corner the implementation records:

- `local_max_speed`: maximum speed between the previous corner exit and braking onset, or turn-in if no strong braking episode is detected;
- `braking_onset`: beginning of sustained deceleration before a strong-braking sample;
- `maximum_braking`: minimum longitudinal acceleration in the approach/corner search window;
- `brake_release`: first actual recovery above the release threshold after maximum braking; no release event is emitted merely because the search window ended;
- `turn_in`: first meaningful movement from the approach/outside condition toward the inside edge, bounded by the accepted raw region and nominal-corner ownership; a traceable medium-confidence bounded fallback is used for compound geometry;
- `geometric_apex`: minimum Euclidean racing-line clearance to the sampled physical inside track edge, with turn direction selecting left or right;
- `maximum_curvature`: maximum absolute racing-line curvature, retained as a separate engineering event;
- `speed_apex`: minimum speed within the corner region, retained separately when it differs from the geometric apex;
- `maximum_lean`: maximum absolute demanded lean, retained separately when it differs from the other apex events;
- `positive_drive_pickup`: a real below-to-above positive-drive threshold crossing after the speed apex that remains above threshold for the hold distance; an already-positive search start or transient spike emits no event;
- `corner_exit`: first meaningful departure from the inside-edge/apex condition toward the exit/outside, bounded by nominal ownership rather than residual low-angle lean;
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

## Phase 12A visual-validation suite

All five figures consume the same solved retained trajectory and extracted event set; they do not re-solve or alter the representative line.

- `phase12a_coaching_overview.png` is the clean whole-lap rider overview. It shows only BRK, TURN, geometric APEX, GAS (when present), and EXIT; REL and engineering events are deliberately absent.
- `phase12a_speed_map.png` continuously colours the racing line by authoritative solved speed in km/h, with only small BRK/APEX/GAS marks.
- `phase12a_T1_T3_detail.png`, `phase12a_T4_T6_detail.png`, and `phase12a_T7_T9_detail.png` provide equal-scale regional engineering review. They include REL where present and distinguish speed apex and maximum curvature from geometric apex. Their extents derive automatically from the selected event/corner spans with a fixed spatial margin.

The generic 6/4-degree lean hysteresis remains a raw candidate-region detector. It does not directly define rider-facing TURN or EXIT. Mallala's nominal-corner ownership and T3/T6/T7 compound grouping remain explicitly case-specific; the event landmarks inside those bounded searches are still derived from trajectory geometry. Missing GAS, braking, release, or engineering events are represented by absence rather than fabricated fallback locations.

Optimiser controls, optimiser spread/envelopes, corridor and centreline diagnostics, convergence metrics, and dense engineering annotations remain excluded from the clean overview. Gear/state/lean/roll-rate coloured maps, polished corner sheets, and richer chainage diagnostics are prospective Phase 12B work, not Phase 12A deliverables.

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
