# Simulator-to-run-off input contract

**Status:** Phase 12B internal prototype  
**Interface version:** `0.1.0`

## Purpose

This contract defines the simulator-side hand-off to a separate run-off calculation package. It transfers solved trajectory state and traceable candidate departure states. Run-off calculation assumptions remain downstream and are not embedded in the lap simulator.

The current ownership boundary is:

- `motorcycle-lap-sim` owns solved trajectory state, simulator track-edge geometry, reviewed event provenance, and candidate departure-state extraction;
- the separate run-off package will own off-track surface parameters, terrain propagation, stopping criteria, protection geometry, uncertainty treatment, and standards-comparison profiles.

Candidate departure seeds are engineering starting points. They do not assert that a departure will occur there or that a selected point is the controlling case.

## Coordinates and chainage

Version `0.1.0` uses local Cartesian metres. It does not invent a CRS, world transform, elevation, grade or banking when those data are not implemented.

Two distance coordinates are retained:

- `track_s_m`: reference-track parameter used to construct and sample the racing line;
- `path_q_m`: arc length along the solved racing line.

`path_heading_rad` is derived from the solved closed racing-line coordinates using a periodic central chord and `atan2(dy, dx)`.

## Required trajectory state

The initial hand-off requires sample index, both chainages, racing-line x/y, physical left/right track-edge x/y, speed, longitudinal acceleration, signed lateral acceleration, path curvature, demanded lean angle, the Level-1 model roll-rate term, and derived path heading.

Gear, RPM and selected model-limit flags may be included when present. Their presence is diagnostic and does not make them run-off criteria.

Required arrays are equal-length deterministic copies. Chainages start at zero and increase strictly, sample indices are contiguous from zero, and exported arrays are read-only.

## Scenario identity

Each package must include at least `scenario_id`, `simulator_commit`, and `track_id`. Additional metadata may record motorcycle configuration identity, retained-line identity, handling scenario, input hashes and geometry version. Warnings are transferred separately.

## Candidate departure seeds

Version `0.1.0` maps only reviewed event semantics with a direct downstream engineering use:

| Reviewed event | Candidate seed type |
| --- | --- |
| `local_max_speed` | `missed_braking_candidate` |
| `braking_onset` | `upright_overrun_candidate` |
| `turn_in` | `entry_lowside_turn_in_candidate` |
| `geometric_apex` | `entry_lowside_apex_candidate` |
| `positive_drive_pickup` | `exit_highside_candidate` |

Each seed records the source event type, source rule, confidence and exact trajectory sample state. Event values are checked against the supplied trajectory before a seed is accepted, so stale event and trajectory data fail closed.

The mapping deliberately excludes optimiser spread, optimiser control points and capability-limit classifications as automatic departure criteria.

## Explicit non-goals of version 0.1.0

The first interface does not calculate off-track travel, assign surface coefficients, set a terminal-speed criterion, implement uncertainty distributions, reproduce the published MA comparison method, or generate the standards-comparison tangential envelope. Those are separate later increments with their own versioned assumptions and tests.

## Next increments

After review of this contract, the next steps are to export the retained Mallala Phase 12A case through the interface, review the generated departure candidates, then begin a separate deterministic run-off calculation core with named profiles and analytically checkable tests. Surface/terrain propagation, protection-geometry intersection, standards comparison, georeferencing and 3D terrain fields should then be added as separately reviewable increments.

## Interpretation boundary

This interface is `INTERNAL-PROTOTYPE` and `NOT-EXTERNALLY-ACCEPTED`. It is a traceable engineering data contract, not a claim of track compliance, homologation, certification, insurance acceptance, or rider instruction.
