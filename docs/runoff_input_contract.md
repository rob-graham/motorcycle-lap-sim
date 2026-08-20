# Simulator-to-run-off input contract

**Status:** Phase 12B internal prototype  
**Interface version:** `0.1.0`
**Bundle version:** `1.0.0`

**Optional georeference extension version:** `0.1.0`

## Purpose

This contract defines the simulator-side hand-off to a separate run-off calculation package. It transfers solved trajectory state and traceable candidate departure states. Run-off calculation assumptions remain downstream and are not embedded in the lap simulator.

The canonical machine-readable cross-repository contract snapshot is [`contracts/runoff_interface_0.1.0.json`](../contracts/runoff_interface_0.1.0.json).

The current ownership boundary is:

- `motorcycle-lap-sim` owns local track geometry, local racing-line trajectory, local track
  boundaries, any authoritative local-to-projected georeference and its provenance, event
  provenance, and candidate departure-state extraction;
- the separate run-off package will consume the extension, transform all local run-off/site
  geometry consistently, create GIS outputs, and own off-track surface parameters, terrain
  propagation, stopping criteria, protection geometry, uncertainty treatment, and
  standards-comparison profiles. It must not guess, fit, or reconstruct the Mallala transform.

Candidate departure seeds are engineering starting points. They do not assert that a departure will occur there or that a selected point is the controlling case.

## Coordinates and chainage

Version `0.1.0` uses local Cartesian metres. It does not invent a CRS, world transform, elevation, grade or banking when those data are not implemented.

Local coordinates remain authoritative for numerical simulation. An optional, separately
versioned georeference maps them into a projected horizontal CRS without changing trajectory
values, physics, scale, or the local coordinate convention. A bundle without the extension
remains a valid three-file version `1.0.0` bundle.

## Optional rigid georeference extension

The canonical snapshot is
[`contracts/georeference_0.1.0.json`](../contracts/georeference_0.1.0.json). Extension `0.1.0`
uses schema `motorcycle-lap-sim-georeference/1` and requires `schema`, `horizontal_crs`,
`origin_projected_x_m`, `origin_projected_y_m`, `rotation_rad_ccw`, `source`, and `status`.
`source_sha256` and `derivation` are optional provenance. Rotation accepts any finite value
exactly; it is not silently normalised. CRS strings are recorded but are not checked against an
online EPSG database.

For `theta = rotation_rad_ccw`, positive counter-clockwise rotation is applied without a GIS
library:

```text
projected_x = origin_projected_x_m + cos(theta)*local_x - sin(theta)*local_y
projected_y = origin_projected_y_m + sin(theta)*local_x + cos(theta)*local_y
```

When supplied, deterministic `georeference.json` is described under
`manifest.json.extensions.georeference`, including extension version, schema, filename, and
SHA-256. It is deliberately absent from the immutable interface `0.1.0` artifact list.

Two distance coordinates are retained:

- `track_s_m`: reference-track parameter used to construct and sample the racing line;
- `path_q_m`: arc length along the solved racing line.

The package also requires explicit `track_length_m` and `path_length_m`. Samples follow the closed-loop convention **duplicated endpoint omitted**, so the final stored chainage is less than the corresponding total length and the total length supplies the wrap segment across start/finish.

`heading_rad` is derived from the solved closed racing-line coordinates using a periodic three-point derivative with respect to `path_q_m`. The unequal-spacing derivative uses the wrapped path length at start/finish; it therefore does not rely on uniform trajectory sampling. It is exported at every trajectory sample so downstream analysis can select arbitrary physical states without duplicating heading calculations. The earlier `path_heading_rad` name remains as a compatibility alias.

## Required trajectory state

The initial hand-off requires sample index, both chainages, racing-line x/y, physical left/right track-edge x/y, speed, longitudinal acceleration, signed lateral acceleration, path curvature, demanded lean angle, the Level-1 model roll-rate term, and derived path heading.

Gear, RPM and selected model-limit flags may be included when present. Their presence is diagnostic and does not make them run-off criteria.

Required arrays are equal-length defensive copies. Chainages start at zero and increase strictly, sample indices are contiguous integer values from zero, and numeric/string array storage is backed by immutable bytes so NumPy writes cannot simply be re-enabled by the consumer.

## Portable directory bundle

Bundle version `1.0.0` serializes the in-memory interface without changing its `0.1.0` semantics. It contains deterministic `manifest.json`, `trajectory.csv`, and `departure_seeds.csv` files. The manifest records field order, counts, interface and bundle versions, coordinate/chainage/sampling conventions, scenario metadata, warnings, and SHA-256 hashes of both CSV files. CSV uses a fixed column order, UTF-8, LF line endings and round-trip-safe floating-point text. It contains no timestamp or other runtime-varying field.

The interface performs structural consistency checks. It does **not** claim to re-prove complete track geometry correctness (for example, swapped boundaries or survey-grade spatial consistency); those matters remain tied to upstream geometry provenance and retained-case integration checks.

## Scenario and event-set identity

Each package must include at least:

- `scenario_id`;
- `simulator_commit`;
- `track_id`; and
- `event_set_id`.

Additional metadata may record motorcycle configuration identity, retained-line identity, handling scenario, input hashes and geometry version. Warnings are transferred separately.

`event_set_id` is the minimum provenance hook distinguishing the event set used for candidate extraction from arbitrary caller-created event objects. For the retained Mallala case, the later integration export should use an identifier/hash tied to the Phase 12A generated event artefact. This interface still validates individual event structure and state correspondence; it does not by itself certify that an arbitrary event set has undergone human review.

## Candidate departure seeds

Version `0.1.0` maps only supported event semantics with a direct downstream engineering use:

| Event | Candidate seed type |
| --- | --- |
| `local_max_speed` | `missed_braking_candidate` |
| `braking_onset` | `upright_overrun_candidate` |
| `turn_in` | `entry_lowside_turn_in_candidate` |
| `geometric_apex` | `entry_lowside_apex_candidate` |
| `corner_exit` | `entry_lowside_corner_exit_candidate` |
| `positive_drive_pickup` | `exit_highside_candidate` |

Each seed records the source event type, source rule, confidence and exact trajectory sample state. The event sample index must be a finite integer-valued scalar; fractional, boolean, string, non-finite and array-like indices fail closed.

Event copies of chainage, position, speed, longitudinal acceleration, curvature and lean are checked against the supplied trajectory. A finite event roll-rate copy is also checked against the trajectory roll-rate field. `NaN` roll rate is the one explicit missing-value convention: it means the event did not retain a roll-rate value, so the trajectory field remains authoritative. Physical seed values are always taken from the validated trajectory, not copied from the event object.

The mapping deliberately excludes optimiser spread, optimiser control points and capability-limit classifications as automatic departure criteria.

The corner-exit LOWSIDE candidate supplies the model-derived downstream endpoint that can be paired with the turn-in candidate for dense corner sampling. The simulator does not choose intermediate sampling spacing, generate dense candidates, or calculate run-off requirements. All departure candidates remain analysis inputs, not occurrence probabilities or safety criteria.

## Explicit non-goals of version 0.1.0

The first interface does not calculate off-track travel, assign surface coefficients, implement uncertainty distributions, reproduce the published MA comparison method, or generate the standards-comparison tangential envelope. Those are separate later increments with their own versioned assumptions and tests.

## Initial stopping-criterion decision for the run-off core

The first deterministic physical run-off calculations will calculate stopping distance to `0.0 m/s` (0 km/h). This is the baseline stopping criterion for initial rider-slide, motorcycle-slide and other deterministic propagation models unless a scenario explicitly states otherwise.

The provisional 24 km/h value considered in the earlier internal run-off working document is **not** the default stopping criterion. It was considered as a possible residual-speed limit for reaching an energy-absorbing protection system rather than as the normal definition of required run-off distance, and it has not been sufficiently established for production use.

Residual-speed-at-barrier criteria, including any future 24 km/h case, must therefore be implemented later as separately named and versioned barrier/protection scenarios with explicit provenance, justification and sensitivity analysis. They must not silently shorten the baseline run-off-to-rest calculation.

## Next increments

The active gate is the bounded retained-Mallala integration export using the retained trajectory and generated Phase 12A event set. The retained acceptance identity is fail-closed: its fixed controls SHA-256 and canonical deleted-control index, margin, roll-rate scenario, sampling settings, expected lap and reproduction tolerance cannot be overridden by a caller. It verifies total-length and wrap semantics, content-derived event-set provenance, candidate counts/types and representative fields. Phase 12B must not be treated as finally closed until the Owner successfully executes that command on the target machine.

The current Owner decision is to complete the first end-to-end track-layout/run-off workflow using
the existing 2D LOWSIDE RIDER model and GIS/mapping before adding further crash models. GIS output
itself remains downstream. EPSG:7854 positioning does not make Mallala's approximate analytical
track survey-grade, and GIS presentation is not certification, homologation, or external
acceptance.

## Interpretation boundary

This interface is `INTERNAL-PROTOTYPE` and `NOT-EXTERNALLY-ACCEPTED`. It is a traceable engineering data contract, not a claim of track compliance, homologation, certification, insurance acceptance, or rider instruction.
