# Provisional 2017+ Yamaha YZF-R6 reference calibration

## Scope and interpretation

`examples/motorcycles/r6_2017plus_reference.yaml` is a reproducible reference
motorcycle for exercising the Phase 3 fixed-path solver. It is **not** an exact
factory model-year reconstruction, a measured specimen, or an experimentally
validated lap-time model. It uses SI units and the existing motorcycle YAML
schema, including an inline torque curve.

The labels in this document distinguish factory information, estimates, legacy
references, and unknowns. In particular, a provisional value must not be read
as a measured Yamaha specification. Better measured inputs can replace
individual values later without changing the solver architecture.

## Parameter register

| Input | Value | Provenance | Assumption or rationale |
|---|---:|---|---|
| `gravity_mps2` | 9.81 m/s² | PROVISIONAL / ASSUMED | Standard reference environment, not a track measurement. |
| `air_density_kgpm3` | 1.225 kg/m³ | PROVISIONAL / ASSUMED | Standard reference environment, not weather data. |
| `mass_kg` | 265.0 kg | LEGACY-REFERENCE / PROVISIONAL | Approximate combined motorcycle-and-rider system mass; **not** Yamaha's published wet-bike mass. |
| `wheelbase_m` | 1.375 m | FACTORY-SOURCED | Reference wheelbase for the 2017+ R6 family; Yamaha's official R6 RACE specifications list a 1,375 mm wheelbase.[^yamaha-wheelbase] |
| `cg_height_m` | 0.625 m | DERIVED-FROM-LEGACY / PROVISIONAL | Chosen to approximate legacy longitudinal limits; not a measured R6 coordinate. |
| `cg_from_rear_m` | 0.625 m | DERIVED-FROM-LEGACY / PROVISIONAL | Chosen to approximate legacy longitudinal limits; not a measured R6 coordinate. |
| `wheel_radius_m` | 0.31 m | LEGACY-REFERENCE / PROVISIONAL | Effective rolling radius remains unvalidated. |
| `cda_m2` | 0.40 m² | LEGACY-REFERENCE / PROVISIONAL | Rider/bike aerodynamic area remains unvalidated. |
| `crr` | 0.015 | LEGACY-REFERENCE / PROVISIONAL | Rolling-resistance coefficient remains unvalidated. |
| `mu_longitudinal` | 1.2 | LEGACY-REFERENCE / PROVISIONAL | Reference coefficient, not a validated modern racing-tyre measurement. |
| `mu_lateral` | 1.2 | LEGACY-REFERENCE / PROVISIONAL | Reference coefficient, not a validated modern racing-tyre measurement. |
| `max_lean_angle_deg` | 55° | LEGACY-REFERENCE / PROVISIONAL | Reference limit rather than a measured setup/rider limit. |
| `primary_ratio` | 2.073170732 | LEGACY-REFERENCE | No stronger source is currently recorded in this repository. |
| gear ratios 1–6 | 2.583, 2.000, 1.667, 1.444, 1.286, 1.150 | LEGACY-REFERENCE | No stronger source is currently recorded in this repository. |
| `final_drive_ratio` | 2.9375 | LEGACY-REFERENCE | No stronger source is currently recorded in this repository. |
| `driveline_efficiency` | 1.0 | PROVISIONAL / ASSUMED (modeling choice) | Curve is rear-wheel/chassis-dyno output; 1.0 prevents double-counting losses already represented by the curve. |
| `idle_rpm` | 1500 rpm | PROVISIONAL / ASSUMED | Operating boundary for the provisional curve. |
| `rev_limit_rpm` | 16000 rpm | PROVISIONAL / ASSUMED | The legacy `shift_rpm` field is not evidence of a physical rev limit. |
| inline torque curve | 28–59 Nm, 1500–16000 rpm | EMPIRICAL-ESTIMATE / PROVISIONAL | Smooth, stock-like **rear-wheel** estimate; individual points were not precisely digitised. |

No parameter in this register currently uses the `UNKNOWN` label because each
configured value has an identified source class or explicit assumption. Actual
measured CG coordinates and condition-specific tyre, aero, rolling-resistance,
wheel-radius, and combined-mass values remain unknown.

## Torque-curve calibration

The supplied dyno image contained higher modified curves likely reflecting an
aftermarket exhaust and possibly fueling changes. Its lower curves appeared
broadly consistent with another stock-looking R6 graph. The selected curve is
therefore a deliberately smooth **EMPIRICAL-ESTIMATE**, not a point-by-point
digitisation and not crankshaft torque.

It peaks at about 59 Nm and represents stock-like high-RPM performance. Its
largest listed power is about 111 rear-wheel horsepower (83 kW), within the
intended approximate 110–115 rear-wheel-horsepower range. Because chassis-dyno
losses are already implicit, `driveline_efficiency: 1.0` avoids applying a
second drivetrain loss. The entire curve remains replaceable when traceable
measured data become available.

## CG derivation

For longitudinal acceleration magnitude, the Phase 2 geometric limits are

```text
a_wheelie = g * cg_from_rear / cg_height
a_stoppie = g * (wheelbase - cg_from_rear) / cg_height
```

With wheelbase 1.375 m and both provisional CG coordinates 0.625 m, these give
`1.000 g` wheelie acceleration and `1.200 g` stoppie deceleration. This
construction approximately reproduces old legacy limits; it does **not** turn
the coordinates into known Yamaha measurements.

## Deterministic CG sensitivity

Each case below changes only the named CG coordinate from the baseline. The
same test oval, reference configuration, solver defaults, and approximately
1 m sampling (389 samples over 388.496 m) are used. All five solutions
converged in three iterations.

| Case | CG height (m) | CG from rear (m) | Wheelie limit | Stoppie limit | Oval lap time (s) |
|---|---:|---:|---:|---:|---:|
| Baseline | 0.625 | 0.625 | 9.810 m/s² (1.000 g) | 11.772 m/s² (1.200 g) | 17.185570 |
| Lower CG | 0.550 | 0.625 | 11.148 m/s² (1.136 g) | 13.377 m/s² (1.364 g) | 17.097604 |
| Higher CG | 0.700 | 0.625 | 8.759 m/s² (0.893 g) | 10.511 m/s² (1.071 g) | 17.362094 |
| Rearward CG | 0.625 | 0.575 | 9.025 m/s² (0.920 g) | 12.557 m/s² (1.280 g) | 17.211476 |
| Forward CG | 0.625 | 0.675 | 10.595 m/s² (1.080 g) | 10.987 m/s² (1.120 g) | 17.198261 |

These cases quantify sensitivity; they do not establish which CG is correct.
The resulting lap-time predictions are provisional because CG and several
other influential inputs are provisional.

Reproduce the table from the repository root with:

```bash
python scripts/r6_cg_sensitivity.py
```

## Baseline diagnostic record

The motorcycle diagnostic reports 45.45% static front load, 54.55% static
rear load, 1.000 g wheelie and 1.200 g stoppie limits, an 11.772 m/s² effective
lateral cap, and unconstrained drive forces of 2193.5 N at 10 m/s in first and
2063.2 N at 20 m/s in second.

At approximately 1 m sampling, the fixed-path diagnostic reports a 17.185570 s
lap, 18.767/36.687/24.039 m/s sampled minimum/maximum/arithmetic-mean speeds,
11.772 m/s² maximum lateral acceleration, 9.810 m/s² maximum forward
acceleration, 11.772 m/s² maximum braking deceleration, gears 1–2, RPM
9094–15630, three iterations, and convergence. The sampled arithmetic mean is
the whole-profile statistic printed by the diagnostic; it is not inferred from
lap distance divided by time. None of these results constitutes experimental
validation.

The CSV diagnostic invocation can reproduce the full sampled result:

```bash
python -m motorcycle_lap_sim.speed_solver.diagnostics \
    examples/tracks/test_oval.yaml \
    examples/motorcycles/r6_2017plus_reference.yaml \
    --spacing 1.0 --csv r6_2017plus_test_oval.csv
```

Racing-line representation and optimisation, measured-data identification,
and exact model-year reconstruction are intentionally deferred.

[^yamaha-wheelbase]: Yamaha Motor Europe, [R6 RACE specifications](https://www.yamaha-motor.eu/gb/en/motorcycles/supersport/pdp/r6-race-2025/),
    "Chassis" technical specification (accessed 14 August 2026). The current
    factory specification corroborates the unchanged 2017+ chassis wheelbase;
    it is not used as provenance for any other parameter in this register.

## Path-curvature transient proxy — LEGACY-REFERENCE / PROVISIONAL

`legacy/bike_params_r6.csv` contained `kappa_dot_max = 0.8` and the old file also enabled its steer-rate cap. Its precise physical provenance and interpretation are unknown. Phase 6 therefore uses 0.8 1/(m*s) only as a provisional calibration for the path-curvature transient proxy. It is **not** a measured R6 steering-rate value and does not validate a handlebar-dynamics model. The legacy CSV remains unchanged.
