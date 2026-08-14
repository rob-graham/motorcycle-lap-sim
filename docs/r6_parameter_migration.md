# Legacy R6 parameter migration notes

`legacy/bike_params_r6.csv` remains unchanged and is **not** a validated Phase 2
configuration. Known field mappings are:

| Legacy | New schema |
|---|---|
| `m` | `motorcycle.mass_kg` |
| `CdA` | `aerodynamics.cda_m2` |
| `Crr` | `rolling_resistance.crr` |
| `rw` | `motorcycle.wheel_radius_m` |
| `mu` | initial reference for both tyre coefficients (not validation) |
| `primary` | `powertrain.primary_ratio` |
| `gear1` … `gear6` | ordered `powertrain.gear_ratios` |
| `final_drive` | `powertrain.final_drive_ratio` |
| `eta_driveline` | `powertrain.driveline_efficiency` |
| `phi_max_deg` | `tyres.max_lean_angle_deg` |

Migration cannot be completed without inventing data. `T_peak = 63 Nm` is not
a torque-versus-RPM curve. Wheelbase, CG height, and longitudinal CG position
are absent. `shift_rpm` is not necessarily a physical rev limit. Old direct
wheelie/braking acceleration limits must not replace geometry-derived limits.
Finally, `kappa_dot` / steering-rate modelling belongs to a later
path/vehicle-dynamics stage.
