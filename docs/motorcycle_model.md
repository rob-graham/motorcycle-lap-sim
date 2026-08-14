# Motorcycle model (Phase 2)

Phase 2 is an independently testable, flat-road physics layer. All inputs and
outputs use SI units except the YAML lean-angle input (degrees) and engine speed
(RPM). Configuration objects and formula results are immutable where practical.

## Coordinates, forces, and assumptions

The rear contact patch is longitudinal coordinate `x=0`, the front contact
patch is `x=L`, and the centre of gravity is at `x=b`, height `h`. Positive
longitudinal acceleration is forward. Resistance functions return **positive
magnitudes**; a later solver will assign their direction. Lateral functions
also return magnitudes.

The model assumes a rigid motorcycle on a flat road, constant coefficients,
no road gradient, no aerodynamic pitching moment, no suspension motion, and no
clutch slip. Negative computed axle loads are retained to expose tip-over.

## Formulas

Aerodynamic drag is `0.5 rho CdA v^2`; rolling resistance is `Crr m g`.
Longitudinal load transfer is

`N_front = (m g b - m a_x h) / L`, `N_rear = m g - N_front`.

Thus the positive wheelie acceleration is `g b / h`; setting `a_x` to this
makes the front load zero. The positive stoppie *deceleration magnitude* is
`g (L-b) / h`; setting `a_x` to its negative makes rear load zero.

The tyre lateral acceleration limit is `mu_lateral g`, the lean limit is
`g tan(phi_max)`, and the effective cap is the smaller value. These are not
converted to corner speed in this phase.

For selected one-based gear `i`, total reduction is
`primary * gear[i] * final`. Wheel speed is `v/r_w`, engine RPM is
`omega_wheel * ratio * 60/(2 pi)`, and unconstrained rear drive force is
`engine_torque * ratio * efficiency / r_w`. Torque is deterministically
linearly interpolated. It is zero below idle and above the rev limit. The table
must cover both limits, so no extrapolation occurs within the operating range.

## Combined tyre force

The generic ellipse utilisation is
`(Fx/(mu_x Fz))^2 + (Fy/(mu_y Fz))^2`; values at most one are feasible.
Remaining longitudinal magnitude is
`mu_x Fz sqrt(1-(Fy/(mu_y Fz))^2)`. It is deterministically zero for zero
normal load and at or beyond lateral capacity. With a zero coefficient, a zero
force contributes zero utilisation and a nonzero force contributes infinity.
Negative load or friction coefficients are invalid.

## Deliberately deferred

Phase 2 has no path coupling, corner-speed calculation, front/rear lateral
apportionment, gradient, automatic gear or shift selection, speed propagation,
lap-time integration, racing-line representation, or racing-line optimisation.
Those belong to the fixed-path and optimisation phases.
