# Fixed-path periodic speed solver (Phase 3)

Phase 3 finds a physically feasible minimum-time speed profile on a **supplied fixed path**. The demonstration adapter uses the track centreline; no racing line is created or optimised. Path coordinate `q` is arc length along the actual path. Closed samples omit the duplicate endpoint, and every calculation includes the final wrap segment.

The local ceiling is the lesser of the lateral limit `sqrt(min(mu_y g, g tan(lean_max))/abs(curvature))` (infinite at zero curvature) and the greatest road speed produced by the rev limit in any gear. Ideal gear selection tests every gear's RPM and interpolated torque and deterministically selects the lowest-numbered maximum-force gear. It assumes instantaneous shifts, no hysteresis, no shift time, and no clutch slip.

Drag and rolling resistance oppose propulsion and assist braking. Propulsion is rear-only; service braking uses both tyres. Longitudinal load transfer is quasi-static. Phase 3 apportions lateral force between axles in proportion to normal load, giving equal lateral utilisation for equal tyre coefficients. Negative normal load is infeasible, never clamped. Remaining longitudinal force comes from the friction ellipse, while geometric wheelie and stoppie limits are also enforced.

Starting from lateral/rev ceilings, cyclic forward acceleration and backward braking passes apply constant capability evaluated at the segment departure (braking at the following sample). Passes continue to an explicit speed-change tolerance, including the wrap constraint. This deterministic first-order scheme has resolution-dependent discretisation error. Lap time uses `dt = 2 dq/(v_i+v_j)`, including wrap.

Deferred: racing-line optimisation, suspension, gradient, downforce/pitching moment, shift dynamics, clutch slip, ABS/brake balance, and R6 parameter migration.
