# Path-curvature transient limit

Phase 6 optionally limits how quickly the curvature of the selected path changes as the motorcycle progresses. Signed path curvature, `kappa`, has units 1/m. The coordinate `q` is distance along the `SampledPath`, in metres, so `d(kappa)/dq` has units 1/m². Since `dq/dt = v`,

    d(kappa)/dt = v d(kappa)/dq

and a configured `max_path_curvature_rate_1pmps` gives the local ceiling

    v_kappa = max_path_curvature_rate_1pmps / abs(d(kappa)/dq).

An exactly zero gradient produces an infinite ceiling, not an arbitrary finite cap. The solver takes the minimum of this ceiling, the lateral ceiling, and the powertrain/rev ceiling before its unchanged cyclic acceleration/braking passes.

## Periodic numerical derivative

The derivative is a centred, three-point, second-order formula with separate distances to the previous and next samples. It therefore supports nonuniform `q` spacing. At index zero and the final omitted-endpoint sample, neighbours wrap using the total path length. A duplicate endpoint is neither expected nor accepted. The difference-form implementation returns exact zero for exactly constant sampled curvature and uses no smoothing or hidden zero tolerance.

Both the curvature gradient and the resulting actual curvature rate are sampled quantities. Sharp geometry and interpolation can make their peaks dependent on resolution; results should be checked by re-evaluating fixed controls at finer spacing rather than assuming convergence from one grid.

## Interpretation and assumptions

This constraint is a **path-curvature transient proxy**, introduced so rapid path-curvature changes are not free in a minimum-time calculation. It is not a validated handlebar or steering-rate model. It omits steering-head kinematics, trail/rake dynamics, countersteering force, rider dynamics, suspension, and other motorcycle dynamics.

### Track centreline, racing line, and motorcycle response

The **track centreline** may deliberately be piecewise straight and circular.
Its curvature may be discontinuous where those convenient specification
primitives join; such a discontinuity is not a track-modelling error and is not
smoothed by this feature.

The **racing line** represents the motorcycle trajectory. It may become
smoother than the underlying centreline. Future work will improve racing-line
geometry independently of the simple track primitives. In particular, smooth
planar x/y splines, guide-point interpolation, and alternative racing-line
curvature calculations are deferred, as are roll-rate, lean-angle-rate,
steering-head dynamics, clothoid primitives, and changes to primitive
definitions.

For **motorcycle response**, a disabled transient limit is the idealised
instantaneous-response interpretation. Enabling it selects an optional finite
path-curvature-response proxy. Neither choice is universally correct, and
neither changes the actual track geometry.

The R6 reference value 0.8 1/(m*s) is **LEGACY-REFERENCE / PROVISIONAL**. It is
not a measured R6 steering, roll, or lean-rate value and the sensitivity
diagnostic must not be used to claim otherwise. It is disabled by default.
When `handling` is absent, the ceiling is infinity and earlier-phase numerical
behaviour is preserved. The diagnostic can apply it temporarily with
`--curvature-rate-limit 0.8`; it does not edit the YAML configuration.

Simple motorcycle-response approximations should not be allowed to create
large unexplained changes in predicted lap time. Handling-model effects must
always be reported relative to the disabled/ideal-response baseline. Large
differences are sensitivity requiring investigation, not automatically a more
accurate result.
