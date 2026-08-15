# Phase 8: direct planar racing-line optimisation

Phase 8 is an alternative to, not a replacement for, the historical Phase 5
optimiser. Phase 5 optimises a latent periodic field and constructs its path
through Phase 4 offset geometry. Phase 8 instead makes every optimisation
variable a physical lateral offset in metres at a planar interpolation guide.
Those guides define the final Cartesian motorcycle trajectory directly.

## Geometry-aware path-model resolution

Uniform sparse guides cannot represent both a tight, high-angle corner and a
straight hundreds of metres long efficiently. Controls are generated within
every analytic track primitive. A straight of length `L` uses
`max(1, ceil(L / max_spacing_m))` subdivisions. A circular arc also takes the
maximum with `ceil(abs(turn_angle_rad) / max_arc_heading_change_rad)`. The start
of each subdivision is a station; every primitive boundary is retained, while
the duplicate closed endpoint is omitted. There are no hidden guides.

At station `s_i`, the guide is `C(s_i) + offset_i N(s_i)`. Positive offsets are
left of travel. Its local bounds are `-(width_right_i - margin)` and
`width_left_i - margin`; widths and calculations use SI units.

## Smooth Cartesian geometry and validity

Guides are interpolated by a non-uniform, C2-periodic Cartesian cubic spline
parameterised by analytic centreline station. The cyclic knot system uses each
actual interval and is solved for x and y with NumPy. First and second
derivatives, curvature and Gauss--Legendre geometric arc length derive from the
same spline. SciPy is not required.

Dense validation at common analytic centreline stations rejects rather than
clips between-guide corridor overshoot. It also requires a finite, strictly
positive dot product between spline derivative and track tangent and rejects a
zero or near-zero tangent. This prevents local reversal and loops without a
curvature-rate constraint. Phase 6 remains disabled by default and motorcycle
physics is unchanged.

All-zero controls put every guide on the analytic centreline, but the C2 spline
between them is not the track's piecewise-straight/circular-arc geometry. It is
the **zero-control planar baseline**, never the exact centreline. Diagnostics
evaluate the analytic fixed centreline separately.

## Two distinct sensitivity studies

Changing the control policy changes path-model order and refits different
geometry. Sampling a saved spline at 1.0, 0.5, or 0.25 m changes only the
fixed-path solver grid: guides, controls, spline, and integrated length remain
identical. Diagnostics separate control-policy and fixed-spline output studies.

The deterministic bounded coordinate search starts at zero, tries `+` then
`-`, uses local asymmetric physical bounds, and follows Phase 5 step reduction.
It is local, so no global optimum is claimed. A finer model is not automatically
more physically accurate. Material policy dependence is reported as
unconverged path-model sensitivity rather than tuned away. The model adds no
clothoids, centreline smoothing, roll dynamics, banking, elevation, kerbs,
calibration changes, or curvature-rate limit.
