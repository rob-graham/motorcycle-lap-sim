# Repository instructions

- This is a clean-sheet implementation.
- Do not import old lap-time simulator source code unless explicitly requested.
- Build and validate fixed-path simulation before racing-line optimisation.
- Use SI units internally.
- Physical formulas belong in clearly identified functions/modules.
- No mutable module-level global state.
- Plotting must be separate from numerical calculations.
- Every new physical or geometric feature requires tests.
- Preserve analytically understandable test cases.
- Never suppress numerical or optimisation warnings merely to make tests pass.
- Record assumptions explicitly.
- Prefer deterministic calculations.
- Maintain backward-compatible data formats only when deliberately specified.
