# Release Notes — v1.1.0 Candidate

## Release identity

- Product: C172S Flight Dynamics and Autopilot Laboratory
- Laboratory artifact: `C172S-FLIGHT-CONTROL-LAB-V1-MATLAB-R4`
- Candidate version: `v1.1.0-candidate`
- Frozen nonlinear-autopilot identity: `NONLINEAR-AUTOPILOT-V1-CLOSED-LOOP-MATLAB-R1`
- Controller dependency: `v1.0.0`
- Nonlinear 6-DOF dependency: `v0.1.0`
- Nonlinear-autopilot integration dependency: `v0.1.0`

## Added in V1.1

- Selectable `MATLAB`, `SIMULINK`, and `COMPARE` execution for normal
  laboratory scenarios.
- A public Simulink runner that reconstructs the same result structure used by
  the existing analysis, plot, and export layers.
- A shared 58-signal result matrix and public backend comparator.
- A supplemental non-trivial public-backend qualification gate.
- Menu access to the V1.1 MATLAB audit and backend audit.
- Backend identity and equivalence evidence in console and text exports.
- Separate MATLAB and Simulink wall-time reporting for comparison runs.
- Both full traces retained in MAT exports from `COMPARE` runs.
- Clean-session Simulink execution that scopes the model input to one run and
  restores the user's base workspace afterward.

## Performance maintenance

- Reuses the diagnostic derivative as RK4 `k1`, removing one redundant full
  nonlinear right-hand-side evaluation per propagated sample. The numerical
  algorithm, state ordering, sample rate, control laws, and outputs are
  unchanged.
- Replaces the quadratic settling-time tail scan with an equivalent linear
  search for the last tolerance-band violation.

The candidate must reproduce the prior 52/52 and 28/28 results and pass the new
public-backend gate, including its cold-start workspace check, before release
promotion.

## Preserved boundaries

No controller gains, plant coefficients, actuator limits, supervisor policy,
state definitions, trim values, request columns, sample rates, or validated
envelope limits were changed. V1.1 remains a near-trim research simulation with
ideal sensors and one-point throttle. It is not approved for aircraft
installation, certification, flight planning, or pilot training.
