# V1.1 Candidate Package Validation Record

## Baseline

- Immutable source release: `C172S_Flight_Dynamics_Autopilot_Laboratory_v1.0.0`
- Baseline ZIP SHA-256: `e93f5cd4548a3bb21d8957ef4815b4af4f1130196b1e223d9ed4672949468d0e`
- Accepted baseline laboratory audit: 52/52 PASS
- Accepted frozen regression chain: 28/28 PASS

The V1.0 directory and ZIP were not modified. V1.1 was assembled as a separate
candidate directory.

## Intended V1.1 changes

- two previously qualified reference-path performance changes;
- public MATLAB, Simulink, and compare execution selection;
- common 58-signal packing and equivalence reporting;
- backend-comparison visualization;
- one supplemental Simulink-backed public API audit;
- R4 laboratory metadata, menu, installation check, documentation, and a
  scoped clean-session input bridge for the released model InitFcn.

No gain, coefficient, state definition, actuator limit, mode-manager policy,
trim datum, request column, integration step, or envelope boundary is intended
to change.

## Packaging-environment checks

- V1.0 baseline ZIP hash rechecked: PASS
- Candidate kept separate from V1.0: PASS
- All 14 Simulink model containers remain readable: PASS
- Required V1.1 source files present: PASS
- Stale R2 artifact references in V1.1 MATLAB/public documentation: 0
- Forbidden caches/build products: 0
- Change-scope comparison against V1.0: 15 expected modifications, 6 expected additions, 0 removals; PASS
- Frozen controller, plant, model, evidence, and robustness files outside the declared change set: byte-identical; PASS
- Candidate SHA-256 manifest: generated for final candidate assembly

## Target-environment qualification

MATLAB/Simulink is not installed in the packaging environment. Runtime
promotion evidence therefore remains intentionally pending. Run every command
in `V1_1_QUALIFICATION.md`; do not rename the candidate as a release until all
required cases pass.
