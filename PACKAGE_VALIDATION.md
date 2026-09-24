# V1.1 Package Validation Record

## Baseline

- Immutable source release: `C172S_Flight_Dynamics_Autopilot_Laboratory_v1.0.0`
- Baseline ZIP SHA-256: `e93f5cd4548a3bb21d8957ef4815b4af4f1130196b1e223d9ed4672949468d0e`
- Accepted baseline laboratory audit: 52/52 PASS
- Accepted frozen regression chain: 28/28 PASS

The V1.0 directory and ZIP were not modified. V1.1 was assembled and qualified as a separate release line.

## V1.1 changes

- two previously qualified reference-path performance changes;
- public MATLAB, Simulink, and compare execution selection;
- common 58-signal packing and equivalence reporting;
- backend-comparison visualization;
- one supplemental Simulink-backed public API audit;
- R4 laboratory metadata, menu, installation check, documentation, and a scoped clean-session input bridge for the released model InitFcn.

No gain, coefficient, state definition, actuator limit, mode-manager policy, trim datum, request column, integration step, or envelope boundary changed.

## Packaging checks

- V1.0 baseline ZIP hash rechecked: PASS
- V1.1 kept separate from V1.0: PASS
- All 14 Simulink model containers readable during package assembly: PASS
- Required V1.1 source files present: PASS
- Stale R2 artifact references in V1.1 MATLAB/public documentation: 0
- Forbidden caches/build products: 0
- Change-scope comparison against V1.0: 15 expected modifications, 6 expected additions, 0 removals; PASS
- Frozen controller, plant, model, evidence, and robustness files outside the declared change set: byte-identical; PASS
- Final SHA-256 manifest: regenerated after release promotion

## Target-environment qualification

Final V1.1 promotion qualification was completed in MATLAB/Simulink R2026a:

| Gate | Final result |
|---|---:|
| Installation preflight | **PASS — R4, 20/20 required files** |
| Public laboratory audit | **57/57 PASS** |
| Public MATLAB/Simulink backend audit | **8/8 PASS — clean start** |
| Frozen nonlinear-autopilot regression | **28/28 PASS** |
| Quick-demo `COMPARE` | **overall PASS / equivalence PASS** |

Backend-audit maximum 58-signal difference: **4.54747351e-13** (limit **3.0e-7**).

Quick-demo comparison maximum difference: **5.00222086e-12**. Recorded wall times were **33.37 s MATLAB**, **2.13 s Simulink**, **35.49 s total**.

The qualification requirements were therefore satisfied and the laboratory was promoted to **`v1.1.0`**.
