# Laboratory V1.1 Acceptance Evidence

## Release status

- Release: `v1.1.0`
- Laboratory artifact: `C172S-FLIGHT-CONTROL-LAB-V1-MATLAB-R4`
- Qualification environment: MATLAB/Simulink R2026a
- Promotion status: **PASS**

## Inherited frozen evidence

| Gate | Result |
|---|---:|
| Frozen controller regression | 18/18 PASS |
| Frozen nonlinear 6-DOF regression | 7/7 PASS |
| Nonlinear-autopilot interface/closed-loop/executable gates | 3/3 PASS |
| Combined frozen project evidence | 28/28 PASS |
| Public laboratory V1.0 R2 audit | 52/52 PASS |
| Manual V1.0 R2 acceptance | PASS |

The saved reports are retained unchanged and establish the frozen baseline against which V1.1 was qualified.

## V1.1 promotion gates

| Gate | Required result | Final result |
|---|---:|---:|
| `verifyC172sFlightDynamicsAutopilotLabInstallation` | PASS | **PASS — R4, 20/20 required files** |
| `auditC172sFlightControlLab` | 57/57 PASS | **57/57 PASS** |
| `auditC172sFlightControlLabBackends` | 8/8 PASS | **8/8 PASS — clean start** |
| `runAllNonlinearAutopilotChecks` | 28/28 PASS | **28/28 PASS** |
| Quick-demo `COMPARE` run | overall PASS and equivalence PASS | **PASS / PASS** |

## MATLAB / Simulink backend equivalence

The public backend audit used the non-trivial 20-second HEADING/ALTITUDE mission defined by the release qualification procedure. Results:

- Maximum absolute difference across 26 states + 32 diagnostics: **4.54747351e-13**
- Required tolerance: **3.0e-7**
- Discrete engagement/mode/event/protection signals: **exact match**
- Cold-start workspace independence: **PASS**
- Base-workspace residue check: **PASS**

The full public quick-demo `COMPARE` inspection also passed:

- Scenario objective: **PASS**
- Overall status: **PASS**
- Backend equivalence: **PASS**
- Maximum 58-signal difference: **5.00222086e-12**
- MATLAB wall time: **33.37 s**
- Simulink wall time: **2.13 s**
- Total comparison wall time: **35.49 s**

No qualification errors were recorded. The clean-start backend audit emitted only benign MATLAB `clear classes` warnings involving loaded `datetime` / `digraph` objects; they did not affect the PASS results.

See `V1_1_QUALIFICATION.md` for the commands and final qualification record.
