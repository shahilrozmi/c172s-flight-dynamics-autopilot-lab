# Laboratory V1.1 Candidate Acceptance Evidence

## Inherited frozen evidence

| Gate | Result |
|---|---:|
| Frozen controller regression | 18/18 PASS |
| Frozen nonlinear 6-DOF regression | 7/7 PASS |
| Nonlinear-autopilot interface/closed-loop/executable gates | 3/3 PASS |
| Combined frozen project evidence | 28/28 PASS |
| Public laboratory V1.0 R2 audit | 52/52 PASS |
| Manual V1.0 R2 acceptance | PASS |

The saved reports are retained unchanged. They establish the baseline against
which V1.1 is qualified; they do not by themselves qualify the new public
backend selection feature.

## V1.1 promotion gates

| Gate | Required result | Candidate status |
|---|---:|---:|
| `verifyC172sFlightDynamicsAutopilotLabInstallation` | PASS | static check required locally |
| `auditC172sFlightControlLab` | 57/57 PASS | pending local MATLAB run |
| `auditC172sFlightControlLabBackends` | 8/8 PASS | pending clean-session Simulink run |
| `runAllNonlinearAutopilotChecks` | 28/28 PASS | pending regression confirmation |
| Quick-demo `COMPARE` run | overall PASS and equivalence PASS | pending manual inspection |

The backend audit uses a non-trivial 20-second HEADING/ALTITUDE mission. It
requires identical discrete engagement/mode/event/protection signals and a
maximum absolute difference of `3.0e-7` across 26 states plus 32 diagnostics.
It also clears the model input before execution and verifies that the public
runner leaves no base-workspace residue.

Use `V1_1_QUALIFICATION.md` for the exact commands and evidence to return.
