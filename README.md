# C172S Flight Dynamics and Autopilot Laboratory

- **Candidate:** `v1.1.0-candidate`
- **Laboratory artifact:** `C172S-FLIGHT-CONTROL-LAB-V1-MATLAB-R4`
- **Controller release:** `v1.0.0`
- **Nonlinear 6-DOF release:** `v0.1.0`
- **Nonlinear-autopilot integration:** `v0.1.0`
- **Frozen external evidence:** `28/28 PASS`
- **Inherited V1.0 laboratory audit:** `52/52 PASS`
- **V1.1 qualification:** run locally before promotion

This package turns the complete C172S-inspired flight-controls project into one
operable MATLAB laboratory. It provides a single menu for predefined missions,
bounded custom missions, autopilot comparisons, disturbance and fallback
experiments, plots, exports, selectable MATLAB/Simulink execution, direct
58-signal backend comparison, scope information, and regression access.

> This is a bounded near-trim research and portfolio simulation. It is not a
> certified Cessna model, flight-training device, flight-planning tool, or
> aircraft-approved control system.

## Start here

1. Extract the ZIP to a normal writable folder.
2. Open MATLAB and make the extracted folder the current folder.
3. Run:

```matlab
session = START_HERE;
```

The launcher performs a fast, non-simulating installation check and then opens
the laboratory menu. The underlying public entry point remains:

```matlab
session = mainC172sFlightControlLab;
```

See `QUICK_START.md` for the recommended first session.

## What the laboratory operates

The released controller stack and nonlinear plant are not retuned or rewritten
by the laboratory. The public layer creates bounded request, wind and gust
histories; executes the 26-state nonlinear closed loop through either the
MATLAB reference or independent executable Simulink model; analyzes the trace;
plots command, attitude, rate, control, mode and health evidence; compares all
58 public signals; and can export MAT, CSV and text summaries.

| Layer | Frozen content |
|---|---|
| Aircraft | One 13-state quaternion/NED nonlinear research plant |
| Controller | 13 released controller and actuator states |
| Supervisor | OFF, ROLL, HEADING, PITCH and ALTITUDE modes |
| Execution | MATLAB reference or independent Simulink; hybrid RK4, fixed step `0.010 s` |
| Trim | 4000 ft ISA, 110 KTAS, 2550 lb, throttle `0.65` |
| Interface | 19 supervisor requests, NED wind and NED gust |
| Output | 26 states plus 32 diagnostic signals |

ROLL and PITCH are capture modes. They capture the current bank and pitch at
mode entry; they do not expose arbitrary bank or pitch selections through the
frozen 19-column request contract. HEADING and ALTITUDE expose bounded selected
references.

## Main menu

1. Quick heading and altitude demonstration
2. Build and run a custom nonlinear mission
3. Compare autopilot ON versus OFF
4. Run a disturbance, fallback or disconnect preset
5. Open the executable nonlinear-autopilot Simulink model
6. View system scope, limits and frozen releases
7. Run the complete 28/28 regression gate
8. Run the V1.1 laboratory and backend qualification
9. Exit laboratory

Each scenario run asks for an execution mode:

| Choice | Purpose |
|---|---|
| `MATLAB` | Fast default reference execution and normal lab work |
| `SIMULINK` | Execute the independent model through the same plots, metrics and exports |
| `COMPARE` | Run both, compare 58 signals against `3.0e-7`, and report equivalence |

## Validation status

The release preserves the following independently recorded evidence:

| Evidence layer | Result |
|---|---:|
| Frozen controller release | 18/18 PASS |
| Frozen nonlinear 6-DOF release | 7/7 PASS |
| Nonlinear-autopilot integration gates | 3/3 PASS |
| Combined project regression | 28/28 PASS |
| Public laboratory audit, V1.0 R2 | 52/52 PASS |
| Manual V1.0 R2 acceptance | PASS |
| Public V1.1 R4 audit | 57/57 required; PENDING LOCAL RUN |
| Public MATLAB/Simulink backend audit | 8/8 required; PENDING CLEAN-SESSION RUN |

Manual R2 acceptance covered bumpless ROLL/PITCH engagement, wind and vertical
gust rejection, confirmed heading-source fallback, pilot disconnect and surface
release, plots, event markers and mode-aware reporting. See
`ACCEPTANCE_EVIDENCE.md`.

For V1.1 qualification, follow `V1_1_QUALIFICATION.md`. The two new gates are:

```matlab
clear functions
results = auditC172sFlightControlLab;
backendResults = auditC172sFlightControlLabBackends;
```

For the complete nonlinear-autopilot regression gate:

```matlab
results = runAllNonlinearAutopilotChecks;
```

## Software

The V1.0 baseline was validated in MATLAB/Simulink R2026a. V1.1 retains MATLAB
as its default backend and adds first-class SIMULINK and COMPARE paths that
must be qualified in the target environment before the candidate is promoted.
Simulink is required for those paths and executable regression gates. Control System Toolbox is required by the
linear design and robustness analyses. Stateflow support is used when building
or inspecting generated MATLAB Function blocks.

## Deliberately flat runtime layout

MATLAB source, generated `.slx` release models, frozen reports and robustness
tables remain together at the package root. This preserves the already-audited
function lookup and Simulink callback behavior. Machine-local `.slxc` files,
`slprj`, previous release packages, editor backups, scratch scripts and old run
exports are intentionally excluded.

## Scope and limitations

- Near-trim, attached-flow coefficient model rather than full-envelope aircraft truth.
- One-point thrust with fixed throttle `0.65`; no autothrottle or airspeed hold.
- Ideal sensor interfaces; no bias, noise, quantization, delay failures beyond the explicitly modeled accepted delays.
- Bounded custom command changes: heading within `+/-10 deg`, altitude within `+/-50 ft`.
- Automatic surface authority remains within the released `+/-5 deg` research bounds.
- No stall/spin, takeoff/landing, icing, propulsion transient, structural, hardware-in-the-loop, flight-test or certification claim.

Detailed frozen design history remains in the included controller, integrated
autopilot and nonlinear 6-DOF release documents.
