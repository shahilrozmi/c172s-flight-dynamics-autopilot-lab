# V1.1 Qualification Record

The C172S Flight Dynamics and Autopilot Laboratory V1.1 was qualified in MATLAB/Simulink R2026a before promotion to `v1.1.0`. This file preserves both the qualification commands and the recorded results.

## 1. Clear cached code and verify the package

```matlab
clear functions
clear classes
bdclose('all')
report = verifyC172sFlightDynamicsAutopilotLabInstallation;
```

Required: installation preflight `PASS`, artifact R4, and all required files present.

**Recorded result: PASS — `C172S-FLIGHT-CONTROL-LAB-V1-MATLAB-R4`, 20/20 required files.**

## 2. Run the MATLAB laboratory audit

```matlab
clear functions
labResults = auditC172sFlightControlLab;
```

Required: `57/57 PASS`.

**Recorded result: 57/57 PASS.**

## 3. Qualify the public Simulink integration

```matlab
clear functions
bdclose('all')
backendResults = auditC172sFlightControlLabBackends;
```

Required: `8/8 PASS`, cold-start workspace independence, discrete signals exact, both scenario objectives PASS, and maximum 58-signal error no greater than `3.0e-7`.

**Recorded result: 8/8 PASS.**

- Clean-start workspace independence: PASS
- Discrete engagement/mode/event/protection signals: exact
- Maximum 58-signal difference: `4.54747351e-13`
- Acceptance tolerance: `3.0e-7`

The run emitted only benign MATLAB `clear classes` warnings involving loaded `datetime` / `digraph` objects; no qualification error was recorded.

## 4. Reconfirm the frozen regression chain

```matlab
clear functions
backendRegression = runAllNonlinearAutopilotChecks;
```

Required: all three nonlinear-autopilot gates and combined `28/28 PASS`.

**Recorded result: 28/28 PASS.**

## 5. Inspect a full public comparison

```matlab
model = c172sNonlinearAutopilotClosedLoopData;
scenario = createC172sFlightControlScenario('quick-demo',struct(),model);
options = struct('Backend','COMPARE','ShowPlots',true, ...
    'PrintSummary',true,'ExportResults',false,'ExportFolder','');
comparisonRun = runC172sFlightControlScenario(scenario,options,model);
```

Required: scenario objective PASS, overall PASS, backend equivalence PASS, and MATLAB/Simulink traces visually coincident at plot scale.

**Recorded result: PASS.**

- Scenario objective: PASS
- Overall result: PASS
- Backend equivalence: PASS
- Maximum 58-signal difference: `5.00222086e-12`
- MATLAB wall time: `33.37 s`
- Simulink wall time: `2.13 s`
- Total comparison wall time: `35.49 s`

## Promotion decision

All five qualification steps satisfied their required results. The candidate was therefore promoted to **`v1.1.0`**.
