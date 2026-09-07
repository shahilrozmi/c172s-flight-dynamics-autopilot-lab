# V1.1 Candidate Qualification

Run these commands from the candidate folder in MATLAB/Simulink R2026a or a
compatible environment. Do not copy only the changed files into V1.0; keep the
candidate as a separate release line.

## 1. Clear cached code and verify the package

```matlab
clear functions
clear classes
bdclose('all')
report = verifyC172sFlightDynamicsAutopilotLabInstallation;
```

Required: installation preflight `PASS`, artifact R4, and all required files
present.

## 2. Run the MATLAB laboratory audit

```matlab
clear functions
labResults = auditC172sFlightControlLab;
```

Required: `57/57 PASS`. This confirms presets, boundaries, analysis,
deterministic replay, the optimized reference path, and the new backend option
contract.

## 3. Qualify the public Simulink integration

```matlab
clear functions
bdclose('all')
backendResults = auditC172sFlightControlLabBackends;
```

Required: `8/8 PASS`, cold-start workspace independence, discrete signals
exact, both scenario objectives PASS, and maximum 58-signal error no greater
than `3.0e-7`.

## 4. Reconfirm the frozen regression chain

```matlab
clear functions
backendRegression = runAllNonlinearAutopilotChecks;
```

Required: all three nonlinear-autopilot gates and combined `28/28 PASS`.

## 5. Inspect a full public comparison

```matlab
model = c172sNonlinearAutopilotClosedLoopData;
scenario = createC172sFlightControlScenario('quick-demo',struct(),model);
options = struct('Backend','COMPARE','ShowPlots',true, ...
    'PrintSummary',true,'ExportResults',false,'ExportFolder','');
comparisonRun = runC172sFlightControlScenario(scenario,options,model);
```

Required: scenario objective PASS, overall PASS, backend equivalence PASS, and
the MATLAB/Simulink traces visually coincident at plot scale.

## Evidence to record

Return the final pass counts, maximum 58-signal difference, separate MATLAB and
Simulink quick-demo wall times, and any warning/error text. Promotion to `v1.1.0` should happen only after
all five steps satisfy their required results.
