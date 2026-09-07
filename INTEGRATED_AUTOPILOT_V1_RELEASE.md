# C172S Flight Controls — Integrated Autopilot V1 Engineering Release

- **Release designation:** C172S Flight Controls and Integrated Autopilot V1
- **Version:** `v1.0.0`
- **Release date:** 2026-08-31
- **Release status:** PASS — frozen engineering baseline
- **Master regression:** **18/18 PASS**

## 1. Release statement

Integrated Autopilot V1 is accepted as the project's complete educational control-system baseline. It joins the frozen longitudinal and lateral-directional plants, protected inner loops, altitude and heading outer loops, and deterministic two-axis Mode Manager in one reproducibly generated Simulink executable.

The final master regression passed all 18 ordered gates. The integrated executable separately passed all 13 declared scenarios while reproducing the standalone longitudinal and lateral implementations with zero measured implementation error.

The release demonstrates internal mathematical consistency, explicit interface control, independent executable checking, deterministic supervisory behavior, and repeatable simulation at one declared trim point. It is not an exact or certified Cessna 172S model and is not approved for aircraft installation, operation, dispatch, or flight planning.

## 2. Released system architecture

| Layer | Function | Frozen artifact |
|---|---|---|
| Longitudinal plant | Four-state small-perturbation dynamics | `Longitudinal Plant V1` |
| Pitch damping | Pitch-rate feedback | `Pitch-Rate Damper V1` |
| Pitch attitude | Filtered PI attitude loop | `Pitch-Attitude Hold V1` |
| Elevator protection | Lag, position/rate protection, tracking anti-windup | `Pitch-Attitude Hold V2` |
| Altitude | Bounded altitude/vertical-speed cascade | `Altitude Hold V1` |
| Lateral plant | Five-state lateral-directional perturbation dynamics | `Lateral-Directional Plant V0.1` |
| Yaw damping | Washout-filtered yaw-rate feedback | `YD-V1-MATLAB-R2` |
| Rudder protection | Lag, authority/rate/hard-stop checks | `YD-V2-AUDIT-MATLAB-R1` |
| Aileron protection | Lag, authority/rate/hard-stop checks | `AILERON-V1-AUDIT-MATLAB-R1` |
| Roll attitude | Protected PI roll loop | `RAH-V1-MATLAB-R1` |
| Heading sensing | Filtering, delay, circular-error assumptions | `HEADING-SENSOR-V1-AUDIT-MATLAB-R1` |
| Heading hold | Bounded proportional circular-error outer loop | `HH-V1-MATLAB-R1` |
| Supervision | Deterministic two-axis mode and protection logic | `AUTOPILOT-MODE-MANAGER-V1-AUDIT-MATLAB-R1` |
| Integration | Simultaneous execution contract and scenario ledger | `INTEGRATED-AUTOPILOT-V1-INTERFACE-AUDIT-MATLAB-R1` |

The supervisor permits one lateral mode (`OFF`, `ROLL`, or `HEADING`) and one vertical mode (`OFF`, `PITCH`, or `ALTITUDE`) at a time. It captures references on engagement, emits one-sample integrator-tracking requests on engagement and mode changes, degrades an invalid outer source to the corresponding inner attitude mode, and globally disconnects for higher-priority protection events.

## 3. Common execution baseline

- Trim condition: 4000 ft pressure altitude, ISA, 110 KTAS, 2550 lb.
- Integrated solver: fixed-step `ode4`.
- Integrated fixed step: 0.010 s.
- Supervisor sample time: 0.010 s.
- Integrated verification signals: 90 (`20 supervisor + 18 longitudinal + 52 lateral`).
- Simultaneous validation command: 30 deg heading and 100 ft altitude.
- Bank-command authority: +/-20 deg.
- Pitch-command authority: +/-3 deg.
- Implementation-equivalence tolerance: `1e-5`.
- Integrated altitude validation horizon: 180 s.

The longitudinal and lateral plants execute simultaneously, but their aerodynamic states are not cross-coupled. Shared time and supervision do not constitute a six-degree-of-freedom aircraft model.

## 4. Master release-gate matrix

| # | Domain | Gate | Checker | Result |
|---:|---|---|---|---|
| 1 | Longitudinal | Longitudinal plant | `runSimulinkPlantCheck.m` | PASS |
| 2 | Longitudinal | Pitch-rate damper V1 | `runPitchRateDamperSimulinkCheck.m` | PASS |
| 3 | Longitudinal | Pitch-attitude hold V1 | `runPitchAttitudeHoldSimulinkCheck.m` | PASS |
| 4 | Longitudinal | Pitch-attitude hold V2 actuator protection | `runPitchAttitudeActuatorSimulinkCheck.m` | PASS |
| 5 | Longitudinal | Altitude hold V1 | `runAltitudeHoldSimulinkCheck.m` | PASS |
| 6 | Lateral-directional | Lateral plant V0.1 MATLAB audit | `runLateralDirectionalPlantValidation.m` | PASS |
| 7 | Lateral-directional | Lateral plant V0.1 Simulink implementation | `runLateralDirectionalSimulinkCheck.m` | PASS |
| 8 | Lateral-directional | Yaw Damper V1 MATLAB audit | `designYawDamper.m` | PASS |
| 9 | Lateral-directional | Yaw Damper V1 Simulink implementation | `runYawDamperSimulinkCheck.m` | PASS |
| 10 | Lateral-directional | Yaw Damper V2 actuator/protection audit | `designYawDamperActuatorProtection.m` | PASS |
| 11 | Lateral-directional | Yaw Damper V2 protected Simulink implementation | `runYawDamperActuatorSimulinkCheck.m` | PASS |
| 12 | Lateral-directional | Roll-Attitude Hold V1 MATLAB audit | `designRollAttitudeHold.m` | PASS |
| 13 | Lateral-directional | Roll-Attitude Hold V1 protected Simulink implementation | `runRollAttitudeHoldSimulinkCheck.m` | PASS |
| 14 | Lateral-directional | Heading Hold V1 protected Simulink implementation | `runHeadingHoldSimulinkCheck.m` | PASS |
| 15 | Supervisory | Mode Manager V1 MATLAB logic audit | `auditAutopilotModeManager.m` | PASS |
| 16 | Supervisory | Mode Manager V1 executable Simulink implementation | `runAutopilotModeManagerSimulinkCheck.m` | PASS |
| 17 | Integrated | Integrated Autopilot V1 interface/scenario audit | `auditIntegratedAutopilotInterfaces.m` | PASS |
| 18 | Integrated | Integrated Autopilot V1 executable Simulink implementation | `runIntegratedAutopilotSimulinkCheck.m` | PASS |

Domain totals are 5/5 longitudinal, 9/9 lateral-directional, 2/2 supervisory, and 2/2 integrated.

## 5. Integrated executable evidence

| Scenario | Required behavior | Result |
|---|---|---|
| Disengaged trim | Both axes off; automatic elevator, aileron, and rudder released | PASS |
| Roll plus pitch engagement | Captured references and one-sample tracking requests | PASS |
| Heading plus altitude acquisition | Simultaneous 30 deg and 100 ft commands converge | PASS |
| Heading to roll | Captured-bank transfer with bounded aileron discontinuity | PASS |
| Altitude to pitch | Captured-pitch transfer with bounded elevator discontinuity | PASS |
| Heading-source fallback | Confirmed loss selects roll; altitude remains active | PASS |
| Altitude-source fallback | Confirmed loss selects pitch; heading remains active | PASS |
| Pilot disconnect | Immediate global release within actuator gates | PASS |
| Disconnect then re-engage | Recapture succeeds and both actuator-rate limits remain enforced | PASS |
| Pilot override | Immediate global-disconnect priority | PASS |
| Sustained aileron saturation | Both axes disconnect after declared dwell | PASS |
| Sustained elevator saturation | Both axes disconnect after declared dwell | PASS |
| Deterministic replay | Simultaneous two-axis trace is bitwise identical | PASS |

Measured final evidence:

- Independent longitudinal maximum error: `0`.
- Independent lateral maximum error: `0`.
- Final heading error: `0.000000 deg`.
- Final altitude error: `0.469268 ft`.
- Heading-to-roll aileron discontinuity residual: `0.00000 deg`.
- Altitude-to-pitch elevator discontinuity residual: `0.00000 deg`.
- Pilot-disconnect release residuals: `0.00001 deg` aileron and `0.00000 deg` elevator.

## 6. Supervisory protection contract

The executable manager preserves the passed MATLAB audit behavior:

- default engagement: `ROLL + PITCH`;
- roll wings-level capture below 6 deg bank;
- bank reference limited to +/-20 deg;
- pitch reference limited to +/-3 deg;
- outer-source fallback confirmation: 0.10 s;
- core-sensor disconnect confirmation: 0.10 s;
- sustained actuator-saturation disconnect: 2.00 s;
- logic-stress thresholds: 30 deg absolute bank and 10 deg absolute pitch;
- highest-to-lowest priority: pilot disconnect, pilot override, abnormal attitude, confirmed core-sensor loss, confirmed sustained saturation, outer-source fallback, then mode/reference requests.

These thresholds, timers, Boolean status interfaces, and fallback policies are transparent project selections. They are not claimed as C172S or Garmin certified values.

## 7. Source and assumption classification

### Source-backed or source-informed inputs

- Longitudinal aerodynamic baseline: NASA CR-2605, Appendix A.
- Aircraft configuration reference: FAA Type Certificate Data Sheet 3A12.
- Lateral-directional derivative baseline: C. Kasnakoglu, PLOS ONE 11(10), e0165017 (2016), Tables 1–2, DOI `10.1371/journal.pone.0165017`.
- High-level roll/heading and autopilot operating architecture: Garmin Cessna Nav III/G1000 operating guidance cited in the component ledgers.
- Standard-rate turn reference: FAA Airplane Flying Handbook glossary.

### Engineering assumptions and selections

- Reuse of source dimensionless lateral derivatives at the project trim point.
- Inertia scaling, `Ixz = 0`, and all declared robustness grids.
- Provisional longitudinal control derivative and pitch-inertia uncertainty.
- Fixed incremental thrust at the longitudinal trim condition.
- All controller gains, filters, delays, command authorities, actuator lags, surface-rate limits, and automatic-control surface limits.
- Sensor noise, bias, delay, and validity models.
- Mode codes, capture rules, tracking handshakes, confirmation timers, disconnect thresholds, priorities, and fallbacks.
- Actuator transfer/release acceptance gates of 0.10 deg.

The MATLAB data ledgers keep these assumptions visible so a later source improvement can replace them without obscuring provenance.

## 8. Scope limitations

V1 is a linear, small-perturbation, single-trim control-system study. Its final executable contains two validated plant stacks under one supervisor, not a coupled nonlinear aircraft.

Outside the release scope:

- nonlinear full-envelope and six-DOF aerodynamics;
- aerodynamic cross-axis coupling and bank-induced altitude coupling;
- automatic throttle, airspeed hold, total-energy control, and propulsion transients;
- wind, gusts, turbulence, crosswind correction, GPS track, navigation, approach, and terrain functions;
- stall, overspeed, load-factor, structural, and full-envelope protection;
- certified sensors, sensor voting, servo clutch/torque behavior, trim runaway, circuit breakers, electrical power, redundant computers, and missed real-time deadlines;
- hardware-in-the-loop, piloted simulation, flight test, certification evidence, or human-factors approval.

The model, plots, gains, limits, and regression results must not be used to command or modify a real aircraft.

## 9. Reproduction and release packaging

Run the authoritative regression from the project root:

```matlab
results = runAllFlightControlChecks;
```

Accept the release only if the console and generated `FlightControlRegressionReport.txt` both report 18/18 PASS. Generated models may open during checks so failure diagnostics remain inspectable.

After the passing report is present, create the clean release package with:

```matlab
release = prepareC172sFlightControlsRelease('v1.0.0');
```

The packager verifies required artifacts, requires 18/18 evidence, copies only releaseable top-level file types, excludes Simulink caches/backups/ZIPs and `untitled.m`, writes `RELEASE_MANIFEST.txt`, and creates a versioned ZIP without modifying the source project.

## 10. Configuration-control rule

Integrated Autopilot V1 is frozen. Any change to plant data, source mapping, signs, units, gains, filters, delays, limits, actuator dynamics, state-machine logic, interfaces, model builders, solver settings, or validation gates requires:

1. rerunning the affected design or source audit;
2. rerunning its independent MATLAB/Simulink implementation check;
3. rerunning `runAllFlightControlChecks.m`;
4. obtaining a fresh 18/18 PASS;
5. generating a new regression report and incrementing the release version.

No component-level pass overrides a failed master gate.

## 11. Release disposition

`v1.0.0` closes the planned single-trim educational flight-controls project through executable two-axis integration. Future work should begin as a separately versioned development line—such as a coupled nonlinear six-DOF plant, wind/gust environment, navigation modes, or hardware-in-the-loop execution—without altering this frozen baseline.
