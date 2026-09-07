# C172S Longitudinal Autopilot V1 — Engineering Release Summary

- **Release designation:** Longitudinal Autopilot V1
- **Release date:** 2026-08-28
- **Release status:** PASS — engineering baseline frozen
- **Master regression result:** 5/5 release gates passed

## 1. Release statement

The C172S Longitudinal Autopilot V1 is accepted as the project's validated small-perturbation longitudinal-control baseline. The release integrates and verifies the aircraft plant, pitch-rate damping, pitch-attitude control, actuator protection with anti-windup, and bounded altitude capture.

All five MATLAB/Simulink release gates passed through `runAllFlightControlChecks.m`. The runner resolved and executed each checker by full path, captured its console output, and generated `FlightControlRegressionReport.txt` as the detailed regression record.

This release establishes internal mathematical consistency and repeatable simulation behavior. It is an educational and portfolio engineering model; it is not an exact certified Cessna 172S model and is not approved for aircraft operation, dispatch, or flight planning.

## 2. Released architecture

| Layer | Released function | Principal implementation |
|---|---|---|
| Aircraft dynamics | Four-state longitudinal perturbation plant: forward-speed perturbation, angle of attack, pitch rate, and pitch attitude | `c172sLongitudinalData.m`, `c172sLongitudinalPlant.m` |
| Inner damping loop | Pitch-rate feedback using `deltaE = deltaECmd + Kq*q` | Pitch-Rate Damper V1 |
| Attitude loop | Filtered-command PI pitch-attitude hold around the frozen pitch-rate damper | Pitch-Attitude Hold V1 |
| Protected attitude loop | Elevator actuator lag, position authority, rate limiting, and tracking anti-windup | Pitch-Attitude Hold V2 |
| Altitude outer loop | Bounded altitude-to-vertical-speed-to-pitch-command cascade around frozen V2 | Altitude Hold V1 |

Elevator angle uses the explicit convention **trailing-edge down positive**. Altitude increases upward. All controller and plant interfaces use SI units internally unless a displayed metric explicitly states otherwise.

## 3. Release-gate matrix

| Gate | Checker | Required evidence | Release result |
|---|---|---|---|
| Longitudinal plant | `runSimulinkPlantCheck.m` | MATLAB/Simulink equivalence, state ordering, input sign, short-period and phugoid reproduction | PASS |
| Pitch-rate damper V1 | `runPitchRateDamperSimulinkCheck.m` | Closed-loop mode check, state-history equivalence, correct elevator-feedback sign, reduced integrated pitch-rate response | PASS |
| Pitch-attitude hold V1 | `runPitchAttitudeHoldSimulinkCheck.m` | Exact six-state reference equivalence, PI and q-feedback directions, command-response gates, linear-envelope gates | PASS |
| Pitch-attitude hold V2 | `runPitchAttitudeActuatorSimulinkCheck.m` | Three-run RK4 equivalence, nominal response, inactive nominal limits, stress-case anti-windup effectiveness, feedback directions | PASS |
| Altitude hold V1 | `runAltitudeHoldSimulinkCheck.m` | Two-capture RK4 equivalence, kinematic and feedback signs, bounded operational response, intended limit activation, preservation of V2 protection | PASS |

Any future change to a released plant, controller, actuator, builder, or checker requires the complete 5/5 regression gate to be rerun.

## 4. Design baseline

### 4.1 Longitudinal plant

- State vector: `{u_mps, alpha_rad, q_radps, theta_rad}`.
- Control input: `deltaE_rad_TE_down_positive`.
- Principal aerodynamic-derivative reference: NASA CR-2605, Appendix A, pp. 77–79.
- Aircraft configuration reference: FAA Type Certificate Data Sheet 3A12.
- Selected derivative values include `CL_alpha = 5.50`, `CL_alphaDot = 1.49`, `CL_q = 3.88`, `CD_alpha = 0.25`, `Cm_alphaDot = -4.36`, `Cm_q = -11.40`, and `Cm_deltaE = -1.26`.
- The nominal incremental thrust derivative is `dT/dV = 0` for this fixed-thrust longitudinal baseline.
- The provisional elevator lift derivative is `CL_deltaE = 0.427`, with robustness coverage from 0.30 to 0.50.
- Pitch inertia uncertainty covers 1800 to 2800 kg·m² around the selected reference-scaled value.

### 4.2 Pitch-rate damper V1

- Feedback gain: `Kq = 0.1100 s`.
- Validation disturbance: initial pitch rate of 5.00 deg/s.
- Simulink solver: fixed-step `ode4`, 0.010 s.
- Maximum MATLAB/Simulink state error: `2.89131295e-09`.
- RMS MATLAB/Simulink state error: `2.8361896e-10`.
- Closed-loop short-period natural frequency: 6.0089 rad/s.
- Closed-loop short-period damping ratio: 0.8532.
- Phugoid period: 33.395 s.
- Phugoid damping ratio: 0.0712.
- Integrated absolute pitch-rate response reduced from 1.5560 deg to 1.0923 deg.

### 4.3 Pitch-attitude hold V1

- Proportional gain: `Kp = 0.450`.
- Integral gain: `Ki = 0.450 1/s`.
- Frozen q-damper gain: `Kq = 0.110 s`.
- Command-filter time constant: 2.0 s.
- PI zero: 1 rad/s.
- Nominal validation command: 1.0 deg.
- Robust design search: `Kp = Ki` from 0.10 to 0.70 in 0.005 increments.
- Uncertainty validation: 36 cases.
- Minimum fast-mode damping across the design set: 0.61.

### 4.4 Pitch-attitude hold V2 actuator protection

- Attitude gains preserved at `Kp = Ki = 0.450`.
- Integrated pitch-rate gain: `Kq = 0.120 s`.
- Elevator-actuator time constant: 0.035 s.
- Automatic elevator authority: ±5 deg.
- Elevator rate limit: ±20 deg/s.
- Tracking anti-windup: enabled.
- Normal-command validation confirms position and rate limits remain inactive.
- Stress validation confirms anti-windup materially limits integrator growth and saturation.

The actuator time constant, automatic authority, rate limit, and associated protection settings are provisional engineering assumptions rather than C172S-certified actuator data.

### 4.5 Altitude hold V1

- Frozen inner loop: Pitch-Attitude Hold V2.
- Altitude kinematics: `hDot = Vtrim*(theta - alpha)`.
- Altitude gain: `Kh = 0.600 1/s`.
- Vertical-speed gain: `Kv = 0.020 rad/(m/s)`.
- Altitude command-filter time constant: 5.00 s.
- Vertical-speed command authority: ±300 ft/min.
- Pitch-command authority: ±3.0 deg.
- Simulink solver: fixed-step `ode4`, 0.020 s.

## 5. Robustness and performance evidence

### 5.1 Altitude-loop robust design

- Stable uncertainty cases: 108/108.
- Vertical-speed feedback phase margin: 97.14 to 139.05 deg.
- Altitude-loop phase margin: 70.95 to 71.65 deg.
- Altitude-loop gain crossover: 0.3180 to 0.3224 rad/s.
- Attitude-to-altitude bandwidth ratio: 3.181 to 4.281.
- Slowest nominal decay rate: 0.01341 1/s.

### 5.2 Ten-foot small-signal capture

| Metric | Result |
|---|---:|
| Maximum Simulink/reference signal error | `4.61852778e-14` |
| Overshoot | 0.0000% |
| 10–90% rise time | 14.120 s |
| 2% settling time | 55.960 s |
| Final altitude error | 0.03780 ft |
| Peak pitch command | 0.5711 deg |
| Peak aircraft pitch | 0.4636 deg |
| Peak vertical speed | 83.539 ft/min |
| Peak actual elevator | 0.0803 deg |
| Peak actuator rate | 0.0478 deg/s |
| Peak speed perturbation | 0.738% of trim speed |
| Peak angle-of-attack perturbation | 0.0816 deg |
| Active limits | None |

### 5.3 Protected 100-foot capture

| Metric | Result |
|---|---:|
| Maximum Simulink/reference signal error | `5.43565193e-13` |
| Overshoot | 0.0000% |
| 10–90% rise time | 39.000 s |
| 2% settling time | 72.600 s |
| Final altitude error | 0.46927 ft |
| Peak pitch command | 1.7437 deg |
| Peak aircraft pitch | 1.1089 deg |
| Peak vertical speed | 197.454 ft/min |
| Peak actual elevator | 0.4358 deg |
| Peak actuator rate | 0.2605 deg/s |
| Peak speed perturbation | 6.454% of trim speed |
| Peak angle-of-attack perturbation | 0.5760 deg |
| Active limits | Vertical-speed command cap only |

The independent-reference tolerance was `1e-5`; both altitude captures remained many orders of magnitude below this limit.

## 6. Source and assumption classification

### Source-backed inputs

- Publicly documented C172/C172S configuration data where explicitly cited in the project source ledger.
- NASA CR-2605 longitudinal aerodynamic derivatives used to construct the linear perturbation model.
- FAA TCDS 3A12 configuration and certification-reference data.

### Engineering assumptions and design selections

- Provisional `CL_deltaE` and its robustness interval.
- Reference-scaled pitch inertia and its uncertainty interval.
- Fixed incremental thrust derivative for the V1 trim condition.
- Controller gains, command filters, command limits, and uncertainty grids.
- Elevator actuator lag, position authority, rate authority, and anti-windup tracking parameters.

Assumed quantities remain visible in data structures and model annotations so that later source improvements can replace them without obscuring their origin.

## 7. Scope limitations

V1 is intentionally limited to a linear small-perturbation model around one trim condition. It provides elevator-only altitude capture around fixed incremental thrust; it is not total-energy control and does not include automatic throttle control.

The following are outside the released V1 scope:

- Full-envelope nonlinear aerodynamics and large-attitude kinematics.
- Engine and propeller transients, throttle control, and airspeed hold.
- Wind, gusts, turbulence, and atmospheric disturbances.
- Sensor noise, bias, quantization, latency, and failure behavior.
- Structural flexibility, control-system hardware, and certification compliance.
- Terrain clearance, navigation, operational envelopes, and flight-safety functions.

## 8. Released verification artifacts

- `runAllFlightControlChecks.m` — master 5/5 release gate.
- `FlightControlRegressionReport.txt` — generated detailed execution record.
- `runSimulinkPlantCheck.m`.
- `runPitchRateDamperSimulinkCheck.m`.
- `runPitchAttitudeHoldSimulinkCheck.m`.
- `runPitchAttitudeActuatorSimulinkCheck.m`.
- `runAltitudeHoldSimulinkCheck.m`.
- Generated Simulink plant and controller models for all five stages.

## 9. Configuration-control rule

Longitudinal Autopilot V1 is now the frozen reference baseline for subsequent work. Any modification to plant data, signs, units, gains, limits, actuator dynamics, model builders, solver settings, or validation logic requires:

1. Rerunning the affected component-level design validation.
2. Rerunning its MATLAB/Simulink equivalence checker.
3. Rerunning `runAllFlightControlChecks.m`.
4. Obtaining a complete 5/5 PASS before accepting the change.

## 10. Approved next phase

The next development phase is the C172S lateral-directional control baseline:

1. Establish and validate the lateral-directional state-space plant.
2. Verify Dutch-roll, roll-subsidence, and spiral modes and all sign conventions.
3. Develop Yaw Damper V1.
4. Develop Roll-Attitude Hold V1 and protected V2 actuator logic.
5. Develop Heading Hold V1.
6. Integrate the lateral stack into the master regression framework.
