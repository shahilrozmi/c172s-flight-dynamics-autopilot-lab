# Five-Minute Quick Start

## 1. Launch

Extract the release, open MATLAB in the extracted folder, and run:

```matlab
session = START_HERE;
```

The installation preflight should report `PASS`, after which the main menu
appears.

## 2. Run the quick demonstration

Choose menu option `1`. Select `MATLAB` for normal work, then choose whether to
export the result. The aircraft should engage HEADING/ALTITUDE, capture the bounded
heading and altitude changes, remain inside the V0.1 envelope, and report an
overall `PASS`.

## 3. Build a bounded mission

Choose option `2`. The laboratory asks for duration, engagement, lateral and
vertical modes, bounded heading/altitude changes, steady NED wind, a vertical
gust and one optional supervisor event.

- ROLL and PITCH capture the current bank and pitch.
- HEADING and ALTITUDE use the selected heading and altitude.
- Positive crosswind-from-right and upward-gust prompts are converted to the
  declared NED environment convention internally.

## 4. Inspect the result

The console reports final modes, mode-appropriate tracking errors, peak body
rates, peak automatic surfaces, TAS range, envelope violations, quaternion
health, event history and overall status.

The first figure shows guidance and aircraft response. The second shows control
surfaces, air-relative angles, autopilot modes and air-data health. Vertical
markers identify engagement, wind, gust start/end and supervisor events.

## 5. Export or validate

Choosing export after a run writes MAT, CSV and text-summary evidence to
`FlightControlLabResults` beside the release.

Run the public laboratory audit with:

```matlab
clear functions
results = auditC172sFlightControlLab;
```

This R4 audit extends the accepted 52-case R2 ledger with execution-contract
checks. Expected result: `57/57 PASS`.

To exercise Simulink as a real laboratory backend, run one scenario with
`SIMULINK`. To directly qualify equivalence, choose `COMPARE` or run:

```matlab
backendResults = auditC172sFlightControlLabBackends;
```

The backend gate must report `8/8 PASS`, including clean-session workspace
independence, an exact discrete-signal match, and a maximum 58-signal
difference no larger than `3.0e-7`.

Run the full frozen backend gate with menu option `7` or:

```matlab
results = runAllNonlinearAutopilotChecks;
```

Expected combined result: `28/28 PASS`.

See `V1_1_QUALIFICATION.md` for the recorded final V1.1 qualification evidence.
