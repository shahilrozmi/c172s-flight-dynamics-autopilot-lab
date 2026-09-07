# C172S Nonlinear 6-DOF Research Validation Plant V0.1

## Release status

- Nonlinear qualification gates: **7/7 PASS**
- Frozen flight-controls v1.0.0 evidence: **18/18 PASS** (external and unchanged)
- Combined project evidence: **25/25 PASS**
- Executable model: `C172S_Nonlinear6Dof_V1.slx`
- Solver: fixed-step `ode4`, 0.010 s

## Implemented scope

This release contains a thirteen-state, quaternion-based, coupled rigid-body
plant using SI units, right-handed body axes, and NED navigation coordinates.
It includes:

- translational and rotational rigid-body equations with nonzero `Ixz`;
- 1976 ISA troposphere atmosphere and body/NED wind transformation;
- TAS, EAS, Mach, Reynolds number, dynamic pressure, alpha and beta air data;
- near-trim aerodynamic forces and moments anchored to the released
  longitudinal and lateral-directional linear plants;
- bounded level-flight trim at 4000 ft ISA, 110 KTAS and 2550 lb;
- one-point calibrated engineering thrust at the frozen trim condition;
- complete thirteen-state nonlinear integration and numerical linearization;
- an independently encoded Simulink executable checked against hybrid RK4
  references for trim, longitudinal, lateral, combined, control and wind/gust
  captures.

## Frozen V0.1 validation envelope

- Airspeed: 80-130 KTAS
- Angle of attack: -4 to +10 deg
- Sideslip: -8 to +8 deg
- Configuration: flaps up
- Initial validated datum: 4000 ft ISA, 110 KTAS, 2550 lb

## Explicit limitations

V0.1 is a research validation plant, not aircraft truth and not aircraft
approval. Aerodynamic coefficients remain frozen near-trim anchors rather than
validated schedules. Mass properties are provisional. Propulsion is a
one-point calibrated thrust assumption; no engine/propeller map, slipstream,
torque or propulsive moment model is claimed. Ground contact, landing gear,
fuel burn, flap schedules, icing, compressibility corrections, sensor errors,
stall, departure, post-stall and spin dynamics are outside this release.

## Reproduction

Place all packaged files in one MATLAB folder and run:

```matlab
results = runAllNonlinear6DofChecks;
```

To create a new non-destructive release package after the regression passes:

```matlab
release = prepareC172sNonlinear6DofRelease('v0.1.0');
```

Existing release folders and ZIP archives are never overwritten.
