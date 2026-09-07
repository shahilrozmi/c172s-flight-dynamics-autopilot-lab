function data = c172sRigidBody6DofData()
%C172SRIGIDBODY6DOFDATA Data contract for the independent 13-state kernel.
%
% Aerodynamic, propulsion and actuator models are deliberately absent. The
% kernel receives externally applied body-axis force and moment vectors.
% Gravity is supplied internally as a constant NED-down acceleration.

source = c172sNonlinear6DofSourceData();
longitudinal = c172sLongitudinalData();
lateral = c172sLateralDirectionalData();

data.artifactRevision = 'NONLINEAR-6DOF-V1-RIGID-BODY-KERNEL-MATLAB-R1';
data.required.sourceAudit = source.artifactRevision;
data.required.longitudinalPlant = source.required.longitudinalPlant;
data.required.lateralPlant = source.required.lateralPlant;
data.required.masterRegressionPasses = 18;

data.mass_kg = longitudinal.trim.mass_kg;
data.gravity_mps2 = longitudinal.constants.g;

% The inertia matrix uses the conventional body-axis products of inertia:
% I = [ Ixx, -Ixy, -Ixz; -Ixy, Iyy, -Iyz; -Ixz, -Iyz, Izz ].
% The frozen lateral source uses Ixz=0. A nonzero test tensor below audits
% that the general coupled Euler equation is nevertheless implemented.
data.inertia.Ixx_kgm2 = lateral.inertia.Ixx_kgm2;
data.inertia.Iyy_kgm2 = lateral.inertia.Iyy_kgm2;
data.inertia.Izz_kgm2 = lateral.inertia.Izz_kgm2;
data.inertia.Ixz_kgm2 = lateral.inertia.Ixz_kgm2;
data.inertia.matrix_kgm2 = [ ...
    data.inertia.Ixx_kgm2, 0, -data.inertia.Ixz_kgm2; ...
    0, data.inertia.Iyy_kgm2, 0; ...
    -data.inertia.Ixz_kgm2, 0, data.inertia.Izz_kgm2];

data.inertia.status = source.massProperties.status;
data.inertia.uncertaintyFraction = ...
    source.massProperties.uncertaintyFraction;
data.inertia.IxzStatus = source.massProperties.IxzStatus;

testIxz = 80.0;
data.validation.coupledTestInertia_kgm2 = [ ...
    data.inertia.Ixx_kgm2,0,-testIxz; ...
    0,data.inertia.Iyy_kgm2,0; ...
    -testIxz,0,data.inertia.Izz_kgm2];

data.state.names = source.state.names;
data.state.count = source.state.count;
data.state.velocity = 1:3;
data.state.angularRate = 4:6;
data.state.positionNED = 7:9;
data.state.quaternion = 10:13;
data.state.quaternionOrder = source.state.quaternionOrder;

data.frame.body = source.axes.frame;
data.frame.navigation = source.axes.navigation;
data.frame.altitudeDefinition = source.axes.altitudeDefinition;
data.input.forceDefinition = ...
    'non-gravitational externally applied body-axis force [Fx;Fy;Fz]';
data.input.momentDefinition = ...
    'externally applied body-axis moment [L;M;N]';

data.quaternion.minimumNorm = 1.0e-12;
% RK4 stage states do not lie exactly on the unit sphere even when every
% completed step is renormalized. Permit that small internal excursion but
% reject materially non-unit external states.
data.quaternion.maximumInputNormError = 1.0e-2;
data.quaternion.renormalizeAfterIntegratorStep = true;

data.validation.fixedStep_s = 0.01;
data.validation.maximumStaticDerivativeError = 1.0e-12;
data.validation.maximumFreeFallError = 1.0e-10;
data.validation.maximumConservationRelativeError = 2.0e-8;
data.validation.minimumRK4ErrorRatio = 12.0;
data.validation.maximumFineRK4AttitudeError_rad = 1.0e-6;
data.validation.requireDeterministicReplay = true;
data.validation.requireFiniteNearEulerSingularity = true;
data.validation.aerodynamicsIncluded = false;
data.validation.propulsionIncluded = false;
data.validation.actuatorsIncluded = false;

data.modelClaim = source.policy.researchPlantName;
data.notForAircraftApproval = true;

end
