function data = c172sNearTrimAerodynamicsData()
%C172SNEARTRIMAERODYNAMICSDATA Near-trim coefficient compatibility data.

source = c172sNonlinear6DofSourceData();
kernel = c172sRigidBody6DofData();
airData = c172sAtmosphereAirDataData();
longitudinal = c172sLongitudinalData();
lateral = c172sLateralDirectionalData();

data.artifactRevision = ...
    'NONLINEAR-6DOF-V1-NEAR-TRIM-AERODYNAMICS-MATLAB-R1';
data.required.sourceAudit = source.artifactRevision;
data.required.rigidBodyKernel = kernel.artifactRevision;
data.required.atmosphereAirData = airData.artifactRevision;
data.required.longitudinalPlant = longitudinal.model.version;
data.required.lateralPlant = lateral.model.version;
data.required.masterRegressionPasses = 18;

data.airDataModel = airData;
data.longitudinal = longitudinal;
data.lateral = lateral;

data.trim.V_mps = longitudinal.trim.V_mps;
data.trim.altitude_m = longitudinal.trim.altitude_m;
data.trim.qbar_Pa = longitudinal.trim.qbar_Pa;
data.trim.CL = longitudinal.trim.CL;
data.trim.CD = longitudinal.trim.CD;
data.trim.alpha_rad = 0.0;
data.trim.beta_rad = 0.0;

data.geometry.S_m2 = longitudinal.geometry.S_m2;
data.geometry.cBar_m = longitudinal.geometry.cBar_m;
data.geometry.b_m = lateral.geometry.b_m;

data.controls.names = { ...
    'deltaE_rad_TE_down','deltaA_rad_rightWingDown','deltaR_rad_noseRight'};
data.controls.count = 3;

% The layer returns perturbation aerodynamics only. Equilibrium lift, drag,
% thrust and weight balancing belong to the future trim/total-force layer.
data.output.forceDefinition = ...
    'incremental non-equilibrium aerodynamic body force [X;Y;Z]';
data.output.momentDefinition = ...
    'incremental aerodynamic body moment [L;M;N]';
data.output.equilibriumForcesIncluded = false;

data.envelope.alpha_rad = deg2rad(source.envelope.alpha_deg);
data.envelope.beta_rad = deg2rad(source.envelope.beta_deg);
data.envelope.V_mps = source.envelope.V_KTAS*0.514444444444444;
data.envelope.flap_deg = source.envelope.flap_deg;
data.envelope.schedulingImplemented = false;
data.envelope.stallImplemented = false;
data.envelope.postStallImplemented = false;

data.validation.finiteDifferenceSteps = [ ...
    1.0e-3, ...  % speed perturbation, m/s
    1.0e-6, ...  % alpha, rad
    1.0e-6, ...  % beta, rad
    1.0e-6, ...  % p, rad/s
    1.0e-6, ...  % q, rad/s
    1.0e-6, ...  % r, rad/s
    1.0e-6, ...  % alphaDot, rad/s
    1.0e-6, ...  % elevator, rad
    1.0e-6, ...  % aileron, rad
    1.0e-6];     % rudder, rad
data.validation.maximumDimensionalDerivativeRelativeError = 2.0e-6;
data.validation.maximumMatrixAbsoluteError = 2.0e-8;
data.validation.maximumTrimResidual = 1.0e-10;
data.validation.requireDeterministicReplay = true;
data.validation.propulsionIncluded = false;
data.validation.aerodynamicSchedulingIncluded = false;
data.notForAircraftApproval = true;

end
