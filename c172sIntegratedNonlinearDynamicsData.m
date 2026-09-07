function data = c172sIntegratedNonlinearDynamicsData()
%C172SINTEGRATEDNONLINEARDYNAMICSDATA Integrated V0.1 plant contract.

source = c172sNonlinear6DofSourceData();
kernel = c172sRigidBody6DofData();
environment = c172sAtmosphereAirDataData();
aero = c172sNearTrimAerodynamicsData();
trim = c172sTrimTotalForceData();
longitudinal = c172sLongitudinalData();

data.artifactRevision = ...
    'NONLINEAR-6DOF-V1-INTEGRATED-DYNAMICS-AND-LINEARIZATION-MATLAB-R1';
data.required.sourceAudit = source.artifactRevision;
data.required.rigidBodyKernel = kernel.artifactRevision;
data.required.atmosphereAirData = environment.artifactRevision;
data.required.nearTrimAerodynamics = aero.artifactRevision;
data.required.trimTotalForce = trim.artifactRevision;
data.required.longitudinalPlant = trim.required.longitudinalPlant;
data.required.lateralPlant = trim.required.lateralPlant;
data.required.masterRegressionPasses = 18;

% The generic rigid-body kernel was audited with the provisional inertia
% tensor assembled from the lateral source. That source's lateral model does
% not use Iyy. At integrated assembly, use the frozen longitudinal plant's
% independently identified/scaled pitch inertia while preserving Ixx, Izz
% and Ixz. No previously audited data artifact is modified in place.
data.trimModel = trim;
data.massProperties.pitchInertia_kgm2 = longitudinal.inertia.Iy_kgm2;
data.massProperties.pitchInertiaSource = longitudinal.model.version;
data.massProperties.reconciliation = ...
    'Iyy from frozen longitudinal plant; Ixx/Izz/Ixz from frozen lateral plant';
data.trimModel.kernel.inertia.Iyy_kgm2 = longitudinal.inertia.Iy_kgm2;
data.trimModel.kernel.inertia.matrix_kgm2(2,2) = longitudinal.inertia.Iy_kgm2;

% The frozen longitudinal equations place equilibrium thrust in the
% flight-path speed balance and contain no trim-thrust normal-force term.
% With propeller-axis/incidence data unavailable, V0.1 extends the calibrated
% one-point thrust along the local longitudinal air-velocity direction. This
% is a compatibility convention, not a physical propeller-axis claim.
data.propulsion.forceDirection = ...
    'local longitudinal air-velocity direction in the body x-z plane';
data.propulsion.forceDirectionStatus = ...
    'frozen-linear-plant compatibility convention';
data.propulsion.physicalPropellerAxisIdentified = false;
data.state.names = source.state.names;
data.state.count = 13;
data.control.names = trim.controls.names;
data.control.count = 4;
data.wind.definition = 'constant NED air-mass velocity during one RHS evaluation';
data.wind.timeVaryingImplemented = false;

data.alphaDot.method = ...
    'exact affine algebraic consistency solve using evaluations at 0 and 1 rad/s';
data.alphaDot.minimumDenominatorMagnitude = 0.05;
data.alphaDot.maximumConsistencyError_radps = 2e-11;

% Reduced physical perturbation coordinates used only for validation:
% [du alpha q theta beta p r phi psi]. The executable plant remains the
% thirteen-state quaternion model.
data.linearization.reducedStateNames = { ...
    'du_mps','alpha_rad','q_radps','theta_rad', ...
    'beta_rad','p_radps','r_radps','phi_rad','psi_rad'};
data.linearization.controlNames = trim.controls.names;
data.linearization.stateSteps = [1e-3;repmat(1e-6,8,1)];
data.linearization.controlSteps = [1e-6;1e-6;1e-6;1e-6];
data.linearization.rawStateSteps = [repmat(1e-4,3,1); ...
    repmat(1e-6,3,1);repmat(1e-3,3,1);repmat(1e-7,4,1)];
data.linearization.maximumFrozenMatrixError = 2e-7;
data.linearization.maximumForbiddenCrossAxisError = 2e-7;
data.linearization.maximumRawJacobianReplayError = 1e-12;

data.integration.fixedStep_s = 0.01;
data.integration.captureDuration_s = 2.0;
data.integration.maximumLongitudinalLinearDifference = 6e-4;
data.integration.maximumLateralLinearDifference = 6e-4;
data.integration.maximumQuaternionNormError = 2e-12;
data.integration.requireDeterministicReplay = true;

data.envelope = aero.envelope;
data.claim.coupledRigidBodyDynamics = true;
data.claim.aerodynamicCrossDerivativeDatabase = false;
data.claim.coefficientScheduling = false;
data.claim.propellerMap = false;
data.claim.stallOrSpin = false;
data.claim.aircraftTruth = false;
data.notForAircraftApproval = true;

end
