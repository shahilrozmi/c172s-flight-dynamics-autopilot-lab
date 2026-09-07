function interface = c172sNonlinearAutopilotInterfaceData()
%C172SNONLINEARAUTOPILOTINTERFACEDATA Frozen controller/plant boundary.
%
% This contract joins the released Integrated Autopilot V1 controller stack
% to the separately released C172S Nonlinear 6-DOF Research Validation Plant
% V0.1. It freezes units, signs, offsets, ownership and validity semantics.
% It does not yet implement or validate a closed-loop Simulink model.

interface.version = 'Nonlinear Autopilot Interface V1';
interface.artifactRevision = ...
    'NONLINEAR-AUTOPILOT-V1-INTERFACE-AUDIT-MATLAB-R1';
interface.architecture = [ ...
    'Frozen autopilot actual actuator positions drive the frozen nonlinear ', ...
    'plant; an ideal deterministic adapter converts quaternion/NED plant ', ...
    'outputs into the existing controller and supervisor feedback channels'];

%% Frozen release boundary

interface.required.flightControlsRelease = 'v1.0.0';
interface.required.flightControlRegressionPasses = 18;
interface.required.integratedAutopilot = ...
    'INTEGRATED-AUTOPILOT-V1-INTERFACE-AUDIT-MATLAB-R1';
interface.required.modeManager = ...
    'AUTOPILOT-MODE-MANAGER-V1-AUDIT-MATLAB-R1';
interface.required.nonlinearPlantRelease = 'v0.1.0';
interface.required.nonlinearRegressionPasses = 7;
interface.required.combinedProjectPasses = 25;
interface.required.nonlinearDynamics = ...
    'NONLINEAR-6DOF-V1-INTEGRATED-DYNAMICS-AND-LINEARIZATION-MATLAB-R1';
interface.required.sourceConvention = ...
    'NONLINEAR-6DOF-V1-SOURCE-AND-CONVENTION-AUDIT-MATLAB-R1';
interface.required.elevatorLoop = 'Pitch-Attitude Hold V2';
interface.required.aileronAudit = 'AILERON-V1-AUDIT-MATLAB-R1';
interface.required.rudderAudit = 'YD-V2-AUDIT-MATLAB-R1';

interface.policy.modifyFlightControlsRelease = false;
interface.policy.modifyNonlinearPlantRelease = false;
interface.policy.retuneControllersAtInterfaceStage = false;
interface.policy.claimAircraftApproval = false;

%% Deterministic execution contract

interface.execution.solver = 'ode4';
interface.execution.fixedStep_s = 0.010;
interface.execution.supervisorSampleTime_s = 0.010;
interface.execution.controllerSampleTime_s = 0.010;
interface.execution.requireDeterministicReplay = true;

%% Plant state and control contracts

interface.plant.stateNames = { ...
    'u_mps','v_mps','w_mps', ...
    'p_radps','q_radps','r_radps', ...
    'positionNorth_m','positionEast_m','positionDown_m', ...
    'q0','q1','q2','q3'};
interface.plant.stateCount = 13;
interface.plant.quaternionOrder = 'scalar-first body-to-NED';
interface.plant.navigationFrame = 'NED';

interface.plant.controlNames = { ...
    'deltaE_rad_TE_down','deltaA_rad_rightWingDown', ...
    'deltaR_rad_noseRight','throttle_0_to_1'};
interface.plant.controlCount = 4;
interface.plant.controlLower = [deg2rad(-5);deg2rad(-5);deg2rad(-5);0];
interface.plant.controlUpper = [deg2rad(5);deg2rad(5);deg2rad(5);1];

interface.trim.altitude_ft = 4000.0;
interface.trim.altitude_m = 1219.2;
interface.trim.TAS_KTAS = 110.0;
interface.trim.TAS_mps = 56.5888888888889;
interface.trim.weight_lb = 2550.0;
interface.trim.surfacePosition_rad = zeros(3,1);
interface.trim.throttle = 0.65;
interface.trim.controlVector = [interface.trim.surfacePosition_rad; ...
    interface.trim.throttle];

%% Controller-to-plant control adapter

interface.controls.controllerActualSurfaceNames = { ...
    'elevatorActual_rad_TE_down'; ...
    'aileronActual_rad_rightWingDown'; ...
    'rudderActual_rad_noseRight'};
interface.controls.controllerActualSurfaceUnit = 'rad';
interface.controls.mapping = [ ...
    'plantSurface = trimSurface + controllerActualSurface; ', ...
    'plantThrottle = fixedTrimThrottle'];
interface.controls.surfaceMappingMatrix = eye(3);
interface.controls.surfaceTrimOffset_rad = zeros(3,1);
interface.controls.fixedThrottle = 0.65;
interface.controls.throttleOwner = 'frozen nonlinear V0.1 trim assumption';
interface.controls.autothrottleImplemented = false;
interface.controls.inputRepresentsActualSurfacePosition = true;
interface.controls.adapterAddsActuatorDynamics = false;
interface.controls.adapterAddsRateLimiting = false;
interface.controls.adapterAddsSaturation = false;
interface.controls.outOfBoundsPolicy = 'reject; do not silently clip';
interface.controls.actuatorDynamicsOwner = ...
    'released protected elevator, aileron and rudder controller stacks';
interface.controls.pilotSurfaceSummationImplemented = false;
interface.controls.pilotOverrideStatusRemainsExternal = true;

interface.controls.signs.elevator = 'positive trailing-edge down';
interface.controls.signs.aileron = 'positive right-wing-down command';
interface.controls.signs.rudder = 'positive nose-right command';
interface.controls.signs.throttle = 'positive increases forward thrust';

%% Supervisor-to-controller command adapter

interface.commands.managerNames = { ...
    'bankCommand_deg'; ...
    'pitchCommand_deg'; ...
    'headingCommand_deg'; ...
    'altitudeCommand_ft'};
interface.commands.controllerNames = { ...
    'bankCommand_rad'; ...
    'pitchCommand_rad'; ...
    'headingCommand_rad'; ...
    'altitudeCommandPerturbation_m'};
interface.commands.count = 4;
interface.commands.headingMapping = ...
    'headingCommand_rad = mod(deg2rad(headingCommand_deg),2*pi)';
interface.commands.altitudeMapping = [ ...
    'altitudeCommandPerturbation_m = ', ...
    '(altitudeCommand_ft - 4000 ft)*0.3048'];
interface.commands.adapterAddsFiltering = false;
interface.commands.enableAndTrackingSignalsPassUnchanged = true;

%% Plant-to-controller feedback adapter

interface.feedback.controllerNames = { ...
    'q_radps'; ...
    'theta_rad'; ...
    'altitudePerturbation_m'; ...
    'verticalSpeed_mps'; ...
    'p_radps'; ...
    'r_radps'; ...
    'phi_rad'; ...
    'psi_rad'};
interface.feedback.controllerCount = 8;
interface.feedback.controllerUnits = { ...
    'rad/s','rad','m','m/s','rad/s','rad/s','rad','rad'};

interface.feedback.supervisorNames = { ...
    'bank_deg'; ...
    'pitch_deg'; ...
    'heading_deg'; ...
    'altitude_ft'};
interface.feedback.supervisorCount = 4;
interface.feedback.supervisorUnits = {'deg','deg','deg','ft'};

interface.feedback.monitorNames = { ...
    'TAS_mps'; ...
    'alpha_rad'; ...
    'beta_rad'; ...
    'quaternionNorm'; ...
    'withinV01Envelope'};
interface.feedback.monitorCount = 5;

interface.feedback.altitudeDefinition = ...
    'altitude_m = -positionDown_m';
interface.feedback.verticalSpeedDefinition = ...
    'verticalSpeed_mps = -positionDownDot_mps; positive is climb';
interface.feedback.altitudePerturbationDefinition = ...
    'altitudePerturbation_m = altitude_m - 1219.2 m';
interface.feedback.headingDefinition = ...
    'psi wrapped to [0,2*pi) for controllers and [0,360) deg for supervisor';
interface.feedback.eulerSequence = '3-2-1 yaw-pitch-roll';

interface.conversions.radToDeg = 180/pi;
interface.conversions.degToRad = pi/180;
interface.conversions.mToFt = 1/0.3048;
interface.conversions.ftToM = 0.3048;

%% Validity, initialization and ownership

interface.validity.maximumQuaternionNormError = 1.0e-8;
interface.validity.minimumLongitudinalAirspeed_mps = 1.0e-4;
interface.validity.coreSensors = ...
    'finite p/q/r and valid normalized quaternion-derived phi/theta';
interface.validity.heading = ...
    'finite valid quaternion-derived psi';
interface.validity.altitude = 'finite NED down position';
interface.validity.airData = 'finite valid atmosphere/air-data output';
interface.validity.aileronSaturatedOwner = 'frozen aileron protection path';
interface.validity.elevatorSaturatedOwner = 'frozen elevator protection path';
interface.validity.trimStableOwner = 'test harness or future pilot model';
interface.validity.pilotOverrideOwner = 'external ideal Boolean interface';

interface.wind.steadyInput = 'three-component NED air-mass velocity in m/s';
interface.wind.gustInput = 'three-component NED gust velocity in m/s';
interface.wind.adapterRotatesWind = false;
interface.wind.plantAirDataOwnsRotation = true;

interface.initialization.state = ...
    'use the frozen nonlinear trim state at 4000 ft ISA and 110 KTAS';
interface.initialization.controls = ...
    'zero trim surface offsets and throttle 0.65';
interface.initialization.controllerIntegrators = ...
    'retain existing one-sample tracking/preload contracts';
interface.initialization.heading_deg = 0.0;
interface.initialization.altitude_ft = 4000.0;

%% V0.1 boundary and next-stage gates

interface.envelope.V_KTAS = [80.0,130.0];
interface.envelope.alpha_deg = [-4.0,10.0];
interface.envelope.beta_deg = [-8.0,8.0];
interface.envelope.bank_deg = [-30.0,30.0];
interface.envelope.pitch_deg = [-10.0,10.0];
interface.envelope.flap_deg = 0.0;

interface.validation.minimumAuditCases = 36;
interface.validation.maximumTrimMappingError = 1.0e-11;
interface.validation.maximumAngleConversionError = 1.0e-12;
interface.validation.maximumUnitConversionError = 1.0e-10;
interface.validation.requireDeterministicReplay = true;
interface.validation.nextExecutableMustUseSingleCoupledPlant = true;
interface.validation.nextExecutableMustPreserveActuatorProtection = true;
interface.validation.nextExecutableMustPreserveModeManager = true;

interface.assumptions = { ...
    'Controller sensor inputs are ideal deterministic signals at this stage.'; ...
    'The released controller actuator outputs represent actual surfaces in radians.'; ...
    'The V0.1 trim surface positions are exactly zero, so perturbation and absolute surface positions coincide.'; ...
    'Throttle remains fixed at the one-point trim value because no autothrottle exists.'; ...
    'Pilot force, control-column and pedal motion remain outside the interface.'};

interface.limitations = [ ...
    'This contract freezes software interfaces only. It does not prove ', ...
    'closed-loop nonlinear stability, preserve performance away from trim, ', ...
    'identify sensor/servo hardware, add an autothrottle or pilot-control ', ...
    'model, validate stall/spin behavior, or constitute aircraft approval.'];
end
