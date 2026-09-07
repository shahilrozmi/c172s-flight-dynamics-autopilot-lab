function lab = c172sFlightControlLabData(model)
%C172SFLIGHTCONTROLLABDATA User-facing laboratory contract and limits.
%
%   LAB = C172SFLIGHTCONTROLLABDATA() returns the public contract for the
%   C172S Flight Dynamics and Autopilot Laboratory V1.1. The laboratory is an
%   orchestration and presentation layer over the frozen nonlinear-autopilot
%   backend; it does not alter any controller, plant, actuator or supervisor.

if nargin < 1 || isempty(model)
    model = c172sNonlinearAutopilotClosedLoopData();
end
assert(isstruct(model) && isfield(model,'artifactRevision') && ...
    strcmp(model.artifactRevision, ...
    'NONLINEAR-AUTOPILOT-V1-CLOSED-LOOP-MATLAB-R1'), ...
    'c172sFlightControlLabData:InvalidBackend', ...
    'The frozen nonlinear-autopilot backend is unavailable or changed.');

lab.name = 'C172S Flight Dynamics and Autopilot Laboratory';
lab.version = 'V1.1';
lab.release = 'v1.1.0-candidate';
lab.artifactRevision = 'C172S-FLIGHT-CONTROL-LAB-V1-MATLAB-R4';
lab.architecture = [ ...
    'One public scenario interface, selectable MATLAB and executable ', ...
    'Simulink backends, and separate analysis, comparison, visualization ', ...
    'and export layers'];

lab.required.controllerRelease = 'v1.0.0';
lab.required.nonlinearPlantRelease = 'v0.1.0';
lab.required.nonlinearAutopilotRelease = 'v0.1.0';
lab.required.externalRegressionEvidence = '28/28 PASS';
lab.required.closedLoopArtifact = model.artifactRevision;
lab.required.interfaceArtifact = ...
    model.dependencies.interface.artifactRevision;
lab.required.modeManagerArtifact = ...
    model.dependencies.manager.artifactRevision;
lab.required.plantArtifact = model.dependencies.plant.artifactRevision;

lab.execution.fixedStep_s = model.execution.fixedStep_s;
lab.execution.backend = 'simulateC172sNonlinearAutopilot';
lab.execution.referenceBackend = 'simulateC172sNonlinearAutopilot';
lab.execution.simulinkBackend = ...
    'simulateC172sNonlinearAutopilotSimulink';
lab.execution.backends = {'MATLAB','SIMULINK','COMPARE'};
lab.execution.defaultBackend = 'MATLAB';
lab.execution.maximumBackendError = 3.0e-7;
lab.execution.simulinkModel = 'C172S_NonlinearAutopilot_V1';
lab.execution.simulinkBuilder = ...
    'buildC172sNonlinearAutopilotSimulink';
lab.execution.regressionRunner = 'runAllNonlinearAutopilotChecks';
lab.execution.backendAudit = 'auditC172sFlightControlLabBackends';

lab.trim.altitude_ft = model.dependencies.interface.trim.altitude_ft;
lab.trim.TAS_KTAS = model.dependencies.interface.trim.TAS_KTAS;
lab.trim.weight_lb = model.dependencies.interface.trim.weight_lb;
lab.trim.heading_deg = ...
    model.dependencies.interface.initialization.heading_deg;
lab.trim.throttle = model.trim.controls(4);

lab.boundary.duration_s = [2,180];
lab.boundary.headingChange_deg = [-10,10];
lab.boundary.altitudeChange_ft = [-50,50];
lab.boundary.engageTime_s = [0,30];
lab.boundary.eventTime_s = [0.1,179.9];
lab.boundary.headwind_mps = [-5,5];
lab.boundary.crosswindFromRight_mps = [-3,3];
lab.boundary.upwardGust_mps = [0,1];
lab.boundary.gustDuration_s = [0.1,10];
lab.boundary.V_KTAS = model.dependencies.interface.envelope.V_KTAS;
lab.boundary.alpha_deg = model.dependencies.interface.envelope.alpha_deg;
lab.boundary.beta_deg = model.dependencies.interface.envelope.beta_deg;
lab.boundary.bank_deg = model.dependencies.interface.envelope.bank_deg;
lab.boundary.pitch_deg = model.dependencies.interface.envelope.pitch_deg;

lab.defaults.duration_s = model.validation.outerDuration_s;
lab.defaults.engage = true;
lab.defaults.engageTime_s = 0;
lab.defaults.lateralMode = 'HEADING';
lab.defaults.verticalMode = 'ALTITUDE';
lab.defaults.headingChange_deg = model.validation.headingCommand_deg;
lab.defaults.altitudeChange_ft = model.validation.altitudeCommand_ft;
lab.defaults.windStart_s = 5;
lab.defaults.headwind_mps = 0;
lab.defaults.crosswindFromRight_mps = 0;
lab.defaults.gustStart_s = 8;
lab.defaults.gustDuration_s = 2;
lab.defaults.upwardGust_mps = 0;
lab.defaults.eventType = 'NONE';
lab.defaults.eventTime_s = 8;

lab.presets.keys = { ...
    'quick-demo'; ...
    'roll-pitch-capture'; ...
    'wind-gust'; ...
    'heading-fallback'; ...
    'altitude-fallback'; ...
    'pilot-disconnect'; ...
    'pilot-override'; ...
    'autopilot-off'};
lab.presets.names = { ...
    'Heading and altitude capture'; ...
    'Bumpless ROLL/PITCH engagement at trim'; ...
    'Wind and vertical-gust rejection'; ...
    'Heading-source loss and ROLL fallback'; ...
    'Altitude-source loss and PITCH fallback'; ...
    'Pilot disconnect and surface release'; ...
    'Pilot override priority'; ...
    'Autopilot-off comparison case'};

lab.analysis.headingTolerance_deg = ...
    model.validation.maximumFinalHeadingError_deg;
lab.analysis.altitudeTolerance_ft = ...
    model.validation.maximumFinalAltitudeError_ft;
lab.analysis.bankCaptureTolerance_deg = 0.10;
lab.analysis.pitchCaptureTolerance_deg = 0.10;
lab.analysis.minimumHeadingSettlingBand_deg = 0.10;
lab.analysis.minimumAltitudeSettlingBand_ft = 1.0;
lab.analysis.minimumBankSettlingBand_deg = 0.10;
lab.analysis.minimumPitchSettlingBand_deg = 0.10;
lab.analysis.maximumQuaternionNormError = ...
    model.validation.maximumQuaternionNormError;
lab.analysis.requireFiniteTrace = true;
lab.analysis.requireNoEnvelopeViolation = true;

lab.export.defaultFolderName = 'FlightControlLabResults';
lab.export.writeMat = true;
lab.export.writeCsv = true;
lab.export.writeTextSummary = true;

lab.validation.minimumAuditCases = 50;
lab.validation.expectedAuditCases = 57;
lab.validation.requireDeterministicReplay = true;
lab.validation.requireFrozenBackend = true;
lab.validation.requireNoninteractivePublicAPI = true;
lab.validation.requireBackendEquivalence = true;

lab.policy.modifyControllerRelease = false;
lab.policy.modifyNonlinearPlantRelease = false;
lab.policy.modifyNonlinearAutopilotRelease = false;
lab.policy.retuneControllers = false;
lab.policy.expandValidatedEnvelope = false;
lab.policy.claimAircraftApproval = false;

lab.limitations = [ ...
    'The laboratory is a user-facing research simulation tool over the ', ...
    'frozen V0.1 nonlinear-autopilot model. Custom inputs are deliberately ', ...
    'bounded near the 4000-ft, 110-KTAS, 2550-lb trim. It retains ideal ', ...
    'sensors, local attached-flow aerodynamics and one-point throttle and ', ...
    'does not represent a certified flight simulator or aircraft approval.'];
end
