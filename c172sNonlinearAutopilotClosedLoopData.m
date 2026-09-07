function model = c172sNonlinearAutopilotClosedLoopData()
%C172SNONLINEARAUTOPILOTCLOSEDLOOPDATA Coupled closed-loop audit contract.

interface = c172sNonlinearAutopilotInterfaceData();
manager = c172sAutopilotModeManagerData();
plant = c172sIntegratedNonlinearDynamicsData();
pitch = c172sPitchAttitudeControllerData();
elevator = c172sElevatorActuatorData();
altitude = c172sAltitudeHoldControllerData();
yaw = c172sYawDamperData();
rudder = c172sRudderActuatorData();
roll = c172sRollAttitudeControllerData();
aileron = c172sAileronActuatorData();
headingSensor = c172sHeadingSensorData();
heading = c172sHeadingHoldControllerData();

model.version = 'Nonlinear Autopilot Closed Loop V1';
model.artifactRevision = ...
    'NONLINEAR-AUTOPILOT-V1-CLOSED-LOOP-MATLAB-R1';
model.architecture = [ ...
    'One coupled thirteen-state nonlinear aircraft driven by the frozen ', ...
    'Mode Manager V1, longitudinal and lateral controller laws, protected ', ...
    'actuators and their accepted sensor/filter delays'];

model.required.interface = interface.artifactRevision;
model.required.integratedAutopilot = ...
    interface.required.integratedAutopilot;
model.required.modeManager = manager.artifactRevision;
model.required.nonlinearDynamics = plant.artifactRevision;
model.required.pitchController = pitch.version;
model.required.elevatorLoop = elevator.integratedLoopVersion;
model.required.altitudeController = altitude.version;
model.required.yawController = yaw.artifactRevision;
model.required.rudderAudit = rudder.artifactRevision;
model.required.rollController = roll.artifactRevision;
model.required.aileronAudit = aileron.artifactRevision;
model.required.headingSensor = headingSensor.artifactRevision;
model.required.headingController = heading.artifactRevision;
model.required.controllerRelease = interface.required.flightControlsRelease;
model.required.nonlinearRelease = interface.required.nonlinearPlantRelease;
model.required.externalRegressionPasses = ...
    interface.required.combinedProjectPasses;

model.dependencies.interface = interface;
model.dependencies.manager = manager;
model.dependencies.plant = plant;
model.dependencies.pitch = pitch;
model.dependencies.elevator = elevator;
model.dependencies.altitude = altitude;
model.dependencies.yaw = yaw;
model.dependencies.rudder = rudder;
model.dependencies.roll = roll;
model.dependencies.aileron = aileron;
model.dependencies.headingSensor = headingSensor;
model.dependencies.heading = heading;

model.trim = solveC172sNonlinearTrim([],plant.trimModel);
model.trim.throttleOwner = 'frozen nonlinear V0.1 one-point trim';

model.execution.solver = 'hybrid fixed-step RK4';
model.execution.fixedStep_s = interface.execution.fixedStep_s;
model.execution.supervisorSampleTime_s = ...
    interface.execution.supervisorSampleTime_s;
model.execution.controllerSampleTime_s = ...
    interface.execution.controllerSampleTime_s;
model.execution.normalizeQuaternionAfterStep = true;
model.execution.requireDeterministicReplay = true;

model.state.plantNames = interface.plant.stateNames;
model.state.plantCount = interface.plant.stateCount;
model.state.controllerNames = { ...
    'thetaCommandFiltered_rad'; ...
    'pitchIntegrator_rad_s'; ...
    'elevatorActual_rad'; ...
    'altitudeCommandFiltered_m'; ...
    'yawRateSensor_radps'; ...
    'yawWashoutState_radps'; ...
    'rudderActual_rad'; ...
    'rollRateSensor_radps'; ...
    'bankSensor_rad'; ...
    'rollIntegrator_rad_s'; ...
    'aileronActual_rad'; ...
    'bankCommandFiltered_rad'; ...
    'headingSensor_rad'};
model.state.controllerCount = numel(model.state.controllerNames);
model.state.totalCount = model.state.plantCount+model.state.controllerCount;
model.state.controllerInitial = zeros(model.state.controllerCount,1);

model.state.index.thetaCommandFiltered = 1;
model.state.index.pitchIntegrator = 2;
model.state.index.elevatorActual = 3;
model.state.index.altitudeCommandFiltered = 4;
model.state.index.yawRateSensor = 5;
model.state.index.yawWashout = 6;
model.state.index.rudderActual = 7;
model.state.index.rollRateSensor = 8;
model.state.index.bankSensor = 9;
model.state.index.rollIntegrator = 10;
model.state.index.aileronActual = 11;
model.state.index.bankCommandFiltered = 12;
model.state.index.headingSensor = 13;

model.delay.yawRateSamples = ...
    round(rudder.sensor.nominalDelay_s/model.execution.fixedStep_s);
model.delay.rollRateSamples = ...
    round(aileron.sensor.nominalDelay_s/model.execution.fixedStep_s);
model.delay.bankAngleSamples = ...
    round(aileron.sensor.nominalDelay_s/model.execution.fixedStep_s);
model.delay.headingSamples = ...
    round(headingSensor.nominalDelay_s/model.execution.fixedStep_s);
model.delay.implementation = ...
    'sampled plant values held in explicit deterministic delay states';

model.input.requestNames = { ...
    'engageRequest','disconnectRequest','pilotOverride','trimStable', ...
    'coreSensorsValid','headingValid','altitudeValid', ...
    'forceAileronSaturated','forceElevatorSaturated', ...
    'lateralModeRequest','verticalModeRequest', ...
    'updateHeadingReference','updateAltitudeReference', ...
    'reservedBank_deg','reservedPitch_deg','reservedHeading_deg', ...
    'reservedAltitude_ft','selectedHeading_deg','selectedAltitude_ft'};
model.input.requestCount = numel(model.input.requestNames);
model.input.liveFeedbackColumns = 14:17;
model.input.windFrame = 'NED';

model.tracking.roll = [ ...
    'On a roll tracking request, reset the filtered bank command to the ', ...
    'actual bank and solve the frozen PI law for the current aileron.'];
model.tracking.pitch = [ ...
    'On a pitch tracking request, reset the filtered pitch command to the ', ...
    'actual pitch and solve the frozen PI law for the current elevator.'];
model.tracking.changesControllerGains = false;

model.validation.minimumAuditCases = 36;
model.validation.trimDuration_s = 2.0;
model.validation.captureDuration_s = 3.0;
model.validation.outerDuration_s = 120.0;
model.validation.faultDuration_s = 15.0;
model.validation.disconnectDuration_s = 12.0;
model.validation.saturationDuration_s = 4.0;
model.validation.windDuration_s = 30.0;
model.validation.headingCommand_deg = 5.0;
model.validation.altitudeCommand_ft = 20.0;
model.validation.maximumTrimDynamicStateError = 2.0e-8;
model.validation.maximumFinalHeadingError_deg = 1.0;
model.validation.maximumFinalAltitudeError_ft = 5.0;
model.validation.maximumQuaternionNormError = 5.0e-10;
model.validation.maximumSurfaceLimitTolerance_deg = 1.0e-8;
model.validation.maximumRateLimitTolerance_degps = 1.0e-7;
model.validation.maximumReleasedSurface_deg = 1.0e-5;
model.validation.minimumDrivenRate_degps = 1.0e-5;
model.validation.maximumEnvelopeViolationSamples = 0;
model.validation.requireBitwiseReplay = true;

model.policy.modifyControllerRelease = false;
model.policy.modifyNonlinearRelease = false;
model.policy.retuneController = false;
model.policy.addAerodynamicCrossDerivatives = false;
model.policy.addAutothrottle = false;
model.policy.claimAircraftApproval = false;

model.limitations = [ ...
    'This is a near-trim research closed-loop validation using ideal ', ...
    'deterministic source signals, frozen local aerodynamic coefficients ', ...
    'and one-point thrust. It does not establish full-envelope stability, ', ...
    'pilot-in-the-loop behavior, hardware performance, stall/spin behavior ', ...
    'or aircraft approval.'];
end
