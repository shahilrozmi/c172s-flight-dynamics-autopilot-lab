function result = simulateC172sNonlinearAutopilot(scenario,model)
%SIMULATEC172SNONLINEARAUTOPILOT Hybrid nonlinear closed-loop reference.
%
% RESULT = SIMULATEC172SNONLINEARAUTOPILOT(SCENARIO,MODEL) executes the
% frozen Mode Manager and protected controller laws around the coupled
% thirteen-state nonlinear research plant. The fixed-step RK4 propagation
% is independent of Simulink and retains explicit sampled sensor delays.

if nargin < 2 || isempty(model)
    model = c172sNonlinearAutopilotClosedLoopData();
end
validateModel(model);
scenario = validateScenario(scenario,model);

t = scenario.time_s(:);
n = numel(t);
dt = model.execution.fixedStep_s;
plantState = initialPlantState(scenario,model);
controllerState = model.state.controllerInitial;
managerState = [];

initialFeedback = plantFeedback(plantState,controllerState, ...
    scenario.steadyWindNED_mps(1,:).',scenario.gustNED_mps(1,:).',model);
delay.r = initialFeedback.r_radps;
delay.p = initialFeedback.p_radps;
delay.phi = initialFeedback.phi_rad;
delay.psi = repmat(initialFeedback.psi_rad,model.delay.headingSamples,1);
internalAileronSaturated = false;
internalElevatorSaturated = false;

result.time_s = t;
result.plantState = zeros(n,model.state.plantCount);
result.controllerState = zeros(n,model.state.controllerCount);
result.control = zeros(n,4);
result.surfaceRate_radps = zeros(n,3);
result.euler_rad = zeros(n,3);
result.altitude_ft = zeros(n,1);
result.altitudePerturbation_m = zeros(n,1);
result.verticalSpeed_mps = zeros(n,1);
result.TAS_mps = zeros(n,1);
result.alpha_rad = zeros(n,1);
result.beta_rad = zeros(n,1);
result.quaternionNorm = zeros(n,1);
result.withinEnvelope = false(n,1);
result.engaged = false(n,1);
result.lateralMode = zeros(n,1,'uint8');
result.verticalMode = zeros(n,1,'uint8');
result.rollTrackRequest = false(n,1);
result.pitchTrackRequest = false(n,1);
result.eventCode = zeros(n,1,'uint8');
result.bankCommand_deg = zeros(n,1);
result.pitchCommand_deg = zeros(n,1);
result.headingCommand_deg = zeros(n,1);
result.altitudeCommand_ft = zeros(n,1);
result.aileronSaturated = false(n,1);
result.elevatorSaturated = false(n,1);
result.aileronSaturationTime_s = zeros(n,1);
result.elevatorSaturationTime_s = zeros(n,1);

for k = 1:n
    wind = scenario.steadyWindNED_mps(k,:).';
    gust = scenario.gustNED_mps(k,:).';
    feedback = plantFeedback(plantState,controllerState,wind,gust,model);
    input = managerInput(scenario.request(k,:),feedback, ...
        internalAileronSaturated,internalElevatorSaturated,model);
    [managerState,supervisor] = c172sAutopilotModeManagerStep( ...
        managerState,input,model.dependencies.manager,dt);

    controllerState = applyTracking(controllerState,feedback,supervisor,model);
    z = [plantState;controllerState];
    % Retain the derivative returned with the diagnostic calculation. This
    % is the RK4 k1 evaluation at the same state, supervisor and environment,
    % so recomputing it below only repeats the complete nonlinear plant RHS.
    [k1,diagnostic] = closedLoopDerivative( ...
        z,supervisor,delay,wind,gust,model);

    result.plantState(k,:) = plantState.';
    result.controllerState(k,:) = controllerState.';
    result.control(k,:) = diagnostic.control.';
    result.surfaceRate_radps(k,:) = diagnostic.surfaceRate_radps.';
    result.euler_rad(k,:) = [diagnostic.feedback.phi_rad, ...
        diagnostic.feedback.theta_rad,diagnostic.feedback.psi_rad];
    result.altitude_ft(k) = diagnostic.feedback.altitude_ft;
    result.altitudePerturbation_m(k) = ...
        diagnostic.feedback.altitudePerturbation_m;
    result.verticalSpeed_mps(k) = diagnostic.feedback.verticalSpeed_mps;
    result.TAS_mps(k) = diagnostic.feedback.TAS_mps;
    result.alpha_rad(k) = diagnostic.feedback.alpha_rad;
    result.beta_rad(k) = diagnostic.feedback.beta_rad;
    result.quaternionNorm(k) = diagnostic.feedback.quaternionNorm;
    result.withinEnvelope(k) = diagnostic.feedback.withinEnvelope;
    result.engaged(k) = supervisor.engaged;
    result.lateralMode(k) = supervisor.lateralMode;
    result.verticalMode(k) = supervisor.verticalMode;
    result.rollTrackRequest(k) = supervisor.rollIntegratorTrackRequest;
    result.pitchTrackRequest(k) = supervisor.pitchIntegratorTrackRequest;
    result.eventCode(k) = supervisor.eventCode;
    result.bankCommand_deg(k) = supervisor.bankCommand_deg;
    result.pitchCommand_deg(k) = supervisor.pitchCommand_deg;
    result.headingCommand_deg(k) = supervisor.headingCommand_deg;
    result.altitudeCommand_ft(k) = supervisor.altitudeCommand_ft;
    result.aileronSaturated(k) = diagnostic.aileronSaturated;
    result.elevatorSaturated(k) = diagnostic.elevatorSaturated;
    result.aileronSaturationTime_s(k) = ...
        managerState.aileronSaturationTime_s;
    result.elevatorSaturationTime_s(k) = ...
        managerState.elevatorSaturationTime_s;

    internalAileronSaturated = diagnostic.aileronSaturated;
    internalElevatorSaturated = diagnostic.elevatorSaturated;

    if k < n
        sampled.r = feedback.r_radps;
        sampled.p = feedback.p_radps;
        sampled.phi = feedback.phi_rad;
        sampled.psi = feedback.psi_rad;
        k2 = closedLoopDerivative(z+0.5*dt*k1,supervisor,delay, ...
            wind,gust,model);
        k3 = closedLoopDerivative(z+0.5*dt*k2,supervisor,delay, ...
            wind,gust,model);
        k4 = closedLoopDerivative(z+dt*k3,supervisor,delay,wind,gust,model);
        z = z+(dt/6)*(k1+2*k2+2*k3+k4);
        z(1:13) = normalizePlantQuaternion(z(1:13));
        plantState = z(1:13);
        controllerState = z(14:end);
        delay.r = sampled.r;
        delay.p = sampled.p;
        delay.phi = sampled.phi;
        delay.psi = [delay.psi(2:end);sampled.psi];
    end
end

result.request = scenario.request;
result.steadyWindNED_mps = scenario.steadyWindNED_mps;
result.gustNED_mps = scenario.gustNED_mps;
result.finalManagerState = managerState;
result.modelArtifactRevision = model.artifactRevision;
result.scenarioName = scenario.name;
result.executionBackend = 'MATLAB';
end

function [zDot,d] = closedLoopDerivative( ...
    z,supervisor,delay,wind,gust,model)
x = z(1:13);
c = z(14:end);
I = model.state.index;
dep = model.dependencies;

control = [c(I.elevatorActual);c(I.aileronActual); ...
    c(I.rudderActual);model.trim.controls(4)];
[xDot,plantOutput] = c172sIntegratedNonlinearDynamics( ...
    x,control,wind,gust,dep.plant);
feedback = feedbackFromPlant(x,xDot,plantOutput,model);

d = controllerSignals(c,feedback,supervisor,delay,model);
cDot = zeros(model.state.controllerCount,1);
cDot(I.thetaCommandFiltered) = ...
    (d.thetaCommandSelected-c(I.thetaCommandFiltered))/ ...
    dep.pitch.commandFilterTimeConstant_s;
cDot(I.pitchIntegrator) = d.pitchError+d.pitchAntiWindup;
cDot(I.elevatorActual) = d.elevatorRate;
cDot(I.altitudeCommandFiltered) = ...
    (d.altitudeCommand_m-c(I.altitudeCommandFiltered))/ ...
    dep.altitude.commandFilterTimeConstant_s;
cDot(I.yawRateSensor) = ...
    (d.yawRateInput-c(I.yawRateSensor))/ ...
    dep.rudder.sensor.filterTimeConstant_s;
cDot(I.yawWashout) = ...
    (c(I.yawRateSensor)-c(I.yawWashout))/ ...
    dep.yaw.washoutTimeConstant_s;
cDot(I.rudderActual) = d.rudderRate;
cDot(I.rollRateSensor) = ...
    (d.rollRateInput-c(I.rollRateSensor))/ ...
    dep.aileron.sensor.filterTimeConstant_s;
cDot(I.bankSensor) = ...
    (d.bankInput-c(I.bankSensor))/ ...
    dep.aileron.sensor.filterTimeConstant_s;
cDot(I.rollIntegrator) = d.bankError+d.rollAntiWindup;
cDot(I.aileronActual) = d.aileronRate;
cDot(I.bankCommandFiltered) = ...
    (d.bankCommandSelected-c(I.bankCommandFiltered))/ ...
    dep.roll.commandFilterTimeConstant_s;
cDot(I.headingSensor) = ...
    (d.headingInput-c(I.headingSensor))/ ...
    dep.headingSensor.filterTimeConstant_s;

zDot = [xDot;cDot];
d.control = control;
d.surfaceRate_radps = [d.elevatorRate;d.aileronRate;d.rudderRate];
d.feedback = feedback;
end

function d = controllerSignals(c,feedback,supervisor,delay,model)
I = model.state.index;
dep = model.dependencies;

d.altitudeCommand_m = ...
    (supervisor.altitudeCommand_ft-model.dependencies.interface.trim.altitude_ft)* ...
    model.dependencies.interface.conversions.ftToM;
d.verticalSpeedCommandRaw = dep.altitude.altitudeGain_per_s* ...
    (c(I.altitudeCommandFiltered)-feedback.altitudePerturbation_m);
d.verticalSpeedCommand = clip(d.verticalSpeedCommandRaw, ...
    -dep.altitude.verticalSpeedCommandLimit_mps, ...
    dep.altitude.verticalSpeedCommandLimit_mps);
d.thetaCommandOuterRaw = dep.altitude.verticalSpeedGain_radPerMps* ...
    (d.verticalSpeedCommand-feedback.verticalSpeed_mps);
d.thetaCommandOuter = clip(d.thetaCommandOuterRaw, ...
    -dep.altitude.pitchCommandLimit_rad,dep.altitude.pitchCommandLimit_rad);
if supervisor.altitudeModeEnable
    d.thetaCommandSelected = d.thetaCommandOuter;
else
    d.thetaCommandSelected = deg2rad(supervisor.pitchCommand_deg);
end

d.pitchError = c(I.thetaCommandFiltered)-feedback.theta_rad;
d.elevatorUnsaturated = ...
    -dep.pitch.Kp_radPerRad*d.pitchError- ...
    dep.pitch.Ki_radPerRadPerS*c(I.pitchIntegrator)+ ...
    dep.elevator.integration.Kq_s*feedback.q_radps;
d.elevatorPositionLimited = clip(d.elevatorUnsaturated, ...
    dep.elevator.authority.lowerLimit_rad, ...
    dep.elevator.authority.upperLimit_rad);
d.elevatorTarget = double(supervisor.engaged)*d.elevatorPositionLimited;
d.elevatorRateRaw = ...
    (d.elevatorTarget-c(I.elevatorActual))/ ...
    dep.elevator.dynamics.timeConstant_s;
d.elevatorRate = clip(d.elevatorRateRaw, ...
    -deg2rad(dep.elevator.dynamics.rateLimit_degps), ...
    deg2rad(dep.elevator.dynamics.rateLimit_degps));
d.elevatorTrack = c(I.elevatorActual)+ ...
    dep.elevator.dynamics.timeConstant_s*d.elevatorRate;
d.pitchAntiWindup = ...
    (d.elevatorUnsaturated-d.elevatorTrack)/( ...
    dep.pitch.Ki_radPerRadPerS* ...
    dep.elevator.antiWindup.trackingTimeConstant_s);

d.headingInput = delay.psi(1);
d.headingError = wrapAngle( ...
    deg2rad(supervisor.headingCommand_deg)-c(I.headingSensor));
d.bankCommandOuterRaw = ...
    dep.heading.headingErrorGain*d.headingError;
bankLimit = deg2rad(dep.heading.bankCommandAuthority_deg);
d.bankCommandOuter = clip(d.bankCommandOuterRaw,-bankLimit,bankLimit);
if supervisor.headingModeEnable
    d.bankCommandSelected = d.bankCommandOuter;
else
    d.bankCommandSelected = deg2rad(supervisor.bankCommand_deg);
end

d.yawRateInput = delay.r;
d.rollRateInput = delay.p;
d.bankInput = delay.phi;
d.yawRateWashout = c(I.yawRateSensor)-c(I.yawWashout);
d.rudderUnsaturated = -dep.yaw.rudderGain_s*d.yawRateWashout;
d.rudderPositionLimited = clip(d.rudderUnsaturated, ...
    dep.rudder.authority.lowerLimit_rad, ...
    dep.rudder.authority.upperLimit_rad);
d.rudderTarget = double(supervisor.engaged)*d.rudderPositionLimited;
d.rudderRateRaw = (d.rudderTarget-c(I.rudderActual))/ ...
    dep.rudder.dynamics.timeConstant_s;
d.rudderRate = clip(d.rudderRateRaw, ...
    -deg2rad(dep.rudder.dynamics.rateLimit_degps), ...
    deg2rad(dep.rudder.dynamics.rateLimit_degps));

d.bankError = c(I.bankCommandFiltered)-c(I.bankSensor);
d.rollProportional = dep.roll.bankProportionalGain*d.bankError;
d.rollIntegral = dep.roll.bankIntegralGain_per_s*c(I.rollIntegrator);
d.rollRateContribution = ...
    -dep.roll.rollRateGain_s*c(I.rollRateSensor);
d.aileronUnsaturated = d.rollProportional+d.rollIntegral+ ...
    d.rollRateContribution;
d.aileronPositionLimited = clip(d.aileronUnsaturated, ...
    dep.aileron.authority.lowerLimit_rad, ...
    dep.aileron.authority.upperLimit_rad);
d.aileronTarget = double(supervisor.engaged)*d.aileronPositionLimited;
d.aileronRateRaw = (d.aileronTarget-c(I.aileronActual))/ ...
    dep.aileron.dynamics.timeConstant_s;
d.aileronRate = clip(d.aileronRateRaw, ...
    -deg2rad(dep.aileron.dynamics.rateLimit_degps), ...
    deg2rad(dep.aileron.dynamics.rateLimit_degps));
d.aileronTrack = c(I.aileronActual)+ ...
    dep.aileron.dynamics.timeConstant_s*d.aileronRate;
d.rollAntiWindup = (d.aileronTrack-d.aileronUnsaturated)/( ...
    dep.roll.bankIntegralGain_per_s* ...
    dep.aileron.antiWindup.trackingTimeConstant_s);

tol = 1.0e-12;
d.elevatorSaturated = supervisor.engaged && ( ...
    abs(d.elevatorUnsaturated-d.elevatorPositionLimited) > tol || ...
    abs(d.elevatorRateRaw-d.elevatorRate) > tol);
d.aileronSaturated = supervisor.engaged && ( ...
    abs(d.aileronUnsaturated-d.aileronPositionLimited) > tol || ...
    abs(d.aileronRateRaw-d.aileronRate) > tol);
end

function controller = applyTracking(controller,feedback,supervisor,model)
I = model.state.index;
dep = model.dependencies;
if supervisor.pitchIntegratorTrackRequest
    controller(I.thetaCommandFiltered) = feedback.theta_rad;
    error = controller(I.thetaCommandFiltered)-feedback.theta_rad;
    controller(I.pitchIntegrator) = ( ...
        -dep.pitch.Kp_radPerRad*error+ ...
        dep.elevator.integration.Kq_s*feedback.q_radps- ...
        controller(I.elevatorActual))/dep.pitch.Ki_radPerRadPerS;
end
if supervisor.rollIntegratorTrackRequest
    controller(I.bankCommandFiltered) = feedback.phi_rad;
    error = controller(I.bankCommandFiltered)-controller(I.bankSensor);
    controller(I.rollIntegrator) = ( ...
        controller(I.aileronActual)- ...
        dep.roll.bankProportionalGain*error+ ...
        dep.roll.rollRateGain_s*controller(I.rollRateSensor))/ ...
        dep.roll.bankIntegralGain_per_s;
end
end

function feedback = plantFeedback(x,c,wind,gust,model)
I = model.state.index;
control = [c(I.elevatorActual);c(I.aileronActual); ...
    c(I.rudderActual);model.trim.controls(4)];
[xDot,plantOutput] = c172sIntegratedNonlinearDynamics( ...
    x,control,wind,gust,model.dependencies.plant);
feedback = feedbackFromPlant(x,xDot,plantOutput,model);
end

function feedback = feedbackFromPlant(x,xDot,plantOutput,model)
q = x(10:13);
qNorm = norm(q);
q = q/qNorm;
[phi,theta,psiRaw] = quaternionToEuler(q);
psi = mod(psiRaw,2*pi);
altitude_m = -x(9);
air = plantOutput.totalForces.aerodynamics.airData;
interface = model.dependencies.interface;

feedback.p_radps = x(4);
feedback.q_radps = x(5);
feedback.r_radps = x(6);
feedback.phi_rad = phi;
feedback.theta_rad = theta;
feedback.psi_rad = psi;
feedback.bank_deg = rad2deg(phi);
feedback.pitch_deg = rad2deg(theta);
feedback.heading_deg = mod(rad2deg(psi),360);
feedback.altitude_ft = altitude_m*interface.conversions.mToFt;
feedback.altitudePerturbation_m = altitude_m-interface.trim.altitude_m;
feedback.verticalSpeed_mps = -xDot(9);
feedback.TAS_mps = air.TAS_mps;
feedback.alpha_rad = air.alpha_rad;
feedback.beta_rad = air.beta_rad;
feedback.quaternionNorm = qNorm;
feedback.coreValid = all(isfinite(x(4:6))) && isfinite(qNorm) && ...
    abs(qNorm-1) <= interface.validity.maximumQuaternionNormError;
feedback.headingValid = feedback.coreValid && isfinite(psi);
feedback.altitudeValid = isfinite(altitude_m);
feedback.airDataValid = air.valid;
feedback.withinEnvelope = air.valid && ...
    air.TAS_mps >= interface.envelope.V_KTAS(1)*0.514444444444444 && ...
    air.TAS_mps <= interface.envelope.V_KTAS(2)*0.514444444444444 && ...
    air.alpha_rad >= deg2rad(interface.envelope.alpha_deg(1)) && ...
    air.alpha_rad <= deg2rad(interface.envelope.alpha_deg(2)) && ...
    air.beta_rad >= deg2rad(interface.envelope.beta_deg(1)) && ...
    air.beta_rad <= deg2rad(interface.envelope.beta_deg(2)) && ...
    phi >= deg2rad(interface.envelope.bank_deg(1)) && ...
    phi <= deg2rad(interface.envelope.bank_deg(2)) && ...
    theta >= deg2rad(interface.envelope.pitch_deg(1)) && ...
    theta <= deg2rad(interface.envelope.pitch_deg(2));
end

function input = managerInput(row,feedback,internalAileronSaturated, ...
    internalElevatorSaturated,model)
manager = model.dependencies.manager;
input.engageRequest = logical(row(1));
input.disconnectRequest = logical(row(2));
input.pilotOverride = logical(row(3));
input.trimStable = logical(row(4));
input.coreSensorsValid = logical(row(5)) && feedback.coreValid;
input.headingValid = logical(row(6)) && feedback.headingValid;
input.altitudeValid = logical(row(7)) && feedback.altitudeValid;
input.aileronSaturated = logical(row(8)) || internalAileronSaturated;
input.elevatorSaturated = logical(row(9)) || internalElevatorSaturated;
input.lateralModeRequest = uint8(row(10));
input.verticalModeRequest = uint8(row(11));
input.updateHeadingReference = logical(row(12));
input.updateAltitudeReference = logical(row(13));
input.bank_deg = feedback.bank_deg;
input.pitch_deg = feedback.pitch_deg;
input.heading_deg = feedback.heading_deg;
input.altitude_ft = feedback.altitude_ft;
input.selectedHeading_deg = row(18);
input.selectedAltitude_ft = row(19);
validLateral = input.lateralModeRequest == manager.codes.lateral.OFF || ...
    input.lateralModeRequest == manager.codes.lateral.ROLL || ...
    input.lateralModeRequest == manager.codes.lateral.HEADING;
validVertical = input.verticalModeRequest == manager.codes.vertical.OFF || ...
    input.verticalModeRequest == manager.codes.vertical.PITCH || ...
    input.verticalModeRequest == manager.codes.vertical.ALTITUDE;
if ~(validLateral && validVertical)
    error('simulateC172sNonlinearAutopilot:InvalidModeRequest', ...
        'A scenario contains an undefined mode request.');
end
end

function scenario = validateScenario(scenario,model)
if ~isstruct(scenario) || ~isfield(scenario,'time_s') || ...
        ~isfield(scenario,'request')
    error('simulateC172sNonlinearAutopilot:InvalidScenario', ...
        'Scenario must contain time_s and request.');
end
t = scenario.time_s(:);
if ~(isnumeric(t) && isreal(t) && all(isfinite(t)) && numel(t) >= 2 && ...
        abs(t(1)) <= eps && all(diff(t) > 0))
    error('simulateC172sNonlinearAutopilot:InvalidTime', ...
        'Scenario time must be a finite increasing column beginning at zero.');
end
dtError = max(abs(diff(t)-model.execution.fixedStep_s));
if dtError > 1.0e-12
    error('simulateC172sNonlinearAutopilot:InvalidTimeStep', ...
        'Scenario must use the frozen %.6f-s sample time.', ...
        model.execution.fixedStep_s);
end
if ~(isnumeric(scenario.request) && isreal(scenario.request) && ...
        all(isfinite(scenario.request),'all') && ...
        isequal(size(scenario.request), ...
        [numel(t),model.input.requestCount]))
    error('simulateC172sNonlinearAutopilot:InvalidRequest', ...
        'Request must be a finite N-by-%d matrix.', ...
        model.input.requestCount);
end
scenario.time_s = t;
scenario.steadyWindNED_mps = environmentMatrix( ...
    scenario,'steadyWindNED_mps',numel(t));
scenario.gustNED_mps = environmentMatrix( ...
    scenario,'gustNED_mps',numel(t));
if ~isfield(scenario,'name')
    scenario.name = 'unnamed nonlinear-autopilot scenario';
end
end

function value = environmentMatrix(scenario,field,n)
if ~isfield(scenario,field)
    value = zeros(n,3);
    return;
end
value = scenario.(field);
if isequal(size(value),[1,3])
    value = repmat(value,n,1);
end
if ~(isnumeric(value) && isreal(value) && all(isfinite(value),'all') && ...
        isequal(size(value),[n,3]))
    error('simulateC172sNonlinearAutopilot:InvalidEnvironment', ...
        '%s must be a finite 1-by-3 or N-by-3 NED matrix.',field);
end
end

function state = initialPlantState(scenario,model)
if isfield(scenario,'initialPlantState')
    state = scenario.initialPlantState;
    if ~(isnumeric(state) && isreal(state) && all(isfinite(state)) && ...
            isequal(size(state),[13,1]))
        error('simulateC172sNonlinearAutopilot:InvalidInitialState', ...
            'initialPlantState must be a finite real 13-by-1 column.');
    end
else
    state = model.trim.state;
end
state = normalizePlantQuaternion(state);
end

function state = normalizePlantQuaternion(state)
qNorm = norm(state(10:13));
if ~(isfinite(qNorm) && qNorm > eps)
    error('simulateC172sNonlinearAutopilot:InvalidQuaternion', ...
        'The propagated quaternion has zero or invalid norm.');
end
state(10:13) = state(10:13)/qNorm;
end

function validateModel(model)
if ~(isstruct(model) && isfield(model,'artifactRevision') && ...
        strcmp(model.artifactRevision, ...
        'NONLINEAR-AUTOPILOT-V1-CLOSED-LOOP-MATLAB-R1') && ...
        model.state.totalCount == 26)
    error('simulateC172sNonlinearAutopilot:InvalidModel', ...
        'The frozen nonlinear-autopilot closed-loop model is invalid.');
end
end

function [phi,theta,psi] = quaternionToEuler(q)
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
phi = atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
argument = 2*(q0*q2-q3*q1);
argument = min(max(argument,-1),1);
theta = asin(argument);
psi = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
end

function value = wrapAngle(value)
value = mod(value+pi,2*pi)-pi;
end

function value = clip(value,lower,upper)
value = min(max(value,lower),upper);
end
