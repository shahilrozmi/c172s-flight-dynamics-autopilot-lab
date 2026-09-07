function results = runNonlinearAutopilotSimulinkCheck()
%RUNNONLINEARAUTOPILOTSIMULINKCHECK Executable closed-loop equivalence.

clc;
close all;
load_system('simulink');
model = c172sNonlinearAutopilotClosedLoopData();
manager = model.dependencies.manager;
modelName = buildC172sNonlinearAutopilotSimulink(true);
load_system(modelName);

blockPath = [modelName '/Independent Executable Nonlinear Autopilot'];
root = sfroot;
chart = root.find('-isa','Stateflow.EMChart','Path',blockPath);
assert(~isempty(chart),'Executable nonlinear-autopilot chart was not found.');
source = chart.Script;
forbidden = { ...
    'simulateC172sNonlinearAutopilot', ...
    'c172sIntegratedNonlinearDynamics', ...
    'c172sTrimTotalForces', ...
    'c172sRigidBody6DofEom', ...
    'c172sAtmosphereAirData', ...
    'c172sAutopilotModeManagerStep'};
for k = 1:numel(forbidden)
    assert(~contains(source,forbidden{k}), ...
        'Executable block calls MATLAB reference function %s.',forbidden{k});
end
assert(contains(source,'persistent initialized z'), ...
    'Executable block no longer owns its propagated state.');
assert(strcmp(get_param(modelName,'Solver'),'ode4') && ...
    strcmp(get_param(modelName,'SolverType'),'Fixed-step'), ...
    'Generated nonlinear-autopilot solver changed.');
assert(abs(str2double(get_param(modelName,'FixedStep'))- ...
    model.execution.fixedStep_s) < eps, ...
    'Generated nonlinear-autopilot fixed step changed.');
configuration = get_param(blockPath,'MATLABFunctionConfiguration');
assert(strcmp(configuration.UpdateMethod,'Discrete') && ...
    abs(str2double(configuration.SampleTime)-0.01) < eps, ...
    'Executable hybrid block sample time changed.');

keys = { ...
    'trim'; ...
    'capture'; ...
    'outer'; ...
    'headingFallback'; ...
    'altitudeFallback'; ...
    'disconnect'; ...
    'override'; ...
    'aileronSaturation'; ...
    'elevatorSaturation'; ...
    'wind'};
captures = cell(size(keys));
for k = 1:numel(keys)
    captures{k} = runCapture(modelName,keys{k},model);
end
replay = runCapture(modelName,'outer',model);

I = indices();
tolerance = 3.0e-7;
pass = false(numel(keys)+1,1);
detail = strings(numel(pass),1);
for k = 1:numel(keys)
    pass(k) = captures{k}.maximumError <= tolerance;
    detail(k) = sprintf('maximum 58-signal error %.9g', ...
        captures{k}.maximumError);
end

trim = captures{1}.simulated;
pass(1) = pass(1) && all(trim(:,I.engaged) == 0) && ...
    max(abs(trim(:,I.control(1:3))),[],'all') < 1.0e-12;
detail(1) = sprintf('OFF/OFF trim; maximum error %.9g', ...
    captures{1}.maximumError);

capture = captures{2}.simulated;
pass(2) = pass(2) && all(capture(:,I.engaged) == 1) && ...
    capture(end,I.lateralMode) == manager.codes.lateral.ROLL && ...
    capture(end,I.verticalMode) == manager.codes.vertical.PITCH && ...
    sum(capture(:,I.rollTrack)) == 1 && ...
    sum(capture(:,I.pitchTrack)) == 1;
detail(2) = sprintf('ROLL/PITCH tracking; maximum error %.9g', ...
    captures{2}.maximumError);

outer = captures{3};
headingError = abs(rad2deg(wrapAngle( ...
    deg2rad(model.validation.headingCommand_deg)- ...
    outer.simulated(end,I.euler(3)))));
altitudeTarget = model.dependencies.interface.trim.altitude_ft+ ...
    model.validation.altitudeCommand_ft;
altitudeError = abs(altitudeTarget-outer.simulated(end,I.altitude));
pass(3) = pass(3) && all(outer.simulated(:,I.engaged) == 1) && ...
    headingError <= model.validation.maximumFinalHeadingError_deg && ...
    altitudeError <= model.validation.maximumFinalAltitudeError_ft && ...
    all(outer.simulated(:,I.withinEnvelope) == 1);
detail(3) = sprintf(['HEADING/ALTITUDE errors %.6f deg/%.6f ft; ', ...
    'maximum trace error %.9g'],headingError,altitudeError, ...
    outer.maximumError);

y = captures{4}.simulated;
pass(4) = pass(4) && y(end,I.engaged) == 1 && ...
    y(end,I.lateralMode) == manager.codes.lateral.ROLL && ...
    y(end,I.verticalMode) == manager.codes.vertical.ALTITUDE && ...
    any(y(:,I.eventCode) == manager.codes.event.HEADING_FALLBACK);
detail(4) = sprintf('HEADING->ROLL; maximum error %.9g', ...
    captures{4}.maximumError);

y = captures{5}.simulated;
pass(5) = pass(5) && y(end,I.engaged) == 1 && ...
    y(end,I.lateralMode) == manager.codes.lateral.HEADING && ...
    y(end,I.verticalMode) == manager.codes.vertical.PITCH && ...
    any(y(:,I.eventCode) == manager.codes.event.ALTITUDE_FALLBACK);
detail(5) = sprintf('ALTITUDE->PITCH; maximum error %.9g', ...
    captures{5}.maximumError);

y = captures{6}.simulated;
releaseResidual = rad2deg(max(abs(y(end,I.control(1:3)))));
pass(6) = pass(6) && y(end,I.engaged) == 0 && ...
    any(y(:,I.eventCode) == manager.codes.event.DISCONNECT_COMMAND) && ...
    releaseResidual <= model.validation.maximumReleasedSurface_deg;
detail(6) = sprintf('release %.3g deg; maximum error %.9g', ...
    releaseResidual,captures{6}.maximumError);

y = captures{7}.simulated;
pass(7) = pass(7) && y(end,I.engaged) == 0 && ...
    any(y(:,I.eventCode) == manager.codes.event.DISCONNECT_OVERRIDE);
detail(7) = sprintf('global override; maximum error %.9g', ...
    captures{7}.maximumError);

y = captures{8}.simulated;
pass(8) = pass(8) && y(end,I.engaged) == 0 && ...
    any(y(:,I.eventCode) == manager.codes.event.DISCONNECT_SATURATION) && ...
    max(y(:,I.aileronTimer)) >= ...
    manager.protection.sustainedSaturationDisconnectDelay_s-0.01;
detail(8) = sprintf('aileron protection; maximum error %.9g', ...
    captures{8}.maximumError);

y = captures{9}.simulated;
pass(9) = pass(9) && y(end,I.engaged) == 0 && ...
    any(y(:,I.eventCode) == manager.codes.event.DISCONNECT_SATURATION) && ...
    max(y(:,I.elevatorTimer)) >= ...
    manager.protection.sustainedSaturationDisconnectDelay_s-0.01;
detail(9) = sprintf('elevator protection; maximum error %.9g', ...
    captures{9}.maximumError);

y = captures{10}.simulated;
windFinite = all(isfinite(y),'all');
pass(10) = pass(10) && all(y(:,I.engaged) == 1) && ...
    all(y(:,I.withinEnvelope) == 1) && windFinite && ...
    max(y(:,I.beta)) > 0 && max(y(:,I.alpha)) > 0;
detail(10) = sprintf('wind/gust inside V0.1; maximum error %.9g', ...
    captures{10}.maximumError);

pass(11) = isequal(outer.time,replay.time) && ...
    isequal(outer.simulated,replay.simulated);
detail(11) = 'outer-mode executable trace is bitwise identical on replay';

allQuaternionError = 0;
allSurfaceRate = zeros(1,3);
for k = 1:numel(captures)
    allQuaternionError = max(allQuaternionError, ...
        max(abs(captures{k}.simulated(:,I.quaternionNorm)-1)));
    allSurfaceRate = max(allSurfaceRate, ...
        max(abs(rad2deg(captures{k}.simulated(:,I.surfaceRate))),[],1));
end
assert(allQuaternionError <= model.validation.maximumQuaternionNormError, ...
    'Executable quaternion norm error %.9g exceeded its gate.', ...
    allQuaternionError);
assert(allSurfaceRate(1) <= 20+1.0e-7 && ...
    allSurfaceRate(2) <= 30+1.0e-7 && ...
    allSurfaceRate(3) <= 20+1.0e-7, ...
    'An executable actuator-rate limit was exceeded.');

fprintf('============================================================\n');
fprintf(' C172S NONLINEAR AUTOPILOT V1 - EXECUTABLE SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('%-39s: %s\n','Model',modelName);
fprintf('%-39s: %s\n','Artifact',model.artifactRevision);
fprintf('%-39s: %s / %.4f s\n','Solver / fixed step', ...
    model.dependencies.interface.execution.solver, ...
    model.execution.fixedStep_s);
fprintf('%-39s: 58 = 26 state + 32 diagnostic\n\n', ...
    'Verification signals');
labels = { ...
    'disengaged nonlinear trim'; ...
    'ROLL plus PITCH capture'; ...
    'simultaneous HEADING plus ALTITUDE'; ...
    'confirmed heading-source fallback'; ...
    'confirmed altitude-source fallback'; ...
    'pilot disconnect and surface release'; ...
    'pilot override priority'; ...
    'sustained aileron protection'; ...
    'sustained elevator protection'; ...
    'steady wind plus vertical gust'; ...
    'deterministic executable replay'};
for k = 1:numel(pass)
    tag = 'PASS'; if ~pass(k), tag = 'FAIL'; end
    fprintf('[%s] %-41s %s\n',tag,labels{k},detail(k));
end
fprintf('\nPassed executable scenarios            : %d/%d\n', ...
    sum(pass),numel(pass));
fprintf('Maximum 58-signal equivalence error    : %.9g\n', ...
    max(cellfun(@(capture)capture.maximumError,captures)));
fprintf('Final heading / altitude error         : %.6f deg / %.6f ft\n', ...
    headingError,altitudeError);
fprintf('Maximum quaternion norm error          : %.9g\n', ...
    allQuaternionError);

assert(all(pass), ...
    'runNonlinearAutopilotSimulinkCheck:ExecutableGateFailed', ...
    '%d of %d nonlinear-autopilot executable scenarios failed.', ...
    sum(~pass),numel(pass));
fprintf('PASS: Simulink reproduces the independent 26-state MATLAB reference.\n');
fprintf('PASS: one nonlinear aircraft executes under the frozen controller stack.\n');
fprintf('PASS: modes, protection, wind/gust and deterministic replay pass.\n');
fprintf(['NOTE: near-trim coefficients, ideal sensors and one-point throttle ', ...
    'remain; this is not aircraft approval.\n']);

plotEvidence(outer,captures{10},model,altitudeTarget);

results = repmat(struct('Name','','MaximumError',0),numel(keys),1);
for k = 1:numel(keys)
    results(k).Name = labels{k};
    results(k).MaximumError = captures{k}.maximumError;
end

try
    close_system(modelName,0);
    load_system([modelName '.slx']);
    open_system(modelName);
    fprintf('Reloaded generated model to clear transient editor diagnostics.\n');
catch
end
end

function capture = runCapture(modelName,key,model)
scenario = makeScenario(key,model);
inputSignal = [scenario.time_s scenario.request ...
    scenario.steadyWindNED_mps scenario.gustNED_mps];
simInput = Simulink.SimulationInput(modelName);
simInput = simInput.setVariable('na1InputSignal',inputSignal);
simInput = simInput.setModelParameter( ...
    'StopTime',num2str(scenario.time_s(end),17));
simOut = sim(simInput);
[time,simulated] = extractArrayOutput(simOut,58);
referenceResult = simulateC172sNonlinearAutopilot(scenario,model);
reference = c172sNonlinearAutopilotResultMatrix(referenceResult);
assert(numel(time) == numel(scenario.time_s) && ...
    max(abs(time-scenario.time_s)) < 1.0e-10, ...
    'Executable sample grid changed in %s.',key);
assert(isequal(size(simulated),size(reference)), ...
    'Executable verification interface changed in %s.',key);
capture.key = key;
capture.time = time;
capture.simulated = simulated;
capture.reference = reference;
capture.maximumError = max(abs(simulated-reference),[],'all');
end

function scenario = makeScenario(key,model)
dt = model.execution.fixedStep_s;
switch key
    case 'trim'
        stopTime = model.validation.trimDuration_s;
    case 'capture'
        stopTime = model.validation.captureDuration_s;
    case 'outer'
        stopTime = model.validation.outerDuration_s;
    case {'headingFallback','altitudeFallback'}
        stopTime = model.validation.faultDuration_s;
    case {'disconnect','override'}
        stopTime = model.validation.disconnectDuration_s;
    case {'aileronSaturation','elevatorSaturation'}
        stopTime = model.validation.saturationDuration_s;
    case 'wind'
        stopTime = model.validation.windDuration_s;
    otherwise
        error('runNonlinearAutopilotSimulinkCheck:UnknownScenario', ...
            'Unknown scenario %s.',key);
end
time = (0:dt:stopTime).';
request = zeros(numel(time),model.input.requestCount);
request(:,4:7) = 1;
request(:,18) = model.dependencies.interface.initialization.heading_deg;
request(:,19) = model.dependencies.interface.trim.altitude_ft;
wind = zeros(numel(time),3);
gust = zeros(numel(time),3);

switch key
    case 'trim'
    case 'capture'
        request(1,1) = 1;
    case {'outer','headingFallback','altitudeFallback','disconnect','override'}
        request(1,1) = 1;
        request(1,10) = double(model.dependencies.manager.codes.lateral.HEADING);
        request(1,11) = double(model.dependencies.manager.codes.vertical.ALTITUDE);
        request(:,18) = model.validation.headingCommand_deg;
        request(:,19) = model.dependencies.interface.trim.altitude_ft+ ...
            model.validation.altitudeCommand_ft;
        if strcmp(key,'headingFallback')
            request(time >= 8,6) = 0;
        elseif strcmp(key,'altitudeFallback')
            request(time >= 8,7) = 0;
        elseif strcmp(key,'disconnect')
            request(nearestIndex(time,8),2) = 1;
        elseif strcmp(key,'override')
            request(nearestIndex(time,8),3) = 1;
        end
    case 'aileronSaturation'
        request(1,1) = 1;
        request(time >= 1,8) = 1;
    case 'elevatorSaturation'
        request(1,1) = 1;
        request(time >= 1,9) = 1;
    case 'wind'
        request(1,1) = 1;
        wind(time >= 5,2) = -2.0;
        gust(time >= 8 & time <= 10,3) = -0.5;
end

scenario.name = key;
scenario.time_s = time;
scenario.request = request;
scenario.steadyWindNED_mps = wind;
scenario.gustNED_mps = gust;
end

function index = nearestIndex(time,value)
[~,index] = min(abs(time-value));
end

function [time,output] = extractArrayOutput(simOut,nSignal)
time = simOut.tout(:);
output = simOut.yout;
if isa(output,'timeseries')
    time = output.Time(:);
    output = output.Data;
end
if isstruct(output) && isfield(output,'signals')
    output = output.signals.values;
end
output = squeeze(output);
if size(output,1) ~= numel(time) && size(output,2) == numel(time)
    output = output.';
end
assert(size(output,1) == numel(time) && size(output,2) == nSignal, ...
    'Expected %d output signals; received %s.', ...
    nSignal,mat2str(size(output)));
end

function I = indices()
I.state = 1:26;
I.control = 27:30;
I.surfaceRate = 31:33;
I.euler = 34:36;
I.altitude = 37;
I.altitudePerturbation = 38;
I.verticalSpeed = 39;
I.TAS = 40;
I.alpha = 41;
I.beta = 42;
I.quaternionNorm = 43;
I.withinEnvelope = 44;
I.engaged = 45;
I.lateralMode = 46;
I.verticalMode = 47;
I.rollTrack = 48;
I.pitchTrack = 49;
I.eventCode = 50;
I.bankCommand = 51;
I.pitchCommand = 52;
I.headingCommand = 53;
I.altitudeCommand = 54;
I.aileronSaturated = 55;
I.elevatorSaturated = 56;
I.aileronTimer = 57;
I.elevatorTimer = 58;
end

function value = wrapAngle(value)
value = mod(value+pi,2*pi)-pi;
end

function plotEvidence(outer,wind,model,altitudeTarget)
I = indices();
figure('Name','C172S Nonlinear Autopilot V1 - executable evidence', ...
    'Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
plot(outer.time,rad2deg(outer.simulated(:,I.euler(3))),'LineWidth',1.3);
hold on;
plot(outer.time,rad2deg(outer.reference(:,I.euler(3))),'--','LineWidth',1.0);
yline(model.validation.headingCommand_deg,':');
grid on; xlabel('Time (s)'); ylabel('Heading (deg)'); title('Heading');
legend('Simulink','Independent MATLAB','Command','Location','best');
nexttile;
plot(outer.time,outer.simulated(:,I.altitude),'LineWidth',1.3);
hold on;
plot(outer.time,outer.reference(:,I.altitude),'--','LineWidth',1.0);
yline(altitudeTarget,':');
grid on; xlabel('Time (s)'); ylabel('Altitude (ft)'); title('Altitude');
nexttile;
plot(outer.time,rad2deg(outer.simulated(:,I.control(1:3))),'LineWidth',1.1);
grid on; xlabel('Time (s)'); ylabel('Surface (deg)');
title('Automatic surfaces'); legend('Elevator','Aileron','Rudder');
nexttile;
plot(wind.time,rad2deg(wind.simulated(:,I.alpha)),'LineWidth',1.2);
hold on;
plot(wind.time,rad2deg(wind.simulated(:,I.beta)),'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Wind and gust air data'); legend('Alpha','Beta');
end
