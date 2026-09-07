%% C172S Integrated Autopilot V1 - executable integration check

clc;
clear;
close all;

integration = c172sIntegratedAutopilotData();
manager = c172sAutopilotModeManagerData();
assertDependencies(integration,manager);
modelName = buildC172sIntegratedAutopilotSimulink(true);
load_system(modelName);

keys = { ...
    'disengagedTrim'; ...
    'captureEngagement'; ...
    'simultaneousOuter'; ...
    'headingToRoll'; ...
    'altitudeToPitch'; ...
    'headingFallback'; ...
    'altitudeFallback'; ...
    'pilotDisconnect'; ...
    'disconnectReengage'; ...
    'pilotOverride'; ...
    'aileronSaturation'; ...
    'elevatorSaturation'; ...
    'deterministicReplay'};
names = integration.scenarios;
assert(numel(keys) == numel(names));

captures = cell(size(keys));
for k = 1:numel(keys)-1
    captures{k} = runCapture(modelName,manager,keys{k});
end
captures{end} = runCapture(modelName,manager,'simultaneousOuter');

I = indices();
pass = false(13,1);
detail = strings(13,1);

%% 1. Disengaged trim releases every automatic path.
y = captures{1}.y;
pass(1) = all(y(:,I.engaged) == 0) && ...
    max(abs(rad2deg(y(:,I.elevator)))) < 1e-10 && ...
    max(abs(rad2deg(y(:,I.aileron)))) < 1e-10 && ...
    max(abs(rad2deg(y(:,I.rudder)))) < 1e-10;
detail(1) = 'OFF/OFF with zero automatic elevator, aileron and rudder';

%% 2. Default attitude engagement and both tracking handshakes.
y = captures{2}.y;
pass(2) = any(y(:,I.rollTrack) == 1) && any(y(:,I.pitchTrack) == 1) && ...
    all(y(end,[I.lateralMode I.verticalMode]) == [1 1]);
detail(2) = 'ROLL/PITCH capture with one-sample tracking requests';

%% 3. Independent simultaneous-axis executable equivalence.
outer = captures{3};
[longReference,lateralReference] = standaloneReferences(outer.t(end));
longError = outer.y(:,I.longitudinal)-longReference;
lateralError = outer.y(:,I.lateral)-lateralReference;
maximumLongitudinalError = max(abs(longError),[],'all');
maximumLateralError = max(abs(lateralError),[],'all');
headingFinalError_deg = abs(rad2deg(wrapAngle( ...
    deg2rad(30)-outer.y(end,I.heading))));
altitudeFinalError_ft = abs(100-outer.y(end,I.altitude)/0.3048);
pass(3) = maximumLongitudinalError <= ...
    integration.requirements.maximumImplementationError && ...
    maximumLateralError <= integration.requirements.maximumImplementationError && ...
    headingFinalError_deg <= integration.requirements.maximumHeadingFinalError_deg && ...
    altitudeFinalError_ft <= integration.requirements.maximumAltitudeFinalError_ft;
detail(3) = sprintf(['standalone errors %.3g/%.3g; final heading/altitude ', ...
    'errors %.4f deg/%.4f ft'],maximumLongitudinalError, ...
    maximumLateralError,headingFinalError_deg,altitudeFinalError_ft);

%% 4-5. Outer-to-inner transfers remain actuator-continuous.
[aStep,~] = transitionResidual(captures{4},40.0,I.aileron,I.aileronRate);
pass(4) = aStep <= integration.requirements.maximumTransferAileronStep_deg && ...
    captures{4}.y(end,I.lateralMode) == 1;
detail(4) = sprintf('HEADING->ROLL aileron discontinuity residual %.5f deg',aStep);

[eStep,~] = transitionResidual(captures{5},80.0,I.elevator,I.elevatorRate);
pass(5) = eStep <= integration.requirements.maximumTransferElevatorStep_deg && ...
    captures{5}.y(end,I.verticalMode) == 1;
detail(5) = sprintf('ALTITUDE->PITCH elevator discontinuity residual %.5f deg',eStep);

%% 6-7. Confirmed outer-source loss degrades only the affected axis.
y = captures{6}.y;
pass(6) = y(end,I.engaged) == 1 && y(end,I.lateralMode) == 1 && ...
    y(end,I.verticalMode) == 2 && any(y(:,I.eventCode) == 5);
detail(6) = 'confirmed heading loss falls back to ROLL; ALTITUDE remains active';

y = captures{7}.y;
pass(7) = y(end,I.engaged) == 1 && y(end,I.lateralMode) == 2 && ...
    y(end,I.verticalMode) == 1 && any(y(:,I.eventCode) == 6);
detail(7) = 'confirmed altitude loss falls back to PITCH; HEADING remains active';

%% 8-10. Disconnect, re-engagement and pilot override priorities.
[aDisc,~] = transitionResidual(captures{8},20.0,I.aileron,I.aileronRate);
[eDisc,~] = transitionResidual(captures{8},20.0,I.elevator,I.elevatorRate);
y = captures{8}.y;
pass(8) = any(y(:,I.eventCode) == 7) && y(end,I.engaged) == 0 && ...
    aDisc <= integration.requirements.maximumDisconnectAileronStep_deg && ...
    eDisc <= integration.requirements.maximumDisconnectElevatorStep_deg;
detail(8) = sprintf('release residuals aileron/elevator %.5f/%.5f deg',aDisc,eDisc);

y = captures{9}.y;
engagementCount = sum(diff([0;y(:,I.engaged)]) > 0.5);
pass(9) = engagementCount == 2 && y(end,I.engaged) == 1 && ...
    max(abs(rad2deg(y(:,I.aileronRate)))) <= 30+1e-8 && ...
    max(abs(rad2deg(y(:,I.elevatorRate)))) <= 20+1e-8;
detail(9) = 'disconnect followed by recapture retains both actuator-rate limits';

y = captures{10}.y;
pass(10) = any(y(:,I.eventCode) == 8) && y(end,I.engaged) == 0;
detail(10) = 'pilot override has immediate global-disconnect priority';

%% 11-12. Sustained protection flags propagate through the integrated gate.
y = captures{11}.y;
pass(11) = any(y(:,I.eventCode) == 10) && y(end,I.engaged) == 0 && ...
    max(y(:,I.aileronTimer)) >= ...
    manager.protection.sustainedSaturationDisconnectDelay_s- ...
    manager.validation.sampleTime_s;
detail(11) = 'sustained aileron-protection status disconnects both axes';

y = captures{12}.y;
pass(12) = any(y(:,I.eventCode) == 10) && y(end,I.engaged) == 0 && ...
    max(y(:,I.elevatorTimer)) >= ...
    manager.protection.sustainedSaturationDisconnectDelay_s- ...
    manager.validation.sampleTime_s;
detail(12) = 'sustained elevator-protection status disconnects both axes';

%% 13. Full executable replay is sample-for-sample deterministic.
pass(13) = isequal(captures{3}.t,captures{13}.t) && ...
    isequal(captures{3}.y,captures{13}.y);
detail(13) = 'simultaneous two-axis trace is bitwise identical on replay';

fprintf('============================================================\n');
fprintf(' C172S INTEGRATED AUTOPILOT V1 - EXECUTABLE SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('%-39s: %s\n','Model',modelName);
fprintf('%-39s: %s\n','Integration artifact',integration.artifactRevision);
fprintf('%-39s: %s\n','Mode Manager',manager.artifactRevision);
fprintf('%-39s: %s / %.4f s\n','Solver / fixed step', ...
    integration.execution.solver,integration.execution.fixedStep_s);
fprintf('%-39s: %d = 20 + 18 + 52\n\n','Verification signals',90);
for k = 1:numel(pass)
    label = 'PASS'; if ~pass(k), label = 'FAIL'; end
    fprintf('[%s] %-39s %s\n',label,names{k},detail(k));
end
fprintf('\nPassed integrated scenarios            : %d/%d\n',sum(pass),numel(pass));
fprintf('Independent longitudinal maximum error : %.8g\n',maximumLongitudinalError);
fprintf('Independent lateral maximum error      : %.8g\n',maximumLateralError);
fprintf('Final heading / altitude error          : %.6f deg / %.6f ft\n', ...
    headingFinalError_deg,altitudeFinalError_ft);

assert(all(pass),'runIntegratedAutopilotSimulinkCheck:IntegrationGateFailed', ...
    '%d of %d integrated scenarios failed.',sum(~pass),numel(pass));
fprintf('\nPASS: both frozen plants execute simultaneously under Mode Manager V1.\n');
fprintf('PASS: standalone longitudinal and lateral executables are reproduced.\n');
fprintf('PASS: transfers, fallbacks, disconnects and deterministic replay pass.\n');
fprintf(['NOTE: the plants remain simultaneous but aerodynamically uncoupled ', ...
    'single-trim models; this is not aircraft approval.\n']);

%% Evidence figures
figure('Color','w','Name','Integrated Autopilot V1 - simultaneous capture');
subplot(2,2,1); plot(outer.t,rad2deg(outer.y(:,I.heading)),'LineWidth',1.2); ...
    yline(30,'k--'); grid on; ylabel('Heading (deg)');
subplot(2,2,2); plot(outer.t,outer.y(:,I.altitude)/0.3048,'LineWidth',1.2); ...
    yline(100,'k--'); grid on; ylabel('Altitude change (ft)');
subplot(2,2,3); plot(outer.t,rad2deg(outer.y(:,I.aileron)),'LineWidth',1.2); ...
    grid on; ylabel('Auto aileron (deg)'); xlabel('Time (s)');
subplot(2,2,4); plot(outer.t,rad2deg(outer.y(:,I.elevator)),'LineWidth',1.2); ...
    grid on; ylabel('Auto elevator (deg)'); xlabel('Time (s)');

figure('Color','w','Name','Integrated Autopilot V1 - mode events');
stairs(captures{9}.t,captures{9}.y(:,I.engaged),'LineWidth',1.2); hold on;
stairs(captures{9}.t,captures{9}.y(:,I.lateralMode),'LineWidth',1.2);
stairs(captures{9}.t,captures{9}.y(:,I.verticalMode),'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Mode code');
legend('Engaged','Lateral','Vertical','Location','best');

set_param(modelName,'SimulationCommand','update');
save_system(modelName);
close_system(modelName,0);
load_system(modelName);
open_system(modelName);
fprintf('Reloaded generated model to clear transient editor diagnostics.\n');

function capture = runCapture(modelName,manager,key)
[t,U,variables] = scenario(manager,key);
simInput = Simulink.SimulationInput(modelName);
simInput = simInput.setVariable('ia1InputSignal',[t U]);
simInput = simInput.setModelParameter('StopTime',num2str(t(end),17));
for k = 1:size(variables,1)
    simInput = simInput.setVariable(variables{k,1},variables{k,2});
end
simOut = sim(simInput);
[tSim,y] = extractArrayOutput(simOut,90);
assert(numel(tSim) == numel(t) && max(abs(tSim-t)) < 1e-10, ...
    'The integrated sample grid changed in %s.',key);
capture.key = key; capture.t = tSim; capture.U = U; capture.y = y;
end

function [t,U,variables] = scenario(manager,key)
dt = manager.validation.sampleTime_s;
variables = cell(0,2);
switch key
    case 'simultaneousOuter', stopTime = 180;
    case {'headingToRoll','altitudeToPitch'}, stopTime = 120;
    case {'headingFallback','altitudeFallback'}, stopTime = 60;
    case {'pilotDisconnect','disconnectReengage','pilotOverride'}, stopTime = 100;
    case {'aileronSaturation','elevatorSaturation'}, stopTime = 4;
    otherwise, stopTime = 5;
end
t = (0:dt:stopTime).';
row = zeros(1,19); row(4:7) = 1; row(17) = 4000; row(19) = 4000;
U = repmat(row,numel(t),1);

switch key
    case 'disengagedTrim'
    case 'captureEngagement'
        U(1,1) = 1;
    case 'simultaneousOuter'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100;
    case 'headingToRoll'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100;
        U(abs(t-40) < dt/2,10) = 1;
    case 'altitudeToPitch'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100;
        U(abs(t-80) < dt/2,11) = 1;
    case 'headingFallback'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100; U(t >= 40,6) = 0;
    case 'altitudeFallback'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100; U(t >= 40,7) = 0;
    case 'pilotDisconnect'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100; U(abs(t-20) < dt/2,2) = 1;
    case 'disconnectReengage'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100;
        reconnect = abs(t-50) < dt/2;
        U(abs(t-20) < dt/2,2) = 1; U(reconnect,1) = 1;
        U(reconnect,10) = 2; U(reconnect,11) = 2;
    case 'pilotOverride'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 4100; U(abs(t-20) < dt/2,3) = 1;
    case 'aileronSaturation'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 90; U(:,19) = 4100; U(t >= 0.10,8) = 1;
    case 'elevatorSaturation'
        U(1,1) = 1; U(1,10) = 2; U(1,11) = 2;
        U(:,18) = 30; U(:,19) = 5000; U(t >= 0.10,9) = 1;
    otherwise
        error('Unknown integrated scenario: %s',key);
end
end

function [longitudinal,lateral] = standaloneReferences(stopTime)
altitudeModel = 'C172S_AltitudeHold_V1';
headingModel = 'C172S_HeadingHold_V1';
load_system(altitudeModel); load_system(headingModel);
in = Simulink.SimulationInput(altitudeModel);
in = in.setVariable('hCommand_ah1_m',100*0.3048);
in = in.setModelParameter('StopTime',num2str(stopTime,17), ...
    'FixedStep','0.01');
out = sim(in); [~,longitudinal] = extractArrayOutput(out,18);
in = Simulink.SimulationInput(headingModel);
in = in.setVariable('headingCommand_hh1_rad',deg2rad(30));
in = in.setModelParameter('StopTime',num2str(stopTime,17));
out = sim(in); [~,lateral] = extractArrayOutput(out,52);
close_system(altitudeModel,0); close_system(headingModel,0);
end

function [residual,index] = transitionResidual(capture,eventTime,position,rate)
[~,index] = min(abs(capture.t-eventTime));
assert(index > 1);
dt = capture.t(index)-capture.t(index-1);
predicted = capture.y(index-1,position)+dt*capture.y(index-1,rate);
residual = abs(rad2deg(capture.y(index,position)-predicted));
end

function I = indices()
I.engaged = 1; I.lateralMode = 2; I.verticalMode = 3;
I.rollTrack = 12; I.pitchTrack = 13; I.eventCode = 15;
I.aileronTimer = 19; I.elevatorTimer = 20;
I.longitudinal = 21:38; I.lateral = 39:90;
I.elevator = 20+7; I.altitude = 20+8; I.elevatorRate = 20+17;
I.heading = 20+18+5; I.rudder = 20+18+26;
I.aileron = 20+18+46; I.aileronRate = 20+18+48;
end

function [t,y] = extractArrayOutput(simOut,nSignal)
t = simOut.tout(:);
y = simOut.yout;
if isa(y,'timeseries'), t = y.Time(:); y = y.Data; end
if isstruct(y) && isfield(y,'signals'), y = y.signals.values; end
y = squeeze(y);
if size(y,1) ~= numel(t) && size(y,2) == numel(t), y = y.'; end
assert(size(y,1) == numel(t) && size(y,2) == nSignal, ...
    'Expected %d output signals; received %s.',nSignal,mat2str(size(y)));
end

function value = wrapAngle(value)
value = mod(value+pi,2*pi)-pi;
end

function assertDependencies(integration,manager)
altitude = c172sAltitudeHoldControllerData();
heading = c172sHeadingHoldControllerData();
assert(strcmp(manager.artifactRevision,integration.required.modeManager));
assert(strcmp(altitude.version,integration.required.altitudeLoop));
assert(strcmp(heading.artifactRevision,integration.required.headingController));
assert(numel(integration.scenarios) == 13);
end
