%% C172S Autopilot Mode Manager V1 - executable Simulink check

clc;
clear;
close all;

try
    load_system('simulink');
catch ME
    error('runAutopilotModeManagerSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

manager = c172sAutopilotModeManagerData();
assertFrozenDependencies(manager);
modelName = buildC172sAutopilotModeManagerSimulink(true);
dt = manager.validation.sampleTime_s;

%% Independent implementation boundary and model configuration

managerPath = [modelName '/Independent Executable Mode Manager'];
root = sfroot;
chart = root.find('-isa','Stateflow.EMChart','Path',managerPath);
assert(numel(chart) == 1, ...
    'The generated executable manager chart could not be identified.');
embeddedSource = chart.Script;
assert(~contains(embeddedSource,'c172sAutopilotModeManagerStep'), ...
    ['The executable chart must be independent; it may not call the ', ...
    'MATLAB reference step.']);
assert(contains(embeddedSource,'persistent ap lat vert'), ...
    'The executable manager state is not present in the chart.');
assert(contains(embeddedSource,'timerUpdate'), ...
    'The executable confirmation timers are not present in the chart.');

assert(strcmp(get_param(modelName,'SolverType'),'Fixed-step'), ...
    'The generated supervisor must use a fixed-step solver.');
assert(strcmp(get_param(modelName,'Solver'),'FixedStepDiscrete'), ...
    'The generated supervisor must use the discrete solver.');
assert(abs(str2double(get_param(modelName,'FixedStep'))-dt) < eps, ...
    'The generated supervisor fixed step differs from its audit ledger.');
assert(abs(str2double(get_param( ...
    [modelName '/Timed Supervisor Inputs'],'SampleTime'))-dt) < eps, ...
    'The timed supervisor source must specify the explicit 0.01-s sample time.');
assert(strcmp(get_param([modelName '/Input Demux'],'Outputs'),'19'), ...
    'The executable supervisor input interface changed.');
assert(strcmp(get_param([modelName '/Output Mux'],'Inputs'),'20'), ...
    'The executable supervisor output interface changed.');

%% Twelve independent deterministic traces

scenarioNames = { ...
    'Default capture'; ...
    'Bounded capture'; ...
    'Direct outer modes'; ...
    'Transfers and updates'; ...
    'Rejected requests'; ...
    'Heading fallback'; ...
    'Altitude fallback'; ...
    'Pilot disconnect'; ...
    'Pilot override'; ...
    'Core-sensor disconnect'; ...
    'Saturation disconnect'; ...
    'Abnormal attitude'};
scenarioKeys = { ...
    'defaultCapture'; ...
    'boundedCapture'; ...
    'directOuter'; ...
    'transfers'; ...
    'rejectedRequests'; ...
    'headingFallback'; ...
    'altitudeFallback'; ...
    'pilotDisconnect'; ...
    'pilotOverride'; ...
    'coreDisconnect'; ...
    'saturationDisconnect'; ...
    'abnormalAttitude'};

captures = cell(size(scenarioKeys));
for k = 1:numel(scenarioKeys)
    captures{k} = runScenario(modelName,manager,scenarioKeys{k});
end

tolerance = 1.0e-12;
for k = 1:numel(captures)
    assert(captures{k}.maximumError <= tolerance, ...
        '%s trace error %.9g exceeds %.9g.', ...
        scenarioNames{k},captures{k}.maximumError,tolerance);
end

% Re-run a stateful fault trace to prove that persistent executable state
% resets at each simulation boundary and replays deterministically.
repeatCapture = runScenario(modelName,manager,'headingFallback');
assert(isequal(captures{6}.ySim,repeatCapture.ySim), ...
    'A repeated executable supervisor trace was not bitwise identical.');

%% Behavioral gates on the executable output traces

I = signalIndices();
E = manager.codes.event;
L = manager.codes.lateral;
V = manager.codes.vertical;

defaultCapture = captures{1};
engageSample = find(defaultCapture.ySim(:,I.eventCode) == E.ENGAGED,1);
assert(~isempty(engageSample), ...
    'The default engagement event was not emitted.');
assert(defaultCapture.ySim(engageSample,I.lateralMode) == L.ROLL && ...
    defaultCapture.ySim(engageSample,I.verticalMode) == V.PITCH && ...
    defaultCapture.ySim(engageSample,I.bankCommand) == 0.0 && ...
    defaultCapture.ySim(engageSample,I.pitchCommand) == 1.25 && ...
    defaultCapture.ySim(engageSample,I.rollTrack) == 1.0 && ...
    defaultCapture.ySim(engageSample,I.pitchTrack) == 1.0, ...
    'Default capture or engagement tracking changed.');
assert(sum(defaultCapture.ySim(:,I.rollTrack)) == 1.0 && ...
    sum(defaultCapture.ySim(:,I.pitchTrack)) == 1.0, ...
    'Engagement tracking requests must be one-sample pulses.');

boundedCapture = captures{2};
boundedSample = find(boundedCapture.ySim(:,I.eventCode) == E.ENGAGED,1);
assert(boundedCapture.ySim(boundedSample,I.bankCommand) == ...
    manager.capture.bankCommandLimit_deg && ...
    boundedCapture.ySim(boundedSample,I.pitchCommand) == ...
    manager.capture.pitchCommandLimit_deg, ...
    'Supervisory attitude capture authority changed.');
rejectionSample = find(boundedCapture.ySim(:,I.eventCode) == ...
    E.ENGAGE_REJECTED,1);
assert(~isempty(rejectionSample) && ...
    boundedCapture.ySim(rejectionSample,I.engaged) == 0.0, ...
    'Unstable engagement was not rejected.');

outerCapture = captures{3};
outerSample = find(outerCapture.ySim(:,I.eventCode) == E.ENGAGED,1);
assert(outerCapture.ySim(outerSample,I.lateralMode) == L.HEADING && ...
    outerCapture.ySim(outerSample,I.verticalMode) == V.ALTITUDE && ...
    outerCapture.ySim(outerSample,I.headingCommand) == 10.0 && ...
    outerCapture.ySim(outerSample,I.altitudeCommand) == 4250.0, ...
    'Direct outer-mode engagement or heading normalization changed.');

transferCapture = captures{4};
assert(sum(transferCapture.ySim(:,I.eventCode) == E.MODE_CHANGED) == 2, ...
    'The declared two-axis transfer sequence changed.');
assert(any(transferCapture.ySim(:,I.headingCommand) == 350.0) && ...
    any(transferCapture.ySim(:,I.altitudeCommand) == 4300.0), ...
    'Active outer references were not updated.');

rejectedCapture = captures{5};
rejectedSample = find(rejectedCapture.ySim(:,I.eventCode) == ...
    E.MODE_REJECTED,1);
assert(~isempty(rejectedSample) && ...
    rejectedCapture.ySim(rejectedSample,I.rollEnable) == 1.0 && ...
    rejectedCapture.ySim(rejectedSample,I.pitchEnable) == 1.0, ...
    'Invalid outer requests disturbed the active safe inner modes.');

headingFallback = captures{6};
headingFallbackSample = find(headingFallback.ySim(:,I.eventCode) == ...
    E.HEADING_FALLBACK,1);
assert(~isempty(headingFallbackSample) && ...
    headingFallback.ySim(headingFallbackSample,I.rollEnable) == 1.0 && ...
    headingFallback.ySim(headingFallbackSample,I.rollTrack) == 1.0 && ...
    headingFallback.ySim(headingFallbackSample,I.engaged) == 1.0, ...
    'Confirmed heading loss did not fall back to ROLL.');
assert(abs(headingFallback.ySim(headingFallbackSample,I.headingTimer)- ...
    manager.protection.outerSensorFallbackDelay_s) < 1.0e-12, ...
    'Heading fallback did not occur at its confirmation threshold.');

altitudeFallback = captures{7};
altitudeFallbackSample = find(altitudeFallback.ySim(:,I.eventCode) == ...
    E.ALTITUDE_FALLBACK,1);
assert(~isempty(altitudeFallbackSample) && ...
    altitudeFallback.ySim(altitudeFallbackSample,I.pitchEnable) == 1.0 && ...
    altitudeFallback.ySim(altitudeFallbackSample,I.pitchTrack) == 1.0 && ...
    altitudeFallback.ySim(altitudeFallbackSample,I.engaged) == 1.0, ...
    'Confirmed altitude loss did not fall back to PITCH.');
assert(abs(altitudeFallback.ySim(altitudeFallbackSample,I.altitudeTimer)- ...
    manager.protection.outerSensorFallbackDelay_s) < 1.0e-12, ...
    'Altitude fallback did not occur at its confirmation threshold.');

assertImmediateDisconnect(captures{8},E.DISCONNECT_COMMAND,I, ...
    'A/P DISC');
assertImmediateDisconnect(captures{9},E.DISCONNECT_OVERRIDE,I, ...
    'pilot override');
assertImmediateDisconnect(captures{12},E.DISCONNECT_ATTITUDE,I, ...
    'abnormal attitude');

coreCapture = captures{10};
coreSample = find(coreCapture.ySim(:,I.eventCode) == ...
    E.DISCONNECT_CORE_SENSOR,1);
assert(~isempty(coreSample) && ...
    abs(coreCapture.ySim(coreSample,I.coreTimer)- ...
    manager.protection.coreSensorDisconnectDelay_s) < 1.0e-12 && ...
    coreCapture.ySim(coreSample,I.engaged) == 0.0 && ...
    coreCapture.ySim(coreSample,I.disconnectTone) == 1.0, ...
    'Confirmed core-sensor loss did not disconnect at its threshold.');

saturationCapture = captures{11};
saturationSample = find(saturationCapture.ySim(:,I.eventCode) == ...
    E.DISCONNECT_SATURATION,1);
assert(~isempty(saturationSample) && ...
    abs(saturationCapture.ySim(saturationSample,I.aileronSatTimer)- ...
    manager.protection.sustainedSaturationDisconnectDelay_s) < 1.0e-10 && ...
    saturationCapture.ySim(saturationSample,I.engaged) == 0.0 && ...
    saturationCapture.ySim(saturationSample,I.disconnectTone) == 1.0, ...
    'Sustained actuator saturation did not disconnect at its threshold.');

for k = 1:numel(captures)
    y = captures{k}.ySim;
    assert(all((y(:,I.rollEnable)+y(:,I.headingEnable)) <= 1.0), ...
        'Lateral mode exclusivity failed in %s.',scenarioNames{k});
    assert(all((y(:,I.pitchEnable)+y(:,I.altitudeEnable)) <= 1.0), ...
        'Vertical mode exclusivity failed in %s.',scenarioNames{k});
    released = y(:,I.engaged) == 0.0;
    assert(all(y(released,I.bankCommand) == 0.0) && ...
        all(y(released,I.pitchCommand) == 0.0), ...
        'Disengaged attitude commands were not released in %s.', ...
        scenarioNames{k});
end

%% Evidence plots

figure('Name','C172S Autopilot Mode Manager V1 - Mode Traces', ...
    'Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
stairs(transferCapture.t,transferCapture.ySim(:,I.lateralMode), ...
    'LineWidth',1.5);
hold on;
stairs(transferCapture.t,transferCapture.yReference(:,I.lateralMode), ...
    '--','LineWidth',1.2);
grid on;
ylim([-0.2 2.2]);
yticks([0 1 2]);
yticklabels({'OFF','ROLL','HEADING'});
xlabel('Time (s)');
title('Lateral Mode Transfers');
legend('Simulink','Independent MATLAB','Location','best');

nexttile;
stairs(transferCapture.t,transferCapture.ySim(:,I.verticalMode), ...
    'LineWidth',1.5);
hold on;
stairs(transferCapture.t,transferCapture.yReference(:,I.verticalMode), ...
    '--','LineWidth',1.2);
grid on;
ylim([-0.2 2.2]);
yticks([0 1 2]);
yticklabels({'OFF','PITCH','ALTITUDE'});
xlabel('Time (s)');
title('Vertical Mode Transfers');

nexttile;
plot(transferCapture.t,transferCapture.ySim(:,I.bankCommand), ...
    'LineWidth',1.5);
hold on;
plot(transferCapture.t,transferCapture.ySim(:,I.headingCommand), ...
    'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Command (deg)');
title('Lateral References');
legend('Bank','Heading','Location','best');

nexttile;
plot(transferCapture.t,transferCapture.ySim(:,I.pitchCommand), ...
    'LineWidth',1.5);
hold on;
plot(transferCapture.t,transferCapture.ySim(:,I.altitudeCommand), ...
    'LineWidth',1.5);
grid on;
xlabel('Time (s)');
title('Vertical References');
legend('Pitch (deg)','Altitude (ft)','Location','best');
sgtitle('Autopilot Mode Manager V1: Executable Supervisor Equivalence');

figure('Name','C172S Autopilot Mode Manager V1 - Protection', ...
    'Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
plot(headingFallback.t,headingFallback.ySim(:,I.headingTimer), ...
    'LineWidth',1.5);
hold on;
yline(manager.protection.outerSensorFallbackDelay_s,'--');
stairs(headingFallback.t,headingFallback.ySim(:,I.eventCode)/50, ...
    'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Time / scaled event');
title('Heading-Source Fallback');

nexttile;
plot(coreCapture.t,coreCapture.ySim(:,I.coreTimer),'LineWidth',1.5);
hold on;
yline(manager.protection.coreSensorDisconnectDelay_s,'--');
stairs(coreCapture.t,coreCapture.ySim(:,I.engaged)/10, ...
    'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Time / scaled engaged');
title('Core-Sensor Disconnect');

nexttile;
plot(saturationCapture.t, ...
    saturationCapture.ySim(:,I.aileronSatTimer),'LineWidth',1.5);
hold on;
yline(manager.protection.sustainedSaturationDisconnectDelay_s,'--');
grid on;
xlabel('Time (s)');
ylabel('Saturation time (s)');
title('Sustained-Saturation Disconnect');

nexttile;
stairs(captures{8}.t,captures{8}.ySim(:,I.eventCode), ...
    'LineWidth',1.5);
hold on;
stairs(captures{9}.t,captures{9}.ySim(:,I.eventCode), ...
    '--','LineWidth',1.5);
stairs(captures{12}.t,captures{12}.ySim(:,I.eventCode), ...
    ':','LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Event code');
title('Immediate Disconnect Priority');
legend('A/P DISC','Override','Attitude','Location','best');
sgtitle('Autopilot Mode Manager V1: Fault Confirmation and Protection');

%% Console report

fprintf('\n============================================================\n');
fprintf(' C172S AUTOPILOT MODE MANAGER V1 - SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('%-37s: %s\n','Model',modelName);
fprintf('%-37s: %s\n','Artifact',manager.artifactRevision);
fprintf('%-37s: %d\n','Frozen dependency interfaces',12);
fprintf('%-37s: %.3f s\n','Supervisor sample time',dt);
fprintf('%-37s: %s / %.4f s\n','Solver / fixed step', ...
    get_param(modelName,'Solver'), ...
    str2double(get_param(modelName,'FixedStep')));
fprintf('%-37s: %d / %d\n\n','Independent traces passed', ...
    numel(captures),numel(captures));

for k = 1:numel(captures)
    fprintf('%-20s maximum error       : %.9g\n', ...
        scenarioNames{k},captures{k}.maximumError);
    fprintf('%-20s RMS error           : %.9g\n', ...
        scenarioNames{k},captures{k}.rmsError);
end
fprintf('%-37s: %.6g\n\n','Allowed implementation error',tolerance);

fprintf('%-37s: %.3f s\n','Heading fallback confirmation', ...
    headingFallback.ySim(headingFallbackSample,I.headingTimer));
fprintf('%-37s: %.3f s\n','Altitude fallback confirmation', ...
    altitudeFallback.ySim(altitudeFallbackSample,I.altitudeTimer));
fprintf('%-37s: %.3f s\n','Core-sensor disconnect confirmation', ...
    coreCapture.ySim(coreSample,I.coreTimer));
fprintf('%-37s: %.3f s\n','Saturation disconnect dwell', ...
    saturationCapture.ySim(saturationSample,I.aileronSatTimer));
fprintf('%-37s: %d / %d\n\n','Mode exclusivity gates',2,2);

fprintf('PASS: Simulink matches twelve independent supervisor traces.\n');
fprintf(['PASS: the executable chart does not call the MATLAB ', ...
    'reference step.\n']);
fprintf(['PASS: default capture, two-axis exclusivity and reference ', ...
    'updates pass.\n']);
fprintf(['PASS: outer-source fallbacks preserve engagement and emit ', ...
    'tracking pulses.\n']);
fprintf(['PASS: pilot, core-sensor, saturation and abnormal-attitude ', ...
    'disconnects pass.\n']);
fprintf('PASS: repeated persistent-state execution is bitwise identical.\n');
fprintf(['NOTE: this validates the executable supervisory boundary; ', ...
    'coupled continuous-plant integration remains the next stage.\n']);
fprintf(['NOTE: project thresholds and timers are assumptions; this is ', ...
    'not aircraft approval.\n']);

save_system(modelName);
close_system(modelName,0);
load_system(modelName);
open_system(modelName);
fprintf('Reloaded generated model to clear transient editor diagnostics.\n');

%% Local helpers

function capture = runScenario(modelName,manager,key)
[t,U] = scenarioInputs(manager,key);
inputSignal = [t U];
simInput = Simulink.SimulationInput(modelName);
simInput = simInput.setVariable('amm1InputSignal',inputSignal);
simInput = simInput.setModelParameter('StopTime',num2str(t(end),17));
simOut = sim(simInput);
[tSim,ySim] = extractArrayOutput(simOut,20);
assert(numel(tSim) == numel(t) && max(abs(tSim-t)) < 1.0e-12, ...
    'The executable sample grid changed for scenario %s.',key);
yReference = referenceTrace(U,manager);
errorSignal = ySim-yReference;
capture.key = key;
capture.t = tSim;
capture.inputs = U;
capture.ySim = ySim;
capture.yReference = yReference;
capture.maximumError = max(abs(errorSignal),[],'all');
capture.rmsError = sqrt(mean(errorSignal(:).^2));
end

function [t,U] = scenarioInputs(manager,key)
dt = manager.validation.sampleTime_s;
switch key
    case 'saturationDisconnect'
        stopTime = 2.40;
    otherwise
        stopTime = 0.40;
end
t = (0:dt:stopTime).';
U = repmat(nominalInputRow(manager),numel(t),1);

switch key
    case 'defaultCapture'
        U(:,14) = 4.0;
        U(:,15) = 1.25;
        U(2,1) = 1.0;

    case 'boundedCapture'
        % First reject an unstable engagement, then accept and bound the
        % captured attitudes at the frozen supervisory authorities.
        U(:,14) = 25.0;
        U(:,15) = 5.0;
        U(2,4) = 0.0;
        U(2,1) = 1.0;
        U(5,1) = 1.0;

    case 'directOuter'
        U(:,18) = 370.0;
        U(:,19) = 4250.0;
        U(2,1) = 1.0;
        U(2,10) = double(manager.codes.lateral.HEADING);
        U(2,11) = double(manager.codes.vertical.ALTITUDE);

    case 'transfers'
        U(:,14) = -12.0;
        U(:,15) = -1.5;
        U(:,18) = 350.0;
        U(:,19) = 4100.0;
        U(2,1) = 1.0;
        U(2,10) = double(manager.codes.lateral.HEADING);
        U(2,11) = double(manager.codes.vertical.ALTITUDE);
        U(10,10) = double(manager.codes.lateral.ROLL);
        U(10,11) = double(manager.codes.vertical.PITCH);
        U(18,10) = double(manager.codes.lateral.HEADING);
        U(18,11) = double(manager.codes.vertical.ALTITUDE);
        U(26,12) = 1.0;
        U(26,13) = 1.0;
        U(26,18) = -10.0;
        U(26,19) = 4300.0;

    case 'rejectedRequests'
        U(2,1) = 1.0;
        U(10,6) = 0.0;
        U(10,7) = 0.0;
        U(10,10) = double(manager.codes.lateral.HEADING);
        U(10,11) = double(manager.codes.vertical.ALTITUDE);

    case 'headingFallback'
        U(:,14) = 8.0;
        U(:,18) = 45.0;
        U(2,1) = 1.0;
        U(2,10) = double(manager.codes.lateral.HEADING);
        U(8:end,6) = 0.0;

    case 'altitudeFallback'
        U(:,15) = 2.0;
        U(:,19) = 4250.0;
        U(2,1) = 1.0;
        U(2,11) = double(manager.codes.vertical.ALTITUDE);
        U(8:end,7) = 0.0;

    case 'pilotDisconnect'
        U(2,1) = 1.0;
        U(10,2) = 1.0;
        U(10,10) = double(manager.codes.lateral.HEADING);
        U(10,11) = double(manager.codes.vertical.ALTITUDE);

    case 'pilotOverride'
        U(2,1) = 1.0;
        U(10,3) = 1.0;
        U(10,10) = double(manager.codes.lateral.HEADING);
        U(10,11) = double(manager.codes.vertical.ALTITUDE);

    case 'coreDisconnect'
        U(2,1) = 1.0;
        U(8:end,5) = 0.0;

    case 'saturationDisconnect'
        U(2,1) = 1.0;
        U(8:end,8) = 1.0;

    case 'abnormalAttitude'
        U(2,1) = 1.0;
        U(10,14) = manager.protection.maximumAbsoluteBank_deg+0.1;
        U(10,2) = 0.0;
        U(10,10) = double(manager.codes.lateral.HEADING);

    otherwise
        error('Unknown supervisor scenario: %s',key);
end
end

function row = nominalInputRow(manager)
row = zeros(1,19);
row(4:7) = 1.0;
row(10) = double(manager.codes.lateral.OFF);
row(11) = double(manager.codes.vertical.OFF);
row(16) = 0.0;
row(17) = 4000.0;
row(18) = 0.0;
row(19) = 4000.0;
end

function y = referenceTrace(U,manager)
dt = manager.validation.sampleTime_s;
y = zeros(size(U,1),20);
state = [];
for k = 1:size(U,1)
    input = inputStruct(U(k,:));
    [state,output] = c172sAutopilotModeManagerStep( ...
        state,input,manager,dt);
    y(k,:) = outputRow(output,state);
end
end

function input = inputStruct(row)
input.engageRequest = logical(row(1));
input.disconnectRequest = logical(row(2));
input.pilotOverride = logical(row(3));
input.trimStable = logical(row(4));
input.coreSensorsValid = logical(row(5));
input.headingValid = logical(row(6));
input.altitudeValid = logical(row(7));
input.aileronSaturated = logical(row(8));
input.elevatorSaturated = logical(row(9));
input.lateralModeRequest = uint8(row(10));
input.verticalModeRequest = uint8(row(11));
input.updateHeadingReference = logical(row(12));
input.updateAltitudeReference = logical(row(13));
input.bank_deg = row(14);
input.pitch_deg = row(15);
input.heading_deg = row(16);
input.altitude_ft = row(17);
input.selectedHeading_deg = row(18);
input.selectedAltitude_ft = row(19);
end

function row = outputRow(output,state)
row = [ ...
    double(output.engaged), ...
    double(output.lateralMode), ...
    double(output.verticalMode), ...
    double(output.rollModeEnable), ...
    double(output.headingModeEnable), ...
    double(output.pitchModeEnable), ...
    double(output.altitudeModeEnable), ...
    output.bankCommand_deg, ...
    output.pitchCommand_deg, ...
    output.headingCommand_deg, ...
    output.altitudeCommand_ft, ...
    double(output.rollIntegratorTrackRequest), ...
    double(output.pitchIntegratorTrackRequest), ...
    double(output.disconnectToneRequest), ...
    double(output.eventCode), ...
    state.coreInvalidTime_s, ...
    state.headingInvalidTime_s, ...
    state.altitudeInvalidTime_s, ...
    state.aileronSaturationTime_s, ...
    state.elevatorSaturationTime_s];
end

function I = signalIndices()
I.engaged = 1;
I.lateralMode = 2;
I.verticalMode = 3;
I.rollEnable = 4;
I.headingEnable = 5;
I.pitchEnable = 6;
I.altitudeEnable = 7;
I.bankCommand = 8;
I.pitchCommand = 9;
I.headingCommand = 10;
I.altitudeCommand = 11;
I.rollTrack = 12;
I.pitchTrack = 13;
I.disconnectTone = 14;
I.eventCode = 15;
I.coreTimer = 16;
I.headingTimer = 17;
I.altitudeTimer = 18;
I.aileronSatTimer = 19;
I.elevatorSatTimer = 20;
end

function assertImmediateDisconnect(capture,eventCode,I,label)
sample = find(capture.ySim(:,I.eventCode) == eventCode,1);
assert(~isempty(sample) && capture.ySim(sample,I.engaged) == 0.0 && ...
    capture.ySim(sample,I.lateralMode) == 0.0 && ...
    capture.ySim(sample,I.verticalMode) == 0.0 && ...
    capture.ySim(sample,I.disconnectTone) == 1.0, ...
    '%s did not immediately release both axes.',label);
end

function [t,y] = extractArrayOutput(simOut,nSignal)
t = simOut.tout;
y = simOut.yout;
if isa(y,'timeseries')
    t = y.Time;
    y = y.Data;
end
t = t(:);
y = squeeze(y);
if size(y,1) ~= numel(t) && size(y,2) == numel(t)
    y = y.';
end
assert(size(y,1) == numel(t), ...
    'The logged supervisor output does not align with tout.');
assert(size(y,2) == nSignal, ...
    'Expected %d logged signals but received %d.',nSignal,size(y,2));
end

function assertFrozenDependencies(manager)
longitudinal = c172sLongitudinalData();
pitchRate = c172sPitchRateDamperData();
pitchAttitude = c172sPitchAttitudeControllerData();
elevator = c172sElevatorActuatorData();
altitude = c172sAltitudeHoldControllerData();
lateral = c172sLateralDirectionalData();
yaw = c172sYawDamperData();
rudder = c172sRudderActuatorData();
aileron = c172sAileronActuatorData();
roll = c172sRollAttitudeControllerData();
headingSensor = c172sHeadingSensorData();
heading = c172sHeadingHoldControllerData();

checks = [ ...
    strcmp(longitudinal.model.version,manager.required.longitudinalPlant); ...
    strcmp(pitchRate.version,manager.required.pitchRateDamper); ...
    strcmp(pitchAttitude.version,manager.required.pitchAttitudeController); ...
    strcmp(elevator.integratedLoopVersion,manager.required.pitchLoop); ...
    strcmp(altitude.version,manager.required.altitudeLoop); ...
    strcmp(lateral.model.version,manager.required.lateralPlant); ...
    strcmp(yaw.artifactRevision,manager.required.yawController); ...
    strcmp(rudder.artifactRevision,manager.required.rudderAudit); ...
    strcmp(aileron.artifactRevision,manager.required.aileronAudit); ...
    strcmp(roll.artifactRevision,manager.required.rollController); ...
    strcmp(headingSensor.artifactRevision, ...
        manager.required.headingSensorAudit); ...
    strcmp(heading.artifactRevision,manager.required.headingController)];
assert(all(checks), ...
    'runAutopilotModeManagerSimulinkCheck:DependencyMismatch', ...
    'At least one frozen dependency does not match the manager ledger.');
end
