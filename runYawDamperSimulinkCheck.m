%% C172S Yaw Damper V1 - Simulink equivalence and release check

clc;
clear;
close all;

try
    load_system('simulink');
catch ME
    error('runYawDamperSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

data = c172sLateralDirectionalData();
[~,plant] = c172sLateralDirectionalPlant(data);
controller = c172sYawDamperData();
assert(strcmp(data.model.version,controller.requiredPlantVersion), ...
    'Yaw Damper V1 plant dependency mismatch.');
assert(strcmp(controller.artifactRevision,'YD-V1-MATLAB-R2'), ...
    'The Simulink check requires artifact YD-V1-MATLAB-R2.');

% Always rebuild from a clean diagram so a stale or interrupted model
% cannot influence the release result.
modelName = buildC172sYawDamperSimulink(true);

zeroState = zeros(5,1);
initialState = zeroState;
initialState(3) = deg2rad(controller.validation.initialYawRate_degps);

initialCapture = runCapture(modelName,plant,controller, ...
    initialState,0.0,0.0);
aileronCapture = runCapture(modelName,plant,controller,zeroState, ...
    deg2rad(controller.validation.aileronPulse_deg),0.0);
rudderCapture = runCapture(modelName,plant,controller,zeroState,0.0, ...
    deg2rad(controller.validation.rudderPulse_deg));

%% Independent-equivalence gates

tolerance = 1.0e-5;
assert(initialCapture.maxError < tolerance, ...
    'Initial-disturbance error %.6g exceeds %.6g.', ...
    initialCapture.maxError,tolerance);
assert(aileronCapture.maxError < tolerance, ...
    'Aileron-capture error %.6g exceeds %.6g.', ...
    aileronCapture.maxError,tolerance);
assert(rudderCapture.maxError < tolerance, ...
    'Rudder-capture error %.6g exceeds %.6g.', ...
    rudderCapture.maxError,tolerance);

assert(strcmp(get_param(modelName,'SolverType'),'Fixed-step'), ...
    'The generated model must use a fixed-step solver.');
assert(strcmp(get_param(modelName,'Solver'),'ode4'), ...
    'The generated model must use ode4.');
assert(abs(str2double(get_param(modelName,'FixedStep')) - 0.01) < eps, ...
    'The generated model fixed step must be 0.01 s.');
assert(strcmp(get_param([modelName '/Automatic Rudder Gain'],'Gain'), ...
    '-Kr_yd1'),'The automatic-rudder feedback sign or gain changed.');
assert(strcmp(get_param([modelName '/Yaw-Rate Washout'],'A'), ...
    '-1/Tw_yd1'),'The washout state equation changed.');
assert(strcmp(get_param([modelName '/Yaw-Rate Washout'],'D'),'1'), ...
    'The washout direct yaw-rate path changed.');

%% Operational and control-interference gates using Simulink outputs

req = controller.requirements;
initialYawRateIntegralRatio = ...
    trapz(initialCapture.t,abs(initialCapture.ySim(:,3)))/ ...
    trapz(initialCapture.t,abs(initialCapture.openStates(:,3)));
initialBetaIntegralRatio = ...
    trapz(initialCapture.t,abs(initialCapture.ySim(:,1)))/ ...
    trapz(initialCapture.t,abs(initialCapture.openStates(:,1)));
peakInitialAutomaticRudder_deg = max(abs( ...
    rad2deg(initialCapture.ySim(:,9))));

aileronYawIntegralRatio = ...
    trapz(aileronCapture.t,abs(aileronCapture.ySim(:,3)))/ ...
    trapz(aileronCapture.t,abs(aileronCapture.openStates(:,3)));
aileronBankPeakRatio = max(abs(aileronCapture.ySim(:,4)))/ ...
    max(abs(aileronCapture.openStates(:,4)));
rudderYawIntegralRatio = ...
    trapz(rudderCapture.t,abs(rudderCapture.ySim(:,3)))/ ...
    trapz(rudderCapture.t,abs(rudderCapture.openStates(:,3)));
rudderBetaPeakRatio = max(abs(rudderCapture.ySim(:,1)))/ ...
    max(abs(rudderCapture.openStates(:,1)));
peakPulseAutomaticRudder_deg = max([ ...
    max(abs(rad2deg(aileronCapture.ySim(:,9)))), ...
    max(abs(rad2deg(rudderCapture.ySim(:,9))))]);

assert(initialYawRateIntegralRatio <= ...
    req.maximumInitialYawRateIntegralRatio, ...
    'The Simulink initial-yaw-rate attenuation gate failed.');
assert(initialBetaIntegralRatio <= ...
    req.maximumInitialBetaIntegralRatio, ...
    'The Simulink initial-sideslip attenuation gate failed.');
assert(peakInitialAutomaticRudder_deg <= ...
    req.maximumInitialDisturbanceRudder_deg, ...
    'The Simulink initial-disturbance rudder-demand gate failed.');
assert(aileronYawIntegralRatio <= req.maximumAileronYawIntegralRatio, ...
    'The Simulink aileron-induced yaw attenuation gate failed.');
assert(aileronBankPeakRatio >= req.minimumAileronBankPeakRatio & ...
    aileronBankPeakRatio <= req.maximumAileronBankPeakRatio, ...
    'The Simulink aileron bank-response preservation gate failed.');
assert(rudderYawIntegralRatio <= req.maximumRudderYawIntegralRatio, ...
    'The Simulink rudder-pulse yaw attenuation gate failed.');
assert(rudderBetaPeakRatio <= req.maximumRudderBetaPeakRatio, ...
    'The Simulink rudder-pulse sideslip attenuation gate failed.');
assert(peakPulseAutomaticRudder_deg <= ...
    req.maximumPulseAutomaticRudder_deg, ...
    'The Simulink pulse automatic-rudder demand gate failed.');

% Sign, summation and washout-return checks.
assert(initialCapture.ySim(1,9) < 0, ...
    'Positive initial yaw rate did not command negative rudder.');
assert(max(aileronCapture.ySim(:,4)) > 0, ...
    'Positive aileron did not produce positive right-wing-down bank.');
assert(max(rudderCapture.ySim(:,3)) > 0, ...
    'Positive rudder did not produce positive nose-right yaw rate.');
for capture = {initialCapture,aileronCapture,rudderCapture}
    y = capture{1}.ySim;
    summationError = max(abs(y(:,10) - (y(:,7) + y(:,9))));
    assert(summationError < 1.0e-12, ...
        'Pilot plus automatic rudder does not equal total rudder.');
end
assert(abs(rad2deg(initialCapture.ySim(end,9))) < 0.01, ...
    'The washout automatic-rudder demand did not return near zero.');

%% Nominal modal report

Aclosed = yawDamperClosedLoopMatrix(plant, ...
    controller.rudderGain_s,controller.washoutTimeConstant_s);
modes = classifyYawDamperModes(Aclosed);

fprintf('============================================================\n');
fprintf('       C172S YAW DAMPER V1 - SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                               : %s\n',modelName);
fprintf('Artifact revision                   : %s\n', ...
    controller.artifactRevision);
fprintf('Frozen plant                        : %s\n', ...
    controller.requiredPlantVersion);
fprintf('Control law                         : deltaR_YD = -Kr*(r-xw)\n');
fprintf('Kr / Tw                             : %.3f s / %.2f s\n', ...
    controller.rudderGain_s,controller.washoutTimeConstant_s);
fprintf('Solver / fixed step                 : ode4 / 0.0100 s\n\n');

fprintf('Initial maximum signal error        : %.9g\n', ...
    initialCapture.maxError);
fprintf('Initial RMS signal error            : %.9g\n', ...
    initialCapture.rmsError);
fprintf('Aileron maximum signal error        : %.9g\n', ...
    aileronCapture.maxError);
fprintf('Aileron RMS signal error            : %.9g\n', ...
    aileronCapture.rmsError);
fprintf('Rudder maximum signal error         : %.9g\n', ...
    rudderCapture.maxError);
fprintf('Rudder RMS signal error             : %.9g\n', ...
    rudderCapture.rmsError);
fprintf('Allowed error                       : %.3g\n\n',tolerance);

fprintf('Closed Dutch-roll poles             : %.6f +/- %.6fi 1/s\n', ...
    real(modes.dutchPole),abs(imag(modes.dutchPole)));
fprintf('Closed Dutch damping                : %.4f\n', ...
    modes.dutchDampingRatio);
fprintf('Roll / spiral / washout poles       : %.6f / %.8f / %.6f 1/s\n\n', ...
    modes.rollPole,modes.spiralPole,modes.washoutPole);

fprintf('Initial-r integral ratio            : %.4f\n', ...
    initialYawRateIntegralRatio);
fprintf('Initial-beta integral ratio         : %.4f\n', ...
    initialBetaIntegralRatio);
fprintf('Peak initial automatic rudder       : %.4f deg\n', ...
    peakInitialAutomaticRudder_deg);
fprintf('Aileron yaw integral ratio          : %.4f\n', ...
    aileronYawIntegralRatio);
fprintf('Aileron bank peak ratio             : %.4f\n', ...
    aileronBankPeakRatio);
fprintf('Rudder yaw integral ratio           : %.4f\n', ...
    rudderYawIntegralRatio);
fprintf('Rudder beta peak ratio              : %.4f\n', ...
    rudderBetaPeakRatio);
fprintf('Peak pulse automatic rudder         : %.4f deg\n\n', ...
    peakPulseAutomaticRudder_deg);

fprintf('PASS: Simulink matches three independent stage-aware RK4 references.\n');
fprintf('PASS: yaw-feedback, pilot-input and rudder-summation signs are correct.\n');
fprintf('PASS: attenuation, bank-preservation and rudder-demand gates pass.\n');
fprintf('PASS: the washout releases the automatic rudder toward zero.\n');
fprintf(['NOTE: this validates implementation equivalence at Plant V0.1; ', ...
    'actuator, sensor, delay, gust and full-envelope work remain open.\n']);

%% Equivalence figures

figure('Name','C172S Yaw Damper Simulink Equivalence','Color','w');
tiledlayout(2,2);

nexttile;
plotComparison(initialCapture.t,initialCapture.ySim(:,3), ...
    initialCapture.yReference(:,3),'Yaw rate (deg/s)');
title('Initial Yaw-Rate Capture');

nexttile;
plotComparison(initialCapture.t,initialCapture.ySim(:,9), ...
    initialCapture.yReference(:,9),'Automatic rudder (deg)');
title('Washout Rudder Demand');

nexttile;
plotComparison(aileronCapture.t,aileronCapture.ySim(:,4), ...
    aileronCapture.yReference(:,4),'Bank angle (deg)');
title('Aileron Capture');

nexttile;
plotComparison(rudderCapture.t,rudderCapture.ySim(:,1), ...
    rudderCapture.yReference(:,1),'Sideslip (deg)');
title('Rudder Capture');

sgtitle('Yaw Damper V1: Simulink vs Independent RK4');

function result = runCapture(modelName,plant,controller,x0, ...
    deltaA_rad,deltaR_rad)
assignin('base','deltaACommand_yd1_rad',deltaA_rad);
assignin('base','deltaRCommand_yd1_rad',deltaR_rad);
assignin('base','x0_yd1',x0);
assignin('base','xw0_yd1',0.0);

simOut = sim(modelName,'StopTime','30','ReturnWorkspaceOutputs','on');
tSim = simOut.tout;
ySim = extractArrayOutput(simOut.yout,tSim);
if size(ySim,2) ~= 10
    error('runYawDamperSimulinkCheck:UnexpectedOutputWidth', ...
        'Expected ten output columns; received %d.',size(ySim,2));
end

dt = diff(tSim);
nominalDt = median(dt);
assert(max(abs(dt - nominalDt)) < 1.0e-10, ...
    'The independent RK4 reference requires uniform sampling.');

[yReference,openStates] = independentReference(tSim,plant,controller, ...
    x0,deltaA_rad,deltaR_rad);
errorSignal = ySim - yReference;

result.t = tSim;
result.ySim = ySim;
result.yReference = yReference;
result.openStates = openStates;
result.maxError = max(abs(errorSignal(:)));
result.rmsError = sqrt(mean(errorSignal(:).^2));
end

function [yReference,openStates] = independentReference(t,plant, ...
    controller,x0,deltaA_rad,deltaR_rad)
Kr = controller.rudderGain_s;
Tw = controller.washoutTimeConstant_s;
Aclosed = yawDamperClosedLoopMatrix(plant,Kr,Tw);
Bclosed = [plant.B;zeros(1,2)];

xClosed = [x0;0.0];
xOpen = x0;
yReference = zeros(numel(t),10);
openStates = zeros(numel(t),5);

closedDerivative = @(state,input) Aclosed*state + Bclosed*input;
openDerivative = @(state,input) plant.A*state + plant.B*input;

for k = 1:numel(t)
    pilotInput = pulseInput(t(k),deltaA_rad,deltaR_rad);
    washoutYawRate = xClosed(3) - xClosed(6);
    automaticRudder = -Kr*washoutYawRate;
    totalRudder = pilotInput(2) + automaticRudder;
    yReference(k,:) = [xClosed(1:5).',pilotInput.', ...
        washoutYawRate,automaticRudder,totalRudder];
    openStates(k,:) = xOpen.';

    if k < numel(t)
        t0 = t(k);
        h = t(k + 1) - t0;
        u1 = pulseInput(t0,deltaA_rad,deltaR_rad);
        u2 = pulseInput(t0 + 0.5*h,deltaA_rad,deltaR_rad);
        u4 = pulseInput(t0 + h,deltaA_rad,deltaR_rad);

        k1 = closedDerivative(xClosed,u1);
        k2 = closedDerivative(xClosed + 0.5*h*k1,u2);
        k3 = closedDerivative(xClosed + 0.5*h*k2,u2);
        k4 = closedDerivative(xClosed + h*k3,u4);
        xClosed = xClosed + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);

        k1 = openDerivative(xOpen,u1);
        k2 = openDerivative(xOpen + 0.5*h*k1,u2);
        k3 = openDerivative(xOpen + 0.5*h*k2,u2);
        k4 = openDerivative(xOpen + h*k3,u4);
        xOpen = xOpen + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    end
end
end

function input = pulseInput(t,deltaA_rad,deltaR_rad)
edges = [0.0 1.0 5.0 6.0];
for k = 1:numel(edges)
    if abs(t - edges(k)) <= 1.0e-12
        t = edges(k);
        break;
    end
end

input = [0.0;0.0];
if t >= 0.0 && t < 1.0
    input(1) = deltaA_rad;
end
if t >= 5.0 && t < 6.0
    input(2) = deltaR_rad;
end
end

function Aclosed = yawDamperClosedLoopMatrix(plant,Kr,Tw)
yawRateOutput = [0 0 1 0 0];
rudderColumn = plant.B(:,2);
Aclosed = zeros(6,6);
Aclosed(1:5,1:5) = plant.A - rudderColumn*Kr*yawRateOutput;
Aclosed(1:5,6) = rudderColumn*Kr;
Aclosed(6,1:5) = yawRateOutput/Tw;
Aclosed(6,6) = -1/Tw;
end

function modes = classifyYawDamperModes(Aclosed)
poles = eig(Aclosed);
[~,headingIndex] = min(abs(poles));
dynamicPoles = poles;
dynamicPoles(headingIndex) = [];
assert(all(real(dynamicPoles) < -1.0e-10), ...
    'The nominal Simulink-equivalent closed-loop dynamics are unstable.');

positiveComplex = dynamicPoles(imag(dynamicPoles) > 1.0e-8);
assert(numel(positiveComplex) == 1, ...
    'Expected exactly one Dutch-roll pole pair.');
modes.dutchPole = positiveComplex(1);
modes.dutchNaturalFrequency = abs(modes.dutchPole);
modes.dutchDampingRatio = ...
    -real(modes.dutchPole)/modes.dutchNaturalFrequency;

realPoles = real(dynamicPoles(abs(imag(dynamicPoles)) <= 1.0e-8));
assert(numel(realPoles) == 3, ...
    'Expected roll, spiral and washout real modes.');
[~,rollIndex] = max(abs(realPoles));
modes.rollPole = realPoles(rollIndex);
realPoles(rollIndex) = [];
[~,spiralIndex] = min(abs(realPoles));
modes.spiralPole = realPoles(spiralIndex);
realPoles(spiralIndex) = [];
modes.washoutPole = realPoles(1);
end

function y = extractArrayOutput(yOut,t)
if isa(yOut,'timeseries')
    y = yOut.Data;
elseif isa(yOut,'Simulink.SimulationData.Dataset')
    y = yOut.getElement(1).Values.Data;
else
    y = yOut;
end

y = squeeze(y);
if size(y,1) ~= numel(t) && size(y,2) == numel(t)
    y = y.';
end
end

function plotComparison(t,ySim,yReference,yLabel)
plot(t,rad2deg(ySim),'LineWidth',1.3);
hold on;
plot(t,rad2deg(yReference),'--','LineWidth',1.0);
grid on;
xlabel('Time (s)');
ylabel(yLabel);
legend('Simulink','Independent RK4','Location','best');
end
