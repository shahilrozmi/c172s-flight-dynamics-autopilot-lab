%% C172S Yaw Damper V2 - protected Simulink equivalence check

clc;
clear;
close all;

try
    load_system('simulink');
catch ME
    error('runYawDamperActuatorSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

data = c172sLateralDirectionalData();
[~,plant] = c172sLateralDirectionalPlant(data);
controller = c172sYawDamperData();
actuator = c172sRudderActuatorData();
assertDependencies(data,controller,actuator);

% Rebuild from a clean diagram so a stale or interrupted SLX cannot
% influence the implementation-equivalence result.
modelName = buildC172sYawDamperActuatorSimulink(true);

initialCapture = runCapture(modelName,plant,controller,actuator, ...
    'initial',controller.validation.initialYawRate_degps,0.0);
aileronCapture = runCapture(modelName,plant,controller,actuator, ...
    'aileron',0.0,0.0);
rudderCapture = runCapture(modelName,plant,controller,actuator, ...
    'rudder',0.0,0.0);
stressCapture = runCapture(modelName,plant,controller,actuator, ...
    'stress',actuator.stress.initialYawRate_degps,0.0);

% A deterministic sinusoid supplies a reproducible sensor-path check. Its
% amplitude is sqrt(2)*RMS so the commanded noise has the stated RMS.
noiseFrequency_radps = 50.0;
noiseCapture = runCapture(modelName,plant,controller,actuator, ...
    'noise',0.0,actuator.sensor.noiseStressRms_degps, ...
    noiseFrequency_radps);

%% Independent hybrid-equivalence gates

tolerance = 1.0e-5;
captureNames = {'Initial','Aileron','Rudder','Stress','Noise'};
captures = {initialCapture,aileronCapture,rudderCapture, ...
    stressCapture,noiseCapture};
for k = 1:numel(captures)
    assert(captures{k}.maxError < tolerance, ...
        '%s-capture error %.6g exceeds %.6g.', ...
        captureNames{k},captures{k}.maxError,tolerance);
end

assert(strcmp(get_param(modelName,'SolverType'),'Fixed-step'), ...
    'The generated model must use a fixed-step solver.');
assert(strcmp(get_param(modelName,'Solver'),'ode4'), ...
    'The generated model must use ode4.');
assert(abs(str2double(get_param(modelName,'FixedStep')) - 0.01) < eps, ...
    'The generated model fixed step must be 0.01 s.');
assert(strcmp(get_param([modelName '/Nominal Pure Delay'],'SampleTime'), ...
    'delaySampleTime_yd2'), ...
    'The nominal pure-delay sample time changed.');
assert(strcmp(get_param([modelName '/Yaw-Rate Sensor Filter'],'A'), ...
    '-1/Ts_yd2'),'The sensor-filter state equation changed.');
assert(strcmp(get_param([modelName '/Yaw-Rate Washout'],'A'), ...
    '-1/Tw_yd2'),'The washout state equation changed.');
assert(strcmp(get_param([modelName '/Automatic Rudder Gain'],'Gain'), ...
    '-Kr_yd2'),'The automatic-rudder sign or gain changed.');
assert(strcmp(get_param([modelName '/Automatic Rudder Position Limit'], ...
    'UpperLimit'),'autoRudderUpper_yd2_rad'), ...
    'The automatic-rudder authority changed.');
assert(strcmp(get_param([modelName '/Servo Rate Limit'],'UpperLimit'), ...
    'rateUpper_yd2_radps'),'The servo rate limit changed.');
assert(strcmp(get_param([modelName '/Physical Rudder Hard Stop'], ...
    'UpperLimit'),'totalRudderUpper_yd2_rad'), ...
    'The total-rudder hard stop changed.');

%% Operational, protection and noise gates

req = actuator.requirements;
initialYawRateIntegralRatio = integralRatio( ...
    initialCapture.t,initialCapture.ySim(:,3), ...
    initialCapture.openStates(:,3));
initialBetaIntegralRatio = integralRatio( ...
    initialCapture.t,initialCapture.ySim(:,1), ...
    initialCapture.openStates(:,1));
peakInitialPosition_deg = max(abs(rad2deg(initialCapture.ySim(:,15))));
peakInitialRate_degps = max(abs(rad2deg(initialCapture.ySim(:,17))));

aileronYawIntegralRatio = integralRatio( ...
    aileronCapture.t,aileronCapture.ySim(:,3), ...
    aileronCapture.openStates(:,3));
aileronBankPeakRatio = max(abs(aileronCapture.ySim(:,4)))/ ...
    max(abs(aileronCapture.openStates(:,4)));
rudderYawIntegralRatio = integralRatio( ...
    rudderCapture.t,rudderCapture.ySim(:,3), ...
    rudderCapture.openStates(:,3));
rudderBetaPeakRatio = max(abs(rudderCapture.ySim(:,1)))/ ...
    max(abs(rudderCapture.openStates(:,1)));
peakPulsePosition_deg = max([ ...
    max(abs(rad2deg(aileronCapture.ySim(:,15)))), ...
    max(abs(rad2deg(rudderCapture.ySim(:,15))))]);
peakPulseRate_degps = max([ ...
    max(abs(rad2deg(aileronCapture.ySim(:,17)))), ...
    max(abs(rad2deg(rudderCapture.ySim(:,17))))]);

assert(initialYawRateIntegralRatio <= ...
    req.maximumInitialYawRateIntegralRatio, ...
    'The initial-yaw-rate attenuation gate failed.');
assert(initialBetaIntegralRatio <= req.maximumInitialBetaIntegralRatio, ...
    'The initial-sideslip attenuation gate failed.');
assert(peakInitialPosition_deg <= ...
    req.maximumInitialActuatorPosition_deg, ...
    'The initial-disturbance actuator-position gate failed.');
assert(peakInitialRate_degps <= req.maximumInitialActuatorRate_degps + 1e-9, ...
    'The initial-disturbance actuator-rate gate failed.');
assert(aileronYawIntegralRatio <= req.maximumAileronYawIntegralRatio, ...
    'The aileron-induced yaw attenuation gate failed.');
assert(aileronBankPeakRatio >= req.minimumAileronBankPeakRatio && ...
    aileronBankPeakRatio <= req.maximumAileronBankPeakRatio, ...
    'The aileron bank-response preservation gate failed.');
assert(rudderYawIntegralRatio <= req.maximumRudderYawIntegralRatio, ...
    'The rudder-pulse yaw attenuation gate failed.');
assert(rudderBetaPeakRatio <= req.maximumRudderBetaPeakRatio, ...
    'The rudder-pulse sideslip attenuation gate failed.');
assert(peakPulsePosition_deg <= req.maximumPulseActuatorPosition_deg, ...
    'The operational-pulse actuator-position gate failed.');
assert(peakPulseRate_degps <= req.maximumPulseActuatorRate_degps, ...
    'The operational-pulse actuator-rate gate failed.');

operationalCaptures = {initialCapture,aileronCapture,rudderCapture};
for k = 1:numel(operationalCaptures)
    y = operationalCaptures{k}.ySim;
    assert(~positionLimitActive(y,actuator), ...
        'Position protection unexpectedly activated operationally.');
    assert(~rateLimitActive(y), ...
        'Rate protection unexpectedly activated operationally.');
end

stressPositionActive = positionLimitActive(stressCapture.ySim,actuator);
stressRateActive = rateLimitActive(stressCapture.ySim);
stressPhysicalActive = physicalLimitActive(stressCapture.ySim);
stressPeakPosition_deg = max(abs(rad2deg(stressCapture.ySim(:,15))));
stressPeakRate_degps = max(abs(rad2deg(stressCapture.ySim(:,17))));
stressFinalYawRate_degps = rad2deg(stressCapture.ySim(end,3));
stressFinalAutomaticRudder_deg = rad2deg(stressCapture.ySim(end,15));

assert(stressPositionActive, ...
    'The 25-deg/s logic stress did not activate position limiting.');
assert(stressRateActive, ...
    'The 25-deg/s logic stress did not activate rate limiting.');
assert(stressPeakPosition_deg <= actuator.authority.right_deg + 1e-9, ...
    'The stress case exceeded automatic-rudder authority.');
assert(stressPeakRate_degps <= actuator.dynamics.rateLimit_degps + 1e-9, ...
    'The stress case exceeded the servo rate limit.');
assert(max(abs(rad2deg(stressCapture.ySim(:,19)))) <= ...
    actuator.physicalTravel.right_deg + 1e-9, ...
    'The stress case exceeded the total-rudder hard stop.');
assert(abs(stressFinalYawRate_degps) <= ...
    actuator.stress.maximumFinalYawRate_degps, ...
    'The stress yaw rate did not substantially recover.');
assert(abs(stressFinalAutomaticRudder_deg) <= ...
    req.maximumReleasedRudder_deg, ...
    'The washout did not release the stress-case automatic rudder.');

noiseRetained = noiseCapture.t >= 5.0;
noiseActuator_deg = rad2deg(noiseCapture.ySim(noiseRetained,15));
noiseActuatorRms_deg = sqrt(mean(noiseActuator_deg.^2));
noiseActuatorPeak_deg = max(abs(noiseActuator_deg));
noiseInputRms_degps = sqrt(mean(rad2deg( ...
    noiseCapture.ySim(noiseRetained,9)).^2));
assert(noiseActuatorRms_deg <= req.maximumNoiseActuatorRms_deg, ...
    'Deterministic stress noise caused excessive RMS rudder activity.');
assert(noiseActuatorPeak_deg <= req.maximumNoiseActuatorPeak_deg, ...
    'Deterministic stress noise caused excessive peak rudder activity.');
assert(~positionLimitActive(noiseCapture.ySim,actuator) && ...
    ~rateLimitActive(noiseCapture.ySim), ...
    'Deterministic sensor noise unexpectedly activated protection.');

% Explicit sign, summation and limiter identities.
assert(min(initialCapture.ySim(:,13)) < -1.0e-6, ...
    'Positive initial yaw rate did not command negative rudder.');
assert(max(aileronCapture.ySim(:,4)) > 0, ...
    'Positive aileron did not produce positive right-wing-down bank.');
assert(max(rudderCapture.ySim(:,3)) > 0, ...
    'Positive rudder did not produce positive nose-right yaw rate.');
for k = 1:numel(captures)
    y = captures{k}.ySim;
    assert(max(abs(y(:,10) - (y(:,8) + y(:,9)))) < 1.0e-12, ...
        'Delayed yaw rate plus noise does not equal sensor input.');
    assert(max(abs(y(:,18) - (y(:,7) + y(:,15)))) < 1.0e-12, ...
        'Pilot plus automatic rudder does not equal raw total rudder.');
    assert(max(abs(y(:,14) - clip(y(:,13), ...
        actuator.authority.lowerLimit_rad, ...
        actuator.authority.upperLimit_rad))) < 1.0e-12, ...
        'Automatic-rudder position limiting is inconsistent.');
    assert(max(abs(y(:,17) - clip(y(:,16), ...
        -deg2rad(actuator.dynamics.rateLimit_degps), ...
        deg2rad(actuator.dynamics.rateLimit_degps)))) < 1.0e-12, ...
        'Servo rate limiting is inconsistent.');
    assert(max(abs(y(:,19) - clip(y(:,18), ...
        actuator.physicalTravel.lowerLimit_rad, ...
        actuator.physicalTravel.upperLimit_rad))) < 1.0e-12, ...
        'Total-rudder physical limiting is inconsistent.');
end

assert(abs(actuator.sensor.nominalDelay_s - 0.010) < 1.0e-12, ...
    'The selected V2 baseline is not the audited 10-ms nominal delay.');
assert(actuator.sensor.nominalDelay_s <= ...
    actuator.sensor.maximumAcceptedDelay_s, ...
    'The selected nominal delay exceeds the accepted delay envelope.');

%% Release report

fprintf('============================================================\n');
fprintf(' C172S YAW DAMPER V2 - PROTECTED SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                                : %s\n',modelName);
fprintf('Frozen controller artifact           : %s\n', ...
    controller.artifactRevision);
fprintf('Actuator/source audit                : %s\n', ...
    actuator.artifactRevision);
fprintf('Frozen plant                         : %s\n', ...
    actuator.requiredPlantVersion);
fprintf('Control law                          : deltaR = -Kr*(rSensor-xw)\n');
fprintf('Kr / Tw                              : %.3f s / %.2f s\n', ...
    controller.rudderGain_s,controller.washoutTimeConstant_s);
fprintf('Sensor filter / nominal pure delay   : %.3f s / %.0f ms\n', ...
    actuator.sensor.filterTimeConstant_s, ...
    1000*actuator.sensor.nominalDelay_s);
fprintf('Actuator tau / authority / rate      : %.3f s / +/-%.1f deg / +/-%.1f deg/s\n', ...
    actuator.dynamics.timeConstant_s,actuator.authority.right_deg, ...
    actuator.dynamics.rateLimit_degps);
fprintf('Total-rudder hard stop               : +/-%.1f deg\n', ...
    actuator.physicalTravel.right_deg);
fprintf('Solver / fixed step                  : ode4 / 0.0100 s\n\n');

for k = 1:numel(captures)
    fprintf('%-20s maximum error       : %.9g\n', ...
        captureNames{k},captures{k}.maxError);
    fprintf('%-20s RMS error           : %.9g\n', ...
        captureNames{k},captures{k}.rmsError);
end
fprintf('Allowed implementation error         : %.3g\n\n',tolerance);

fprintf('Initial-r integral ratio             : %.4f\n', ...
    initialYawRateIntegralRatio);
fprintf('Initial-beta integral ratio          : %.4f\n', ...
    initialBetaIntegralRatio);
fprintf('Peak initial actuator position       : %.4f deg\n', ...
    peakInitialPosition_deg);
fprintf('Peak initial actuator rate           : %.4f deg/s\n', ...
    peakInitialRate_degps);
fprintf('Aileron yaw / bank ratios            : %.4f / %.4f\n', ...
    aileronYawIntegralRatio,aileronBankPeakRatio);
fprintf('Rudder yaw / beta ratios             : %.4f / %.4f\n', ...
    rudderYawIntegralRatio,rudderBetaPeakRatio);
fprintf('Peak operational position / rate     : %.4f deg / %.4f deg/s\n\n', ...
    peakPulsePosition_deg,peakPulseRate_degps);

fprintf('Stress initial yaw rate              : %.1f deg/s\n', ...
    actuator.stress.initialYawRate_degps);
fprintf('Stress peak automatic rudder         : %.4f deg\n', ...
    stressPeakPosition_deg);
fprintf('Stress peak actuator rate            : %.4f deg/s\n', ...
    stressPeakRate_degps);
fprintf('Stress position/rate limits active   : %d / %d\n', ...
    stressPositionActive,stressRateActive);
fprintf('Stress physical hard stop active     : %d\n',stressPhysicalActive);
fprintf('Stress final yaw rate                : %.6f deg/s\n', ...
    stressFinalYawRate_degps);
fprintf('Stress final automatic rudder        : %.6f deg\n\n', ...
    stressFinalAutomaticRudder_deg);

fprintf('Deterministic sensor-noise RMS       : %.5f deg/s\n', ...
    noiseInputRms_degps);
fprintf('Noise-driven automatic-rudder RMS    : %.5f deg\n', ...
    noiseActuatorRms_deg);
fprintf('Noise-driven automatic-rudder peak   : %.5f deg\n\n', ...
    noiseActuatorPeak_deg);

fprintf('PASS: Simulink matches five independent hybrid RK4 references.\n');
fprintf('PASS: sensor delay/filter, feedback and rudder summation are correct.\n');
fprintf('PASS: operational attenuation and control-interference gates pass.\n');
fprintf('PASS: position, rate and physical protection enforce their limits.\n');
fprintf('PASS: the 25-deg/s stress activates both automatic-rudder protections.\n');
fprintf('PASS: deterministic sensor-noise amplification remains below its gates.\n');
fprintf(['NOTE: this validates the nominal executable V2 implementation against ', ...
    'the audited Plant V0.1 assumptions; it is not aircraft approval.\n']);

%% Equivalence figures

figure('Name','C172S Yaw Damper V2 Simulink Equivalence','Color','w');
tiledlayout(2,3);

nexttile;
plotComparison(initialCapture.t,initialCapture.ySim(:,3), ...
    initialCapture.yReference(:,3),'Yaw rate (deg/s)');
title('5-deg/s Initial Capture');

nexttile;
plotComparison(initialCapture.t,initialCapture.ySim(:,15), ...
    initialCapture.yReference(:,15),'Automatic rudder (deg)');
title('Protected Actuator Position');

nexttile;
plotComparison(aileronCapture.t,aileronCapture.ySim(:,4), ...
    aileronCapture.yReference(:,4),'Bank angle (deg)');
title('Aileron Pulse');

nexttile;
plotComparison(rudderCapture.t,rudderCapture.ySim(:,1), ...
    rudderCapture.yReference(:,1),'Sideslip (deg)');
title('Rudder Pulse');

nexttile;
plotComparison(stressCapture.t,stressCapture.ySim(:,17), ...
    stressCapture.yReference(:,17),'Servo rate (deg/s)');
title('25-deg/s Logic Stress');

nexttile;
plotComparison(noiseCapture.t,noiseCapture.ySim(:,15), ...
    noiseCapture.yReference(:,15),'Automatic rudder (deg)');
title('Deterministic Sensor Noise');

sgtitle('Yaw Damper V2: Simulink vs Independent Hybrid RK4');

function result = runCapture(modelName,plant,controller,actuator, ...
    caseName,initialYawRate_degps,noiseRms_degps,noiseFrequency_radps)
if nargin < 8
    noiseFrequency_radps = 50.0;
end

x0 = zeros(5,1);
x0(3) = deg2rad(initialYawRate_degps);
deltaA_rad = 0.0;
deltaR_rad = 0.0;
if strcmp(caseName,'aileron')
    deltaA_rad = deg2rad(controller.validation.aileronPulse_deg);
elseif strcmp(caseName,'rudder')
    deltaR_rad = deg2rad(controller.validation.rudderPulse_deg);
end
noiseAmplitude_radps = sqrt(2.0)*deg2rad(noiseRms_degps);

assignin('base','deltaACommand_yd2_rad',deltaA_rad);
assignin('base','deltaRCommand_yd2_rad',deltaR_rad);
assignin('base','noiseAmplitude_yd2_radps',noiseAmplitude_radps);
assignin('base','noiseFrequency_yd2_radps',noiseFrequency_radps);
assignin('base','x0_yd2',x0);
% The delay history equals the initial yaw rate, representing a disturbance
% already present immediately before t=0 without an artificial delay jump.
assignin('base','rDelay0_yd2_radps',x0(3));
assignin('base','rSensor0_yd2_radps',0.0);
assignin('base','xw0_yd2_radps',0.0);
assignin('base','deltaRAuto0_yd2_rad',0.0);

simOut = sim(modelName,'StopTime','30','ReturnWorkspaceOutputs','on');
tSim = simOut.tout;
ySim = extractArrayOutput(simOut.yout,tSim);
if size(ySim,2) ~= 19
    error('runYawDamperActuatorSimulinkCheck:UnexpectedOutputWidth', ...
        'Expected 19 output columns; received %d.',size(ySim,2));
end

dt = diff(tSim);
nominalDt = median(dt);
assert(max(abs(dt - nominalDt)) < 1.0e-10, ...
    'The independent RK4 reference requires uniform sampling.');
assert(abs(nominalDt - actuator.validation.timeStep_s) < 1.0e-12, ...
    'The Simulink sample grid differs from the V2 validation grid.');

[yReference,openStates] = independentReference(tSim,plant,controller, ...
    actuator,x0,deltaA_rad,deltaR_rad,noiseAmplitude_radps, ...
    noiseFrequency_radps);
errorSignal = ySim - yReference;

result.t = tSim;
result.ySim = ySim;
result.yReference = yReference;
result.openStates = openStates;
result.maxError = max(abs(errorSignal(:)));
result.rmsError = sqrt(mean(errorSignal(:).^2));
end

function [yReference,openStates] = independentReference(t,plant, ...
    controller,actuator,x0,deltaA_rad,deltaR_rad, ...
    noiseAmplitude_radps,noiseFrequency_radps)
% Continuous state z = [beta p r phi psi rSensor xw deltaRActual].
z = [x0;0.0;0.0;0.0];
xOpen = x0;
delayState = x0(3);
yReference = zeros(numel(t),19);
openStates = zeros(numel(t),5);

for k = 1:numel(t)
    pilot = pulseInput(t(k),deltaA_rad,deltaR_rad,controller);
    noise = noiseAmplitude_radps*sin(noiseFrequency_radps*t(k));
    diagnostic = protectedSignals(z,pilot,delayState,noise, ...
        controller,actuator);
    yReference(k,:) = [z(1:5).',pilot.',delayState,noise, ...
        diagnostic.sensorInput,z(6),diagnostic.washoutYawRate, ...
        diagnostic.rawCommand,diagnostic.positionCommand,z(8), ...
        diagnostic.rawRate,diagnostic.limitedRate, ...
        diagnostic.rawTotalRudder,diagnostic.totalRudder];
    openStates(k,:) = xOpen.';

    if k < numel(t)
        t0 = t(k);
        h = t(k + 1) - t0;
        sampledYawRate = z(3);

        k1 = protectedDerivative(t0,z,delayState,plant,controller, ...
            actuator,deltaA_rad,deltaR_rad,noiseAmplitude_radps, ...
            noiseFrequency_radps);
        k2 = protectedDerivative(t0 + 0.5*h,z + 0.5*h*k1, ...
            delayState,plant,controller,actuator,deltaA_rad,deltaR_rad, ...
            noiseAmplitude_radps,noiseFrequency_radps);
        k3 = protectedDerivative(t0 + 0.5*h,z + 0.5*h*k2, ...
            delayState,plant,controller,actuator,deltaA_rad,deltaR_rad, ...
            noiseAmplitude_radps,noiseFrequency_radps);
        k4 = protectedDerivative(t0 + h,z + h*k3,delayState,plant, ...
            controller,actuator,deltaA_rad,deltaR_rad, ...
            noiseAmplitude_radps,noiseFrequency_radps);
        z = z + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);

        u1 = pulseInput(t0,deltaA_rad,deltaR_rad,controller);
        u2 = pulseInput(t0 + 0.5*h,deltaA_rad,deltaR_rad,controller);
        u4 = pulseInput(t0 + h,deltaA_rad,deltaR_rad,controller);
        q1 = plant.A*xOpen + plant.B*u1;
        q2 = plant.A*(xOpen + 0.5*h*q1) + plant.B*u2;
        q3 = plant.A*(xOpen + 0.5*h*q2) + plant.B*u2;
        q4 = plant.A*(xOpen + h*q3) + plant.B*u4;
        xOpen = xOpen + (h/6.0)*(q1 + 2*q2 + 2*q3 + q4);

        % Unit Delay output at t(k+1) is the yaw rate sampled at t(k).
        delayState = sampledYawRate;
    end
end
end

function zDot = protectedDerivative(t,z,delayState,plant,controller, ...
    actuator,deltaA_rad,deltaR_rad,noiseAmplitude_radps, ...
    noiseFrequency_radps)
pilot = pulseInput(t,deltaA_rad,deltaR_rad,controller);
noise = noiseAmplitude_radps*sin(noiseFrequency_radps*t);
diagnostic = protectedSignals(z,pilot,delayState,noise, ...
    controller,actuator);

zDot = zeros(8,1);
zDot(1:5) = plant.A*z(1:5) + ...
    plant.B*[pilot(1);diagnostic.totalRudder];
zDot(6) = (diagnostic.sensorInput - z(6))/ ...
    actuator.sensor.filterTimeConstant_s;
zDot(7) = (z(6) - z(7))/controller.washoutTimeConstant_s;
zDot(8) = diagnostic.limitedRate;
end

function diagnostic = protectedSignals(z,pilot,delayState,noise, ...
    controller,actuator)
diagnostic.sensorInput = delayState + noise;
diagnostic.washoutYawRate = z(6) - z(7);
diagnostic.rawCommand = ...
    -controller.rudderGain_s*diagnostic.washoutYawRate;
diagnostic.positionCommand = clip(diagnostic.rawCommand, ...
    actuator.authority.lowerLimit_rad,actuator.authority.upperLimit_rad);
diagnostic.rawRate = (diagnostic.positionCommand - z(8))/ ...
    actuator.dynamics.timeConstant_s;
diagnostic.limitedRate = clip(diagnostic.rawRate, ...
    -deg2rad(actuator.dynamics.rateLimit_degps), ...
    deg2rad(actuator.dynamics.rateLimit_degps));
diagnostic.rawTotalRudder = pilot(2) + z(8);
diagnostic.totalRudder = clip(diagnostic.rawTotalRudder, ...
    actuator.physicalTravel.lowerLimit_rad, ...
    actuator.physicalTravel.upperLimit_rad);
end

function input = pulseInput(t,deltaA_rad,deltaR_rad,controller)
edges = [controller.validation.aileronPulseStart_s, ...
    controller.validation.aileronPulseEnd_s, ...
    controller.validation.rudderPulseStart_s, ...
    controller.validation.rudderPulseEnd_s];
for k = 1:numel(edges)
    if abs(t - edges(k)) <= 1.0e-12
        t = edges(k);
        break;
    end
end

input = [0.0;0.0];
if t >= controller.validation.aileronPulseStart_s && ...
        t < controller.validation.aileronPulseEnd_s
    input(1) = deltaA_rad;
end
if t >= controller.validation.rudderPulseStart_s && ...
        t < controller.validation.rudderPulseEnd_s
    input(2) = deltaR_rad;
end
end

function active = positionLimitActive(y,actuator)
active = any(abs(y(:,13) - y(:,14)) > 1.0e-10) || ...
    any(abs(y(:,13)) > actuator.authority.upperLimit_rad + 1.0e-10);
end

function active = rateLimitActive(y)
active = any(abs(y(:,16) - y(:,17)) > 1.0e-10);
end

function active = physicalLimitActive(y)
active = any(abs(y(:,18) - y(:,19)) > 1.0e-10);
end

function ratio = integralRatio(t,closedSignal,openSignal)
denominator = trapz(t,abs(openSignal));
assert(denominator > 1.0e-12, ...
    'An operational reference integral is numerically zero.');
ratio = trapz(t,abs(closedSignal))/denominator;
end

function value = clip(value,lowerLimit,upperLimit)
value = min(max(value,lowerLimit),upperLimit);
end

function assertDependencies(data,controller,actuator)
assert(strcmp(data.model.version,actuator.requiredPlantVersion), ...
    'Yaw Damper V2 plant dependency mismatch.');
assert(strcmp(controller.artifactRevision, ...
    actuator.requiredYawDamperRevision), ...
    'Yaw Damper V2 controller dependency mismatch.');
assert(strcmp(actuator.artifactRevision,'YD-V2-AUDIT-MATLAB-R1'), ...
    'The Simulink check requires YD-V2-AUDIT-MATLAB-R1.');
assert(controller.feedbackSign == -1.0, ...
    'Yaw Damper V2 requires negative yaw-rate feedback.');
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
