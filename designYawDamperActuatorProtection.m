%% C172S Yaw Damper V2 - actuator, sensor and protection audit
%
% Frozen dependency:
%   Yaw Damper V1, artifact YD-V1-MATLAB-R2
%
% V2 adds without retuning Kr or Tw:
%   * a first-order yaw-rate sensor filter;
%   * an independently audited pure sensor/processing delay;
%   * automatic-rudder position authority;
%   * first-order, position- and rate-limited rudder-servo dynamics;
%   * a conservative total-rudder mechanical stop; and
%   * deterministic sensor-noise and nonlinear protection stress tests.
%
% V1 files are not modified by this script.

clc;
clear;
close all;

baselineData = c172sLateralDirectionalData();
yawDamper = c172sYawDamperData();
actuator = c172sRudderActuatorData();
[~,nominalPlant] = c172sLateralDirectionalPlant(baselineData);

assert(exist('ss','file') == 2 && exist('tf','file') == 2, ...
    'Control System Toolbox is required for the V2 actuator audit.');
assert(strcmp(baselineData.model.version, ...
    actuator.requiredPlantVersion), ...
    'Yaw Damper V2 plant dependency mismatch.');
assert(strcmp(yawDamper.artifactRevision, ...
    actuator.requiredYawDamperRevision), ...
    'Yaw Damper V2 must use the frozen YD-V1-MATLAB-R2 controller.');
assert(yawDamper.feedbackSign == -1.0, ...
    'Yaw Damper V2 requires explicit negative yaw-rate feedback.');
assert(actuator.authority.upperLimit_rad < ...
    actuator.physicalTravel.upperLimit_rad, ...
    'Automatic authority must remain inside aircraft rudder travel.');
assert(actuator.authority.lowerLimit_rad > ...
    actuator.physicalTravel.lowerLimit_rad, ...
    'Automatic authority must remain inside aircraft rudder travel.');

Kr = yawDamper.rudderGain_s;
Tw = yawDamper.washoutTimeConstant_s;
Ts = actuator.sensor.filterTimeConstant_s;
tauActuatorCases_s = actuator.dynamics.timeConstantCases_s;

[plants,caseLedger] = buildUncertaintyFamily(baselineData,yawDamper);
nPlantCase = numel(plants);
assert(nPlantCase == yawDamper.uncertainty.caseCount, ...
    'The 729-case plant ledger is incomplete.');

openModes = cell(nPlantCase,1);
for iCase = 1:nPlantCase
    openModes{iCase} = classifyPlantModes(plants{iCase}.A);
end

%% Complete 729-plant x 3-actuator-time-constant linear audit

nTau = numel(tauActuatorCases_s);
nLinearCase = nPlantCase*nTau;
caseID = (1:nLinearCase).';
plantCaseID = zeros(nLinearCase,1);
actuatorTau_s = zeros(nLinearCase,1);
isDynamicStable = false(nLinearCase,1);
neutralHeadingPoleMagnitude = zeros(nLinearCase,1);
dutchNaturalFrequency_radps = zeros(nLinearCase,1);
dutchDampingRatio = zeros(nLinearCase,1);
dutchDecayRate_radps = zeros(nLinearCase,1);
rollModeFrequencyRatio = zeros(nLinearCase,1);
rollModeDampingRatio = zeros(nLinearCase,1);
spiralDecayRateRatio = zeros(nLinearCase,1);
balancedDiskMargin = zeros(nLinearCase,1);
balancedDiskPhaseMargin_deg = zeros(nLinearCase,1);
balancedDiskGainMargin_dB = zeros(nLinearCase,1);
diskMarginCriticalFrequency_radps = zeros(nLinearCase,1);

row = 0;
for iCase = 1:nPlantCase
    plant = plants{iCase};
    for iTau = 1:nTau
        row = row + 1;
        tauActuator = tauActuatorCases_s(iTau);
        Aclosed = protectedLinearClosedLoopMatrix( ...
            plant,Kr,Tw,Ts,tauActuator);
        modes = classifyProtectedModes(Aclosed,openModes{iCase});
        loopTransfer = protectedLoopTransfer( ...
            plant,Kr,Tw,Ts,tauActuator);
        margins = balancedDiskMarginsWithDelay( ...
            loopTransfer,0.0,actuator);

        plantCaseID(row) = iCase;
        actuatorTau_s(row) = tauActuator;
        isDynamicStable(row) = modes.dynamicStable;
        neutralHeadingPoleMagnitude(row) = abs(modes.headingPole);
        dutchNaturalFrequency_radps(row) = ...
            modes.dutchNaturalFrequency;
        dutchDampingRatio(row) = modes.dutchDampingRatio;
        dutchDecayRate_radps(row) = modes.dutchDecayRate;
        rollModeFrequencyRatio(row) = ...
            abs(modes.rollModePole)/abs(openModes{iCase}.rollPole);
        rollModeDampingRatio(row) = modes.rollModeDampingRatio;
        spiralDecayRateRatio(row) = ...
            abs(modes.spiralPole)/abs(openModes{iCase}.spiralPole);
        balancedDiskMargin(row) = margins.diskMargin;
        balancedDiskPhaseMargin_deg(row) = margins.phaseMargin_deg;
        balancedDiskGainMargin_dB(row) = margins.gainMargin_dB;
        diskMarginCriticalFrequency_radps(row) = ...
            margins.criticalFrequency_radps;
    end
end
assert(row == nLinearCase,'Linear audit loop count is incorrect.');

linearResults = table(caseID,plantCaseID,actuatorTau_s, ...
    isDynamicStable,neutralHeadingPoleMagnitude, ...
    dutchNaturalFrequency_radps,dutchDampingRatio, ...
    dutchDecayRate_radps,rollModeFrequencyRatio, ...
    rollModeDampingRatio,spiralDecayRateRatio,balancedDiskMargin, ...
    balancedDiskPhaseMargin_deg,balancedDiskGainMargin_dB, ...
    diskMarginCriticalFrequency_radps);

req = actuator.requirements;
assert(all(linearResults.isDynamicStable), ...
    'At least one plant/actuator case has an unstable dynamic mode.');
assert(all(linearResults.neutralHeadingPoleMagnitude < 1.0e-10), ...
    'The neutral kinematic heading pole was not preserved.');
assert(all(linearResults.dutchDampingRatio >= ...
    req.minimumDutchDampingRatio), ...
    'At least one case failed the Dutch-roll damping gate.');
assert(all(linearResults.dutchNaturalFrequency_radps >= ...
    req.minimumDutchNaturalFrequency_radps), ...
    'At least one case failed the Dutch-roll frequency gate.');
assert(all(linearResults.dutchDecayRate_radps >= ...
    req.minimumDutchDecayRate_radps), ...
    'At least one case failed the Dutch-roll decay-rate gate.');
assert(all(linearResults.balancedDiskPhaseMargin_deg >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'At least one zero-delay case failed the disk phase-margin gate.');
assert(all(linearResults.balancedDiskGainMargin_dB >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'At least one zero-delay case failed the disk gain-margin gate.');
assert(all(linearResults.rollModeFrequencyRatio >= ...
    req.minimumRollModeFrequencyRatio & ...
    linearResults.rollModeFrequencyRatio <= ...
    req.maximumRollModeFrequencyRatio), ...
    'Actuator integration changed the matched roll mode excessively.');
assert(all(linearResults.rollModeDampingRatio >= ...
    req.minimumRollModeDampingRatio), ...
    'At least one matched roll/servo mode is insufficiently damped.');
assert(all(linearResults.spiralDecayRateRatio >= ...
    req.minimumSpiralDecayRateRatio), ...
    'At least one case degraded stable spiral decay excessively.');

%% 729-case operational initial-disturbance audit at selected hardware

t = (0:actuator.validation.timeStep_s: ...
    actuator.validation.stopTime_s).';
initialYawRate_radps = deg2rad( ...
    yawDamper.validation.initialYawRate_degps);
initialYawRateIntegralRatio = zeros(nPlantCase,1);
initialBetaIntegralRatio = zeros(nPlantCase,1);
peakInitialActuatorPosition_deg = zeros(nPlantCase,1);
peakInitialActuatorRate_degps = zeros(nPlantCase,1);

for iCase = 1:nPlantCase
    metrics = linearInitialMetrics(plants{iCase},Kr,Tw,Ts, ...
        actuator.dynamics.timeConstant_s,t,initialYawRate_radps);
    initialYawRateIntegralRatio(iCase) = metrics.yawRateIntegralRatio;
    initialBetaIntegralRatio(iCase) = metrics.betaIntegralRatio;
    peakInitialActuatorPosition_deg(iCase) = ...
        metrics.peakActuatorPosition_deg;
    peakInitialActuatorRate_degps(iCase) = ...
        metrics.peakActuatorRate_degps;
end

operationalResults = [caseLedger,table(initialYawRateIntegralRatio, ...
    initialBetaIntegralRatio,peakInitialActuatorPosition_deg, ...
    peakInitialActuatorRate_degps)];

assert(all(operationalResults.initialYawRateIntegralRatio <= ...
    req.maximumInitialYawRateIntegralRatio), ...
    'At least one V2 case failed initial-yaw-rate attenuation.');
assert(all(operationalResults.initialBetaIntegralRatio <= ...
    req.maximumInitialBetaIntegralRatio), ...
    'At least one V2 case failed initial-sideslip attenuation.');
assert(all(operationalResults.peakInitialActuatorPosition_deg <= ...
    req.maximumInitialActuatorPosition_deg), ...
    'At least one V2 case exceeded the operational position gate.');
assert(all(operationalResults.peakInitialActuatorRate_degps <= ...
    req.maximumInitialActuatorRate_degps), ...
    'At least one V2 case exceeded the operational rate gate.');
assert(all(operationalResults.peakInitialActuatorPosition_deg < ...
    min(actuator.authority.cases_deg)), ...
    'The 5-deg/s operational case unexpectedly requires position limiting.');

%% Nominal sensor-filter sensitivity

sensorCases_s = actuator.sensor.filterTimeConstantCases_s;
nSensorAudit = numel(sensorCases_s)*numel(tauActuatorCases_s);
sensorFilterTau_s = zeros(nSensorAudit,1);
sensorActuatorTau_s = zeros(nSensorAudit,1);
sensorAuditDutchDampingRatio = zeros(nSensorAudit,1);
sensorAuditPhaseMargin_deg = zeros(nSensorAudit,1);
sensorAuditGainMargin_dB = zeros(nSensorAudit,1);
sensorAuditStable = false(nSensorAudit,1);
row = 0;
nominalOpenModes = classifyPlantModes(nominalPlant.A);
for iSensor = 1:numel(sensorCases_s)
    for iTau = 1:nTau
        row = row + 1;
        currentTs = sensorCases_s(iSensor);
        currentTau = tauActuatorCases_s(iTau);
        Aclosed = protectedLinearClosedLoopMatrix( ...
            nominalPlant,Kr,Tw,currentTs,currentTau);
        modes = classifyProtectedModes(Aclosed,nominalOpenModes);
        margins = balancedDiskMarginsWithDelay( ...
            protectedLoopTransfer(nominalPlant,Kr,Tw,currentTs,currentTau), ...
            0.0,actuator);
        sensorFilterTau_s(row) = currentTs;
        sensorActuatorTau_s(row) = currentTau;
        sensorAuditDutchDampingRatio(row) = modes.dutchDampingRatio;
        sensorAuditPhaseMargin_deg(row) = margins.phaseMargin_deg;
        sensorAuditGainMargin_dB(row) = margins.gainMargin_dB;
        sensorAuditStable(row) = modes.dynamicStable;
    end
end
sensorResults = table(sensorFilterTau_s,sensorActuatorTau_s, ...
    sensorAuditStable,sensorAuditDutchDampingRatio, ...
    sensorAuditPhaseMargin_deg,sensorAuditGainMargin_dB);
assert(all(sensorResults.sensorAuditStable), ...
    'A nominal sensor/actuator sensitivity case is unstable.');
assert(all(sensorResults.sensorAuditDutchDampingRatio >= ...
    req.minimumDutchDampingRatio), ...
    'A sensor-filter sensitivity case failed Dutch damping.');
assert(all(sensorResults.sensorAuditPhaseMargin_deg >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'A sensor-filter sensitivity case failed disk phase margin.');
assert(all(sensorResults.sensorAuditGainMargin_dB >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'A sensor-filter sensitivity case failed disk gain margin.');

%% Exact frequency-response delay sensitivity across all 729 plants

delayCases_s = actuator.sensor.delayCases_s;
nDelayCase = nPlantCase*numel(delayCases_s);
delayPlantCaseID = zeros(nDelayCase,1);
sensorDelay_s = zeros(nDelayCase,1);
delayDiskMargin = zeros(nDelayCase,1);
delayPhaseMargin_deg = zeros(nDelayCase,1);
delayGainMargin_dB = zeros(nDelayCase,1);
delayCriticalFrequency_radps = zeros(nDelayCase,1);
row = 0;
for iCase = 1:nPlantCase
    loopTransfer = protectedLoopTransfer(plants{iCase},Kr,Tw,Ts, ...
        actuator.dynamics.timeConstant_s);
    for iDelay = 1:numel(delayCases_s)
        row = row + 1;
        delay = delayCases_s(iDelay);
        margins = balancedDiskMarginsWithDelay( ...
            loopTransfer,delay,actuator);
        delayPlantCaseID(row) = iCase;
        sensorDelay_s(row) = delay;
        delayDiskMargin(row) = margins.diskMargin;
        delayPhaseMargin_deg(row) = margins.phaseMargin_deg;
        delayGainMargin_dB(row) = margins.gainMargin_dB;
        delayCriticalFrequency_radps(row) = ...
            margins.criticalFrequency_radps;
    end
end
delayResults = table(delayPlantCaseID,sensorDelay_s,delayDiskMargin, ...
    delayPhaseMargin_deg,delayGainMargin_dB, ...
    delayCriticalFrequency_radps);

acceptedDelayRows = delayResults.sensorDelay_s <= ...
    actuator.sensor.maximumAcceptedDelay_s + 1.0e-12;
assert(all(delayResults.delayPhaseMargin_deg(acceptedDelayRows) >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'The accepted delay envelope failed the disk phase-margin gate.');
assert(all(delayResults.delayGainMargin_dB(acceptedDelayRows) >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'The accepted delay envelope failed the disk gain-margin gate.');
beyondAcceptedRows = ~acceptedDelayRows;
assert(any(delayResults.delayPhaseMargin_deg(beyondAcceptedRows) < ...
    req.minimumBalancedDiskPhaseMargin_deg | ...
    delayResults.delayGainMargin_dB(beyondAcceptedRows) < ...
    req.minimumBalancedDiskGainMargin_dB), ...
    ['The declared delay boundary was not demonstrated; expand or ', ...
    'reclassify the accepted envelope.']);

%% Nominal nonlinear operational captures and protection stress

initialCapture = simulateProtectedCapture(nominalPlant,yawDamper, ...
    actuator,'initial',yawDamper.validation.initialYawRate_degps);
aileronCapture = simulateProtectedCapture(nominalPlant,yawDamper, ...
    actuator,'aileron',0.0);
rudderCapture = simulateProtectedCapture(nominalPlant,yawDamper, ...
    actuator,'rudder',0.0);

initialMetrics = nonlinearCaptureMetrics( ...
    initialCapture,nominalPlant,yawDamper,'initial');
aileronMetrics = nonlinearCaptureMetrics( ...
    aileronCapture,nominalPlant,yawDamper,'aileron');
rudderMetrics = nonlinearCaptureMetrics( ...
    rudderCapture,nominalPlant,yawDamper,'rudder');

assert(initialMetrics.yawRateIntegralRatio <= ...
    req.maximumInitialYawRateIntegralRatio, ...
    'Nominal nonlinear initial-yaw-rate attenuation failed.');
assert(initialMetrics.betaIntegralRatio <= ...
    req.maximumInitialBetaIntegralRatio, ...
    'Nominal nonlinear initial-sideslip attenuation failed.');
assert(~initialCapture.positionLimitActive && ...
    ~initialCapture.rateLimitActive, ...
    'Operational initial disturbance unexpectedly activated protection.');
assert(aileronMetrics.yawRateIntegralRatio <= ...
    req.maximumAileronYawIntegralRatio, ...
    'Nonlinear aileron-pulse yaw attenuation failed.');
assert(aileronMetrics.bankPeakRatio >= ...
    req.minimumAileronBankPeakRatio && ...
    aileronMetrics.bankPeakRatio <= ...
    req.maximumAileronBankPeakRatio, ...
    'Nonlinear aileron-pulse bank preservation failed.');
assert(rudderMetrics.yawRateIntegralRatio <= ...
    req.maximumRudderYawIntegralRatio, ...
    'Nonlinear rudder-pulse yaw attenuation failed.');
assert(rudderMetrics.betaPeakRatio <= ...
    req.maximumRudderBetaPeakRatio, ...
    'Nonlinear rudder-pulse sideslip attenuation failed.');
assert(max(aileronMetrics.peakActuatorPosition_deg, ...
    rudderMetrics.peakActuatorPosition_deg) <= ...
    req.maximumPulseActuatorPosition_deg, ...
    'An operational pulse exceeded the actuator-position gate.');
assert(max(aileronMetrics.peakActuatorRate_degps, ...
    rudderMetrics.peakActuatorRate_degps) <= ...
    req.maximumPulseActuatorRate_degps, ...
    'An operational pulse exceeded the actuator-rate gate.');
assert(~aileronCapture.positionLimitActive && ...
    ~aileronCapture.rateLimitActive && ...
    ~rudderCapture.positionLimitActive && ...
    ~rudderCapture.rateLimitActive, ...
    'An operational pulse unexpectedly activated protection.');
assert(~initialCapture.physicalLimitActive && ...
    ~aileronCapture.physicalLimitActive && ...
    ~rudderCapture.physicalLimitActive, ...
    'An operational capture unexpectedly reached the aircraft hard stop.');

stressCapture = simulateProtectedCapture(nominalPlant,yawDamper, ...
    actuator,'stress',actuator.stress.initialYawRate_degps);
assert(stressCapture.positionLimitActive, ...
    'The logic-stress case did not activate position limiting.');
assert(stressCapture.rateLimitActive, ...
    'The logic-stress case did not activate rate limiting.');
assert(max(abs(rad2deg(stressCapture.z(:,8)))) <= ...
    actuator.authority.right_deg + 1.0e-6, ...
    'The stress case exceeded automatic-rudder authority.');
assert(max(abs(rad2deg(stressCapture.actuatorRate))) <= ...
    actuator.dynamics.rateLimit_degps + 1.0e-6, ...
    'The stress case exceeded the rudder-servo rate limit.');
assert(max(abs(rad2deg(stressCapture.totalRudder))) <= ...
    actuator.physicalTravel.right_deg + 1.0e-6, ...
    'The stress case exceeded the total-rudder hard stop.');
assert(abs(rad2deg(stressCapture.z(end,3))) <= ...
    actuator.stress.maximumFinalYawRate_degps, ...
    'The stress-case yaw rate did not recover near zero.');
assert(abs(rad2deg(stressCapture.z(end,8))) <= ...
    req.maximumReleasedRudder_deg, ...
    'The stress-case automatic rudder did not release near zero.');

%% Deterministic sensor-noise amplification audit

noiseNominal = simulateNoiseResponse(nominalPlant,yawDamper,actuator, ...
    actuator.sensor.noiseRms_degps);
noiseStress = simulateNoiseResponse(nominalPlant,yawDamper,actuator, ...
    actuator.sensor.noiseStressRms_degps);
assert(noiseStress.actuatorRms_deg <= ...
    req.maximumNoiseActuatorRms_deg, ...
    'Stress sensor noise caused excessive RMS automatic rudder.');
assert(noiseStress.actuatorPeak_deg <= ...
    req.maximumNoiseActuatorPeak_deg, ...
    'Stress sensor noise caused excessive peak automatic rudder.');
assert(~noiseStress.positionLimitActive && ...
    ~noiseStress.rateLimitActive, ...
    'Sensor noise unexpectedly activated actuator protection.');

%% Console report and result ledgers

fprintf('============================================================\n');
fprintf(' C172S YAW DAMPER V2 - ACTUATOR AND SENSOR PROTECTION AUDIT\n');
fprintf('============================================================\n\n');
fprintf('Frozen controller artifact           : %s\n', ...
    yawDamper.artifactRevision);
fprintf('Frozen control law                   : deltaR = -Kr*(rSensor-xw)\n');
fprintf('Kr / Tw                              : %.3f s / %.2f s\n',Kr,Tw);
fprintf('Plant                                : %s\n', ...
    baselineData.model.version);
fprintf('Trim                                 : 4000 ft ISA / 110 KTAS / 2550 lb\n\n');

fprintf('FAA total-rudder hard stop           : +/- %.1f deg\n', ...
    actuator.physicalTravel.right_deg);
fprintf('Automatic-rudder authority           : +/- %.1f deg (assumption)\n', ...
    actuator.authority.right_deg);
fprintf('Actuator time constant               : %.3f s [%.3f %.3f %.3f]\n', ...
    actuator.dynamics.timeConstant_s,tauActuatorCases_s(1), ...
    tauActuatorCases_s(2),tauActuatorCases_s(3));
fprintf('Actuator rate limit                  : +/- %.1f deg/s (assumption)\n', ...
    actuator.dynamics.rateLimit_degps);
fprintf('Sensor-filter time constant          : %.3f s (assumption)\n',Ts);
fprintf('Nominal sensor/processing delay      : %.3f s (assumption)\n\n', ...
    actuator.sensor.nominalDelay_s);

fprintf('Stable plant/actuator cases          : %d/%d\n', ...
    nnz(linearResults.isDynamicStable),height(linearResults));
fprintf('Robust Dutch damping                 : %.4f to %.4f\n', ...
    min(linearResults.dutchDampingRatio), ...
    max(linearResults.dutchDampingRatio));
fprintf('Robust Dutch frequency               : %.4f to %.4f rad/s\n', ...
    min(linearResults.dutchNaturalFrequency_radps), ...
    max(linearResults.dutchNaturalFrequency_radps));
fprintf('Balanced disk phase margin           : %.2f to %.2f deg\n', ...
    min(linearResults.balancedDiskPhaseMargin_deg), ...
    max(linearResults.balancedDiskPhaseMargin_deg));
fprintf('Balanced disk minimum gain margin    : %.2f dB\n', ...
    min(linearResults.balancedDiskGainMargin_dB));
fprintf('Matched roll frequency ratio         : %.4f to %.4f\n', ...
    min(linearResults.rollModeFrequencyRatio), ...
    max(linearResults.rollModeFrequencyRatio));
fprintf('Matched roll damping                 : %.4f to %.4f\n', ...
    min(linearResults.rollModeDampingRatio), ...
    max(linearResults.rollModeDampingRatio));
fprintf('Spiral decay-rate ratio              : %.4f to %.4f\n\n', ...
    min(linearResults.spiralDecayRateRatio), ...
    max(linearResults.spiralDecayRateRatio));

fprintf('Initial-r integral ratio, 729 cases  : %.4f to %.4f\n', ...
    min(operationalResults.initialYawRateIntegralRatio), ...
    max(operationalResults.initialYawRateIntegralRatio));
fprintf('Initial-beta ratio, 729 cases        : %.4f to %.4f\n', ...
    min(operationalResults.initialBetaIntegralRatio), ...
    max(operationalResults.initialBetaIntegralRatio));
fprintf('Peak initial actuator position       : %.4f deg\n', ...
    max(operationalResults.peakInitialActuatorPosition_deg));
fprintf('Peak initial actuator rate           : %.4f deg/s\n\n', ...
    max(operationalResults.peakInitialActuatorRate_degps));

fprintf('Nominal aileron yaw ratio            : %.4f\n', ...
    aileronMetrics.yawRateIntegralRatio);
fprintf('Nominal aileron bank ratio           : %.4f\n', ...
    aileronMetrics.bankPeakRatio);
fprintf('Nominal rudder yaw ratio             : %.4f\n', ...
    rudderMetrics.yawRateIntegralRatio);
fprintf('Nominal rudder beta ratio            : %.4f\n', ...
    rudderMetrics.betaPeakRatio);
fprintf('Peak operational pulse position      : %.4f deg\n', ...
    max(aileronMetrics.peakActuatorPosition_deg, ...
    rudderMetrics.peakActuatorPosition_deg));
fprintf('Peak operational pulse rate          : %.4f deg/s\n\n', ...
    max(aileronMetrics.peakActuatorRate_degps, ...
    rudderMetrics.peakActuatorRate_degps));

fprintf('Accepted exact-delay envelope        : 0 to %.0f ms\n', ...
    1000*actuator.sensor.maximumAcceptedDelay_s);
for iDelay = 1:numel(delayCases_s)
    selection = abs(delayResults.sensorDelay_s - ...
        delayCases_s(iDelay)) < 1.0e-12;
    fprintf('  %3.0f ms: min phase %.2f deg, min gain %.2f dB\n', ...
        1000*delayCases_s(iDelay), ...
        min(delayResults.delayPhaseMargin_deg(selection)), ...
        min(delayResults.delayGainMargin_dB(selection)));
end
fprintf('\n');

fprintf('Stress initial yaw rate              : %.1f deg/s\n', ...
    actuator.stress.initialYawRate_degps);
fprintf('Stress peak automatic rudder         : %.4f deg\n', ...
    max(abs(rad2deg(stressCapture.z(:,8)))));
fprintf('Stress peak actuator rate            : %.4f deg/s\n', ...
    max(abs(rad2deg(stressCapture.actuatorRate))));
fprintf('Stress position/rate limits active   : %d / %d\n', ...
    stressCapture.positionLimitActive,stressCapture.rateLimitActive);
fprintf('Stress final yaw rate                : %.6f deg/s\n\n', ...
    rad2deg(stressCapture.z(end,3)));

fprintf('Noise %.3f deg/s RMS -> rudder RMS   : %.5f deg\n', ...
    actuator.sensor.noiseRms_degps,noiseNominal.actuatorRms_deg);
fprintf('Noise %.3f deg/s RMS -> rudder RMS   : %.5f deg\n', ...
    actuator.sensor.noiseStressRms_degps,noiseStress.actuatorRms_deg);
fprintf('Stress-noise peak automatic rudder   : %.5f deg\n\n', ...
    noiseStress.actuatorPeak_deg);

fprintf('PASS: every structured plant/actuator case retains stable dynamics.\n');
fprintf('PASS: V1 damping, disk-margin and operational-response gates pass.\n');
fprintf('PASS: the declared 60-ms delay envelope passes both disk-margin gates.\n');
fprintf('PASS: position/rate protection activates only in the logic stress case.\n');
fprintf('PASS: deterministic sensor-noise amplification remains below its gates.\n');
fprintf(['NOTE: the actuator and sensor values remain explicit project ', ...
    'assumptions, not identified C172S hardware data.\n']);

linearFile = fullfile(pwd,'yaw_damper_v2_linear_robustness_results.csv');
operationalFile = fullfile(pwd, ...
    'yaw_damper_v2_operational_robustness_results.csv');
delayFile = fullfile(pwd,'yaw_damper_v2_delay_sensitivity_results.csv');
writetable(linearResults,linearFile);
writetable(operationalResults,operationalFile);
writetable(delayResults,delayFile);
fprintf('\nSaved linear ledger                 : %s\n',linearFile);
fprintf('Saved operational ledger            : %s\n',operationalFile);
fprintf('Saved delay ledger                  : %s\n',delayFile);

assignin('base','yawDamperV2ActuatorData',actuator);
assignin('base','yawDamperV2LinearResults',linearResults);
assignin('base','yawDamperV2OperationalResults',operationalResults);
assignin('base','yawDamperV2DelayResults',delayResults);

%% Figures

figure('Name','C172S Yaw Damper V2 - Protected Captures','Color','w');
tiledlayout(2,2);
nexttile;
plot(initialCapture.t,rad2deg(initialCapture.z(:,3)),'LineWidth',1.3);
grid on;
xlabel('Time (s)'); ylabel('r (deg/s)');
title('5-deg/s Initial Yaw-Rate Capture');
nexttile;
plot(initialCapture.t,rad2deg(initialCapture.z(:,8)),'LineWidth',1.3);
hold on;
yline(actuator.authority.right_deg,'k--','Authority');
yline(-actuator.authority.left_deg,'k--');
grid on;
xlabel('Time (s)'); ylabel('\delta_r (deg)');
title('Operational Automatic Rudder');
nexttile;
plot(stressCapture.t,rad2deg(stressCapture.z(:,8)),'LineWidth',1.3);
hold on;
yline(actuator.authority.right_deg,'k--','Authority');
yline(-actuator.authority.left_deg,'k--');
grid on;
xlabel('Time (s)'); ylabel('\delta_r (deg)');
title('25-deg/s Logic Stress');
nexttile;
plot(stressCapture.t,rad2deg(stressCapture.actuatorRate), ...
    'LineWidth',1.3);
hold on;
yline(actuator.dynamics.rateLimit_degps,'k--','Rate limit');
yline(-actuator.dynamics.rateLimit_degps,'k--');
grid on;
xlabel('Time (s)'); ylabel('d\delta_r/dt (deg/s)');
title('Stress-Case Servo Rate');
sgtitle('Yaw Damper V2 Actuator Protection');

figure('Name','C172S Yaw Damper V2 - Delay Sensitivity','Color','w');
minimumDelayPhase = zeros(size(delayCases_s));
minimumDelayGain = zeros(size(delayCases_s));
for iDelay = 1:numel(delayCases_s)
    selection = abs(delayResults.sensorDelay_s - ...
        delayCases_s(iDelay)) < 1.0e-12;
    minimumDelayPhase(iDelay) = ...
        min(delayResults.delayPhaseMargin_deg(selection));
    minimumDelayGain(iDelay) = ...
        min(delayResults.delayGainMargin_dB(selection));
end
yyaxis left;
plot(1000*delayCases_s,minimumDelayPhase,'o-','LineWidth',1.4);
hold on;
yline(req.minimumBalancedDiskPhaseMargin_deg,'k--','Phase gate');
ylabel('Minimum disk phase margin (deg)');
yyaxis right;
plot(1000*delayCases_s,minimumDelayGain,'s-','LineWidth',1.4);
yline(req.minimumBalancedDiskGainMargin_dB,'k:','Gain gate');
ylabel('Minimum disk gain margin (dB)');
grid on;
xlabel('Pure sensor/processing delay (ms)');
title('Exact Frequency-Response Delay Sensitivity');

%% Local functions

function [plants,ledger] = buildUncertaintyFamily(baselineData,yawDamper)
derivativeScales = yawDamper.uncertainty.derivativeScaleValues;
inertiaScales = yawDamper.uncertainty.inertiaScaleValues;
ixzNormalizedValues = yawDamper.uncertainty.IxzNormalizedValues;
nCase = numel(derivativeScales)^4*numel(inertiaScales)* ...
    numel(ixzNormalizedValues);

plants = cell(nCase,1);
caseID = (1:nCase).';
sideForceScale = zeros(nCase,1);
rollMomentScale = zeros(nCase,1);
yawMomentScale = zeros(nCase,1);
rudderEffectivenessScale = zeros(nCase,1);
inertiaScale = zeros(nCase,1);
IxzNormalized = zeros(nCase,1);

row = 0;
for iCY = 1:numel(derivativeScales)
    for iCl = 1:numel(derivativeScales)
        for iCn = 1:numel(derivativeScales)
            for iRudder = 1:numel(derivativeScales)
                for iInertia = 1:numel(inertiaScales)
                    for iIxz = 1:numel(ixzNormalizedValues)
                        row = row + 1;
                        data = baselineData;
                        cyScale = derivativeScales(iCY);
                        clScale = derivativeScales(iCl);
                        cnScale = derivativeScales(iCn);
                        rudderScale = derivativeScales(iRudder);
                        currentInertiaScale = inertiaScales(iInertia);
                        currentIxzNormalized = ixzNormalizedValues(iIxz);

                        data.aero.CY_beta = cyScale*data.aero.CY_beta;
                        data.aero.CY_p = cyScale*data.aero.CY_p;
                        data.aero.CY_r = cyScale*data.aero.CY_r;
                        data.aero.Cl_beta = clScale*data.aero.Cl_beta;
                        data.aero.Cl_p = clScale*data.aero.Cl_p;
                        data.aero.Cl_r = clScale*data.aero.Cl_r;
                        data.aero.Cn_beta = cnScale*data.aero.Cn_beta;
                        data.aero.Cn_p = cnScale*data.aero.Cn_p;
                        data.aero.Cn_r = cnScale*data.aero.Cn_r;
                        data.aero.CY_deltaR = ...
                            rudderScale*data.aero.CY_deltaR;
                        data.aero.Cl_deltaR = ...
                            rudderScale*data.aero.Cl_deltaR;
                        data.aero.Cn_deltaR = ...
                            rudderScale*data.aero.Cn_deltaR;

                        data.inertia.Ixx_kgm2 = currentInertiaScale* ...
                            data.inertia.Ixx_kgm2;
                        data.inertia.Izz_kgm2 = currentInertiaScale* ...
                            data.inertia.Izz_kgm2;
                        data.inertia.Ixz_kgm2 = currentIxzNormalized* ...
                            sqrt(data.inertia.Ixx_kgm2* ...
                            data.inertia.Izz_kgm2);

                        [~,plants{row}] = ...
                            c172sLateralDirectionalPlant(data);
                        sideForceScale(row) = cyScale;
                        rollMomentScale(row) = clScale;
                        yawMomentScale(row) = cnScale;
                        rudderEffectivenessScale(row) = rudderScale;
                        inertiaScale(row) = currentInertiaScale;
                        IxzNormalized(row) = currentIxzNormalized;
                    end
                end
            end
        end
    end
end

assert(row == nCase,'The uncertainty-family loop count is incorrect.');
ledger = table(caseID,sideForceScale,rollMomentScale, ...
    yawMomentScale,rudderEffectivenessScale,inertiaScale,IxzNormalized);
end

function Aclosed = protectedLinearClosedLoopMatrix( ...
    plant,Kr,Tw,Ts,tauActuator)
% State order: [beta p r phi psi rSensor xw deltaRServo].
Aclosed = zeros(8,8);
Aclosed(1:5,1:5) = plant.A;
Aclosed(1:5,8) = plant.B(:,2);
Aclosed(6,3) = 1.0/Ts;
Aclosed(6,6) = -1.0/Ts;
Aclosed(7,6) = 1.0/Tw;
Aclosed(7,7) = -1.0/Tw;
Aclosed(8,6) = -Kr/tauActuator;
Aclosed(8,7) = Kr/tauActuator;
Aclosed(8,8) = -1.0/tauActuator;
end

function loopTransfer = protectedLoopTransfer( ...
    plant,Kr,Tw,Ts,tauActuator)
rudderToYawRate = ss(plant.A(1:4,1:4),plant.B(1:4,2), ...
    [0 0 1 0],0);
sensor = tf(1,[Ts 1]);
washout = tf([Tw 0],[Tw 1]);
servo = tf(1,[tauActuator 1]);
loopTransfer = Kr*sensor*washout*servo*rudderToYawRate;
end

function modes = classifyPlantModes(A)
poles = eig(A);
[~,headingIndex] = min(abs(poles));
modes.headingPole = poles(headingIndex);
poles(headingIndex) = [];

positiveComplex = poles(imag(poles) > 1.0e-8);
assert(numel(positiveComplex) == 1, ...
    'Expected exactly one open-loop Dutch-roll pole pair.');
modes.dutchPole = positiveComplex(1);
modes.dutchNaturalFrequency = abs(modes.dutchPole);
modes.dutchDampingRatio = ...
    -real(modes.dutchPole)/modes.dutchNaturalFrequency;
modes.dutchDecayRate = -real(modes.dutchPole);

realPoles = real(poles(abs(imag(poles)) <= 1.0e-8));
assert(numel(realPoles) == 2, ...
    'Expected open-loop roll and spiral real modes.');
[~,rollIndex] = max(abs(realPoles));
modes.rollPole = realPoles(rollIndex);
realPoles(rollIndex) = [];
modes.spiralPole = realPoles(1);
end

function modes = classifyProtectedModes(Aclosed,openModes)
poles = eig(Aclosed);
[~,headingIndex] = min(abs(poles));
modes.headingPole = poles(headingIndex);
dynamicPoles = poles;
dynamicPoles(headingIndex) = [];
modes.dynamicStable = all(real(dynamicPoles) < -1.0e-10);

positiveComplex = dynamicPoles(imag(dynamicPoles) > 1.0e-8);
assert(~isempty(positiveComplex), ...
    'No protected Dutch-roll pole pair was found.');
[~,dutchIndex] = min(abs(positiveComplex - openModes.dutchPole));
modes.dutchPole = positiveComplex(dutchIndex);
modes.dutchNaturalFrequency = abs(modes.dutchPole);
modes.dutchDampingRatio = ...
    -real(modes.dutchPole)/modes.dutchNaturalFrequency;
modes.dutchDecayRate = -real(modes.dutchPole);

[~,rollIndex] = min(abs(dynamicPoles - openModes.rollPole));
modes.rollModePole = dynamicPoles(rollIndex);
modes.rollModeDampingRatio = ...
    -real(modes.rollModePole)/abs(modes.rollModePole);

realPoles = real(dynamicPoles(abs(imag(dynamicPoles)) <= 1.0e-8));
assert(~isempty(realPoles), ...
    'No real protected spiral mode was found.');
[~,spiralIndex] = min(abs(realPoles - openModes.spiralPole));
modes.spiralPole = realPoles(spiralIndex);
end

function margins = balancedDiskMarginsWithDelay( ...
    loopTransfer,delay_s,actuator)
frequencyRange = ...
    actuator.validation.diskMarginFrequencyRange_radps;
nGrid = actuator.validation.diskMarginGridPointCount;
frequencyGrid = logspace(log10(frequencyRange(1)), ...
    log10(frequencyRange(2)),nGrid).';
loopResponse = reshape(freqresp(loopTransfer,frequencyGrid),[],1);
loopResponse = loopResponse.*exp(-1i*frequencyGrid*delay_s);
alphaGrid = abs(2*(1 + loopResponse)./(1 - loopResponse));
[diskMargin,minimumIndex] = min(alphaGrid);
assert(minimumIndex > 1 && minimumIndex < nGrid, ...
    'Disk-margin grid does not bracket its critical frequency.');

margins.diskMargin = diskMargin;
margins.criticalFrequency_radps = frequencyGrid(minimumIndex);
margins.phaseMargin_deg = 2*atand(diskMargin/2);
if diskMargin < 2
    gainMarginUpper = (2 + diskMargin)/(2 - diskMargin);
    margins.gainMargin_dB = 20*log10(gainMarginUpper);
else
    margins.gainMargin_dB = inf;
end
end

function metrics = linearInitialMetrics( ...
    plant,Kr,Tw,Ts,tauActuator,t,initialYawRate_radps)
Aclosed = protectedLinearClosedLoopMatrix( ...
    plant,Kr,Tw,Ts,tauActuator);
dt = t(2) - t(1);
AdOpen = expm(plant.A*dt);
AdClosed = expm(Aclosed*dt);
xOpen = zeros(5,1);
xOpen(3) = initialYawRate_radps;
xClosed = zeros(8,1);
xClosed(3) = initialYawRate_radps;
openHistory = zeros(numel(t),5);
closedHistory = zeros(numel(t),8);
actuatorRate = zeros(numel(t),1);

for k = 1:numel(t)
    openHistory(k,:) = xOpen.';
    closedHistory(k,:) = xClosed.';
    actuatorCommand = -Kr*(xClosed(6) - xClosed(7));
    actuatorRate(k) = ...
        (actuatorCommand - xClosed(8))/tauActuator;
    if k < numel(t)
        xOpen = AdOpen*xOpen;
        xClosed = AdClosed*xClosed;
    end
end

metrics.yawRateIntegralRatio = ...
    trapz(t,abs(closedHistory(:,3)))/ ...
    trapz(t,abs(openHistory(:,3)));
metrics.betaIntegralRatio = ...
    trapz(t,abs(closedHistory(:,1)))/ ...
    trapz(t,abs(openHistory(:,1)));
metrics.peakActuatorPosition_deg = ...
    max(abs(rad2deg(closedHistory(:,8))));
metrics.peakActuatorRate_degps = ...
    max(abs(rad2deg(actuatorRate)));
end

function capture = simulateProtectedCapture( ...
    plant,yawDamper,actuator,caseName,initialYawRate_degps)
if strcmp(caseName,'stress')
    dt = actuator.stress.timeStep_s;
    stopTime = actuator.stress.stopTime_s;
else
    dt = actuator.validation.timeStep_s;
    stopTime = actuator.validation.stopTime_s;
end
t = (0:dt:stopTime).';
z = zeros(8,1);
z(3) = deg2rad(initialYawRate_degps);
zHistory = zeros(numel(t),8);
actuatorRate = zeros(numel(t),1);
totalRudder = zeros(numel(t),1);
positionLimited = false(numel(t),1);
rateLimited = false(numel(t),1);
physicalLimited = false(numel(t),1);

for k = 1:numel(t)
    zHistory(k,:) = z.';
    pilot = validationPilotInput(t(k),caseName,yawDamper);
    [~,diagnostic] = protectedDerivative( ...
        z,pilot,0.0,plant,yawDamper,actuator);
    actuatorRate(k) = diagnostic.actuatorRate;
    totalRudder(k) = diagnostic.totalRudder;
    positionLimited(k) = diagnostic.positionLimited;
    rateLimited(k) = diagnostic.rateLimited;
    physicalLimited(k) = diagnostic.physicalLimited;

    if k < numel(t)
        t0 = t(k);
        h = t(k + 1) - t0;
        u1 = validationPilotInput(t0,caseName,yawDamper);
        u2 = validationPilotInput(t0 + 0.5*h,caseName,yawDamper);
        u4 = validationPilotInput(t0 + h,caseName,yawDamper);
        k1 = protectedDerivative(z,u1,0.0,plant,yawDamper,actuator);
        k2 = protectedDerivative(z + 0.5*h*k1,u2,0.0, ...
            plant,yawDamper,actuator);
        k3 = protectedDerivative(z + 0.5*h*k2,u2,0.0, ...
            plant,yawDamper,actuator);
        k4 = protectedDerivative(z + h*k3,u4,0.0, ...
            plant,yawDamper,actuator);
        z = z + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

capture.t = t;
capture.z = zHistory;
capture.actuatorRate = actuatorRate;
capture.totalRudder = totalRudder;
capture.positionLimited = positionLimited;
capture.rateLimited = rateLimited;
capture.physicalLimited = physicalLimited;
capture.positionLimitActive = any(positionLimited);
capture.rateLimitActive = any(rateLimited);
capture.physicalLimitActive = any(physicalLimited);
end

function metrics = nonlinearCaptureMetrics( ...
    capture,plant,yawDamper,caseName)
openHistory = openReference(capture.t,plant,yawDamper,caseName, ...
    capture.z(1,3));
metrics.yawRateIntegralRatio = ...
    trapz(capture.t,abs(capture.z(:,3)))/ ...
    trapz(capture.t,abs(openHistory(:,3)));
metrics.betaIntegralRatio = ...
    trapz(capture.t,abs(capture.z(:,1)))/ ...
    trapz(capture.t,abs(openHistory(:,1)));
metrics.betaPeakRatio = max(abs(capture.z(:,1)))/ ...
    max(abs(openHistory(:,1)));
metrics.bankPeakRatio = max(abs(capture.z(:,4)))/ ...
    max(abs(openHistory(:,4)));
metrics.peakActuatorPosition_deg = ...
    max(abs(rad2deg(capture.z(:,8))));
metrics.peakActuatorRate_degps = ...
    max(abs(rad2deg(capture.actuatorRate)));
end

function openHistory = openReference( ...
    t,plant,yawDamper,caseName,initialYawRate_radps)
x = zeros(5,1);
x(3) = initialYawRate_radps;
openHistory = zeros(numel(t),5);
derivative = @(state,input) plant.A*state + plant.B*input;
for k = 1:numel(t)
    openHistory(k,:) = x.';
    if k < numel(t)
        t0 = t(k);
        h = t(k + 1) - t0;
        u1 = validationPilotInput(t0,caseName,yawDamper);
        u2 = validationPilotInput(t0 + 0.5*h,caseName,yawDamper);
        u4 = validationPilotInput(t0 + h,caseName,yawDamper);
        k1 = derivative(x,u1);
        k2 = derivative(x + 0.5*h*k1,u2);
        k3 = derivative(x + 0.5*h*k2,u2);
        k4 = derivative(x + h*k3,u4);
        x = x + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    end
end
end

function pilot = validationPilotInput(t,caseName,yawDamper)
pilot = zeros(2,1);
if strcmp(caseName,'aileron') && ...
        t >= yawDamper.validation.aileronPulseStart_s && ...
        t < yawDamper.validation.aileronPulseEnd_s
    pilot(1) = deg2rad(yawDamper.validation.aileronPulse_deg);
elseif strcmp(caseName,'rudder') && ...
        t >= yawDamper.validation.rudderPulseStart_s && ...
        t < yawDamper.validation.rudderPulseEnd_s
    pilot(2) = deg2rad(yawDamper.validation.rudderPulse_deg);
end
end

function [zDot,diagnostic] = protectedDerivative( ...
    z,pilot,measurementNoise,plant,yawDamper,actuator)
Kr = yawDamper.rudderGain_s;
Tw = yawDamper.washoutTimeConstant_s;
Ts = actuator.sensor.filterTimeConstant_s;
tauActuator = actuator.dynamics.timeConstant_s;
rateLimit = deg2rad(actuator.dynamics.rateLimit_degps);

measuredYawRate = z(3) + measurementNoise;
rawCommand = -Kr*(z(6) - z(7));
positionCommand = min(max(rawCommand, ...
    actuator.authority.lowerLimit_rad), ...
    actuator.authority.upperLimit_rad);
rawRate = (positionCommand - z(8))/tauActuator;
limitedRate = min(max(rawRate,-rateLimit),rateLimit);
rawTotalRudder = pilot(2) + z(8);
totalRudder = min(max(rawTotalRudder, ...
    actuator.physicalTravel.lowerLimit_rad), ...
    actuator.physicalTravel.upperLimit_rad);

zDot = zeros(8,1);
zDot(1:5) = plant.A*z(1:5) + ...
    plant.B*[pilot(1);totalRudder];
zDot(6) = (measuredYawRate - z(6))/Ts;
zDot(7) = (z(6) - z(7))/Tw;
zDot(8) = limitedRate;

diagnostic.rawCommand = rawCommand;
diagnostic.positionCommand = positionCommand;
diagnostic.actuatorRate = limitedRate;
diagnostic.totalRudder = totalRudder;
diagnostic.positionLimited = ...
    abs(positionCommand - rawCommand) > 1.0e-12;
diagnostic.rateLimited = abs(limitedRate - rawRate) > 1.0e-12;
diagnostic.physicalLimited = ...
    abs(totalRudder - rawTotalRudder) > 1.0e-12;
end

function metrics = simulateNoiseResponse( ...
    plant,yawDamper,actuator,noiseRms_degps)
dt = actuator.noise.timeStep_s;
t = (0:dt:actuator.noise.stopTime_s).';
sampleInterval = round(actuator.sensor.noiseSampleTime_s/dt);
assert(abs(sampleInterval*dt - ...
    actuator.sensor.noiseSampleTime_s) < 1.0e-12, ...
    'Noise sample time must be an integer multiple of integration time.');

rng(actuator.sensor.noiseRandomSeed,'twister');
z = zeros(8,1);
actuatorHistory = zeros(numel(t),1);
positionLimited = false(numel(t),1);
rateLimited = false(numel(t),1);
measurementNoise = 0.0;
pilot = zeros(2,1);

for k = 1:numel(t)
    if mod(k - 1,sampleInterval) == 0
        measurementNoise = deg2rad(noiseRms_degps)*randn();
    end
    actuatorHistory(k) = z(8);
    [~,diagnostic] = protectedDerivative( ...
        z,pilot,measurementNoise,plant,yawDamper,actuator);
    positionLimited(k) = diagnostic.positionLimited;
    rateLimited(k) = diagnostic.rateLimited;
    if k < numel(t)
        h = t(k + 1) - t(k);
        k1 = protectedDerivative(z,pilot,measurementNoise, ...
            plant,yawDamper,actuator);
        k2 = protectedDerivative(z + 0.5*h*k1,pilot,measurementNoise, ...
            plant,yawDamper,actuator);
        k3 = protectedDerivative(z + 0.5*h*k2,pilot,measurementNoise, ...
            plant,yawDamper,actuator);
        k4 = protectedDerivative(z + h*k3,pilot,measurementNoise, ...
            plant,yawDamper,actuator);
        z = z + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

retained = t >= actuator.noise.discardTime_s;
retainedActuator_deg = rad2deg(actuatorHistory(retained));
metrics.actuatorRms_deg = sqrt(mean(retainedActuator_deg.^2));
metrics.actuatorPeak_deg = max(abs(retainedActuator_deg));
metrics.positionLimitActive = any(positionLimited);
metrics.rateLimitActive = any(rateLimited);
end
