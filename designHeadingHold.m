%% C172S Heading Hold V1 - MATLAB design and source-assumption audit
%
% Frozen dependencies:
%   * Lateral-Directional Plant V0.1
%   * YD-V1-MATLAB-R2 and YD-V2-AUDIT-MATLAB-R1
%   * AILERON-V1-AUDIT-MATLAB-R1
%   * RAH-V1-MATLAB-R1
%
% Heading Hold V1 adds only a circular heading-error outer loop. It issues
% a bounded bank-angle command to the frozen protected roll-attitude loop.
% No frozen plant, yaw, roll, actuator or sensor artifact is modified.

clc;
clear;
close all;

baselineData = c172sLateralDirectionalData();
yawDamper = c172sYawDamperData();
rudder = c172sRudderActuatorData();
rollController = c172sRollAttitudeControllerData();
aileron = c172sAileronActuatorData();
headingSensor = c172sHeadingSensorData();
controller = c172sHeadingHoldControllerData();
[~,nominalPlant] = c172sLateralDirectionalPlant(baselineData);

assert(exist('ss','file') == 2 && exist('lsim','file') == 2, ...
    'Control System Toolbox is required for Heading Hold V1.');
assertDependencies(baselineData,yawDamper,rudder,rollController, ...
    aileron,headingSensor,controller);
assert(abs(controller.bankCommandAuthority_deg- ...
    rollController.commandAuthority.rightWingDown_deg) < 1.0e-12, ...
    'Heading Hold V1 must reuse the frozen RAH bank-command authority.');

Kpsi = controller.headingErrorGain;
[plants,plantLedger] = buildHeadingUncertaintyFamily( ...
    baselineData,controller);
nPlant = numel(plants);
assert(nPlant == controller.uncertainty.caseCount, ...
    'The inherited 729-case heading uncertainty ledger is incomplete.');

tauCases_s = aileron.dynamics.timeConstantCases_s;
nTau = numel(tauCases_s);
w = logspace(log10(controller.validation. ...
    diskMarginFrequencyRange_radps(1)), ...
    log10(controller.validation.diskMarginFrequencyRange_radps(2)), ...
    controller.validation.diskMarginGridPointCount).';

% Treat the maximum already-accepted RAH roll-channel delay as an added
% pure outer-loop delay. This is conservative at the slow heading-loop
% crossover and prevents the outer audit from silently assuming an
% instantaneous frozen inner loop.
inheritedRollDelay_s = aileron.sensor.maximumAcceptedDelay_s;

%% Precompute open outer-loop frequency responses

Gheading = complex(zeros(numel(w),nPlant,nTau));
for iPlant = 1:nPlant
    for iTau = 1:nTau
        [Aopen,Bopen,Copen] = openHeadingLoopSystem(plants{iPlant}, ...
            yawDamper,rudder,rollController,aileron,headingSensor, ...
            tauCases_s(iTau),headingSensor.filterTimeConstant_s);
        response = freqresp(ss(Aopen,Bopen,Copen,0),w);
        Gheading(:,iPlant,iTau) = reshape(response,[],1);
    end
end

%% Reproducible outer-loop gain search

gainGrid = (controller.design.gainSearchRange(1): ...
    controller.design.gainIncrement: ...
    controller.design.gainSearchRange(2)).';
nGain = numel(gainGrid);
gainAllStable = false(nGain,1);
gainMinimumDecayRate_per_s = zeros(nGain,1);
gainMinimumDampingRatio = zeros(nGain,1);
gainMinimumPhaseMargin_deg = zeros(nGain,1);
gainMinimumGainMargin_dB = zeros(nGain,1);
gainPass = false(nGain,1);
acceptedHeadingDelays_s = headingSensor.delayCases_s( ...
    headingSensor.delayCases_s <= ...
    headingSensor.maximumAcceptedDelay_s + 1.0e-12);

for iGain = 1:nGain
    candidateGain = gainGrid(iGain);
    minimumDecay = inf;
    minimumDamping = inf;
    minimumPhase = inf;
    minimumGain = inf;
    allStable = true;
    for iPlant = 1:nPlant
        for iTau = 1:nTau
            Aclosed = headingClosedLoopMatrix(plants{iPlant}, ...
                yawDamper,rudder,rollController,aileron,headingSensor, ...
                tauCases_s(iTau),headingSensor.filterTimeConstant_s, ...
                candidateGain);
            modes = classifyHeadingClosedLoop(Aclosed);
            allStable = allStable && modes.stable;
            minimumDecay = min(minimumDecay,modes.minimumDecayRate_per_s);
            minimumDamping = min(minimumDamping, ...
                modes.minimumOscillatoryDampingRatio);
            for headingDelay_s = acceptedHeadingDelays_s
                totalDelay_s = headingDelay_s + inheritedRollDelay_s;
                margins = headingLoopMargins( ...
                    Gheading(:,iPlant,iTau),w,candidateGain,totalDelay_s);
                minimumPhase = min(minimumPhase,margins.phaseMargin_deg);
                minimumGain = min(minimumGain,margins.gainMargin_dB);
            end
        end
    end
    gainAllStable(iGain) = allStable;
    gainMinimumDecayRate_per_s(iGain) = minimumDecay;
    gainMinimumDampingRatio(iGain) = minimumDamping;
    gainMinimumPhaseMargin_deg(iGain) = minimumPhase;
    gainMinimumGainMargin_dB(iGain) = minimumGain;
    gainPass(iGain) = allStable && ...
        minimumDecay >= controller.design.minimumDynamicDecayRate_per_s && ...
        minimumDamping >= controller.design. ...
            minimumOscillatoryDampingRatio && ...
        minimumPhase >= controller.design. ...
            minimumBalancedDiskPhaseMargin_deg && ...
        minimumGain >= controller.design.minimumBalancedDiskGainMargin_dB;
end

gainSearchResults = table(gainGrid,gainAllStable, ...
    gainMinimumDecayRate_per_s,gainMinimumDampingRatio, ...
    gainMinimumPhaseMargin_deg,gainMinimumGainMargin_dB,gainPass);
selectedGainIndex = find(gainPass,1,'last');
assert(~isempty(selectedGainIndex), ...
    'No heading gain satisfies the complete-family release gates.');
assert(abs(gainGrid(selectedGainIndex)-Kpsi) < 1.0e-12, ...
    'The frozen Kpsi is not the largest passing grid point.');
assert(selectedGainIndex < nGain && ~gainPass(selectedGainIndex+1), ...
    'The next larger Kpsi must fail at least one declared gate.');

%% Complete 729-plant x 3-aileron linear audit

nLinear = nPlant*nTau;
caseID = (1:nLinear).';
plantCaseID = zeros(nLinear,1);
aileronTau_s = zeros(nLinear,1);
isStable = false(nLinear,1);
minimumDecayRate_per_s = zeros(nLinear,1);
minimumOscillatoryDampingRatio = zeros(nLinear,1);
balancedDiskPhaseMargin_deg = zeros(nLinear,1);
balancedDiskGainMargin_dB = zeros(nLinear,1);
diskMarginCriticalFrequency_radps = zeros(nLinear,1);
row = 0;
for iPlant = 1:nPlant
    for iTau = 1:nTau
        row = row + 1;
        tauA = tauCases_s(iTau);
        Aclosed = headingClosedLoopMatrix(plants{iPlant}, ...
            yawDamper,rudder,rollController,aileron,headingSensor, ...
            tauA,headingSensor.filterTimeConstant_s,Kpsi);
        modes = classifyHeadingClosedLoop(Aclosed);
        margins = headingLoopMargins(Gheading(:,iPlant,iTau),w,Kpsi, ...
            headingSensor.nominalDelay_s + inheritedRollDelay_s);
        plantCaseID(row) = iPlant;
        aileronTau_s(row) = tauA;
        isStable(row) = modes.stable;
        minimumDecayRate_per_s(row) = modes.minimumDecayRate_per_s;
        minimumOscillatoryDampingRatio(row) = ...
            modes.minimumOscillatoryDampingRatio;
        balancedDiskPhaseMargin_deg(row) = margins.phaseMargin_deg;
        balancedDiskGainMargin_dB(row) = margins.gainMargin_dB;
        diskMarginCriticalFrequency_radps(row) = ...
            margins.criticalFrequency_radps;
    end
end

linearResults = table(caseID,plantCaseID,aileronTau_s,isStable, ...
    minimumDecayRate_per_s,minimumOscillatoryDampingRatio, ...
    balancedDiskPhaseMargin_deg,balancedDiskGainMargin_dB, ...
    diskMarginCriticalFrequency_radps);
req = controller.requirements;
assert(all(linearResults.isStable), ...
    'At least one heading plant/actuator case is unstable.');
assert(all(linearResults.minimumDecayRate_per_s >= ...
    req.minimumDynamicDecayRate_per_s), ...
    'At least one heading case failed the decay-rate gate.');
assert(all(linearResults.minimumOscillatoryDampingRatio >= ...
    req.minimumOscillatoryDampingRatio), ...
    'At least one heading case failed the damping gate.');
assert(all(linearResults.balancedDiskPhaseMargin_deg >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'At least one nominal-delay heading case failed the phase gate.');
assert(all(linearResults.balancedDiskGainMargin_dB >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'At least one nominal-delay heading case failed the gain gate.');

%% Heading-filter sensitivity at the nominal plant

sensorTauCases_s = headingSensor.filterTimeConstantCases_s;
nSensorCase = numel(sensorTauCases_s)*nTau;
headingFilterTau_s = zeros(nSensorCase,1);
sensorAileronTau_s = zeros(nSensorCase,1);
sensorStable = false(nSensorCase,1);
sensorMinimumDecayRate_per_s = zeros(nSensorCase,1);
sensorMinimumDampingRatio = zeros(nSensorCase,1);
sensorPhaseMargin_deg = zeros(nSensorCase,1);
sensorGainMargin_dB = zeros(nSensorCase,1);
row = 0;
for currentSensorTau_s = sensorTauCases_s
    for currentAileronTau_s = tauCases_s
        row = row + 1;
        [Aopen,Bopen,Copen] = openHeadingLoopSystem(nominalPlant, ...
            yawDamper,rudder,rollController,aileron,headingSensor, ...
            currentAileronTau_s,currentSensorTau_s);
        currentG = reshape(freqresp(ss(Aopen,Bopen,Copen,0),w),[],1);
        Aclosed = Aopen-Bopen*Kpsi*Copen;
        modes = classifyHeadingClosedLoop(Aclosed);
        margins = headingLoopMargins(currentG,w,Kpsi, ...
            headingSensor.nominalDelay_s + inheritedRollDelay_s);
        headingFilterTau_s(row) = currentSensorTau_s;
        sensorAileronTau_s(row) = currentAileronTau_s;
        sensorStable(row) = modes.stable;
        sensorMinimumDecayRate_per_s(row) = modes.minimumDecayRate_per_s;
        sensorMinimumDampingRatio(row) = ...
            modes.minimumOscillatoryDampingRatio;
        sensorPhaseMargin_deg(row) = margins.phaseMargin_deg;
        sensorGainMargin_dB(row) = margins.gainMargin_dB;
    end
end
sensorResults = table(headingFilterTau_s,sensorAileronTau_s, ...
    sensorStable,sensorMinimumDecayRate_per_s, ...
    sensorMinimumDampingRatio,sensorPhaseMargin_deg,sensorGainMargin_dB);
assert(all(sensorResults.sensorStable), ...
    'A heading-filter sensitivity case is unstable.');
assert(all(sensorResults.sensorMinimumDecayRate_per_s >= ...
    req.minimumDynamicDecayRate_per_s), ...
    'A heading-filter case failed the decay-rate gate.');
assert(all(sensorResults.sensorMinimumDampingRatio >= ...
    req.minimumOscillatoryDampingRatio), ...
    'A heading-filter case failed the damping gate.');
assert(all(sensorResults.sensorPhaseMargin_deg >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'A heading-filter case failed the phase-margin gate.');
assert(all(sensorResults.sensorGainMargin_dB >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'A heading-filter case failed the gain-margin gate.');

%% Exact heading-delay sensitivity across every plant/actuator case

delayCases_s = headingSensor.delayCases_s(:);
nDelay = numel(delayCases_s);
headingDelay_s = delayCases_s;
auditedTotalDelay_s = delayCases_s + inheritedRollDelay_s;
delayPhaseMargin_deg = inf(nDelay,1);
delayGainMargin_dB = inf(nDelay,1);
delayCriticalFrequency_radps = zeros(nDelay,1);
for iDelay = 1:nDelay
    for iPlant = 1:nPlant
        for iTau = 1:nTau
            margins = headingLoopMargins(Gheading(:,iPlant,iTau),w,Kpsi, ...
                auditedTotalDelay_s(iDelay));
            if margins.phaseMargin_deg < delayPhaseMargin_deg(iDelay)
                delayPhaseMargin_deg(iDelay) = margins.phaseMargin_deg;
                delayCriticalFrequency_radps(iDelay) = ...
                    margins.criticalFrequency_radps;
            end
            delayGainMargin_dB(iDelay) = min(delayGainMargin_dB(iDelay), ...
                margins.gainMargin_dB);
        end
    end
end
delayResults = table(headingDelay_s,auditedTotalDelay_s, ...
    delayPhaseMargin_deg,delayGainMargin_dB, ...
    delayCriticalFrequency_radps);
acceptedRows = delayResults.headingDelay_s <= ...
    headingSensor.maximumAcceptedDelay_s + 1.0e-12;
assert(all(delayResults.delayPhaseMargin_deg(acceptedRows) >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'The accepted heading-delay envelope failed the phase gate.');
assert(all(delayResults.delayGainMargin_dB(acceptedRows) >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'The accepted heading-delay envelope failed the gain gate.');
outsideRows = ~acceptedRows;
assert(any(delayResults.delayPhaseMargin_deg(outsideRows) < ...
    req.minimumBalancedDiskPhaseMargin_deg | ...
    delayResults.delayGainMargin_dB(outsideRows) < ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'The selected heading-delay boundary is not demonstrated by the grid.');

%% 729-case linear 30-deg heading-command audit

t = (0:controller.validation.timeStep_s: ...
    controller.validation.stopTime_s).';
command_rad = deg2rad(controller.validation.commandStep_deg);
nTime = numel(t);
headingHistory_deg = zeros(nTime,nPlant);
bankHistory_deg = zeros(nTime,nPlant);
rollRateHistory_degps = zeros(nTime,nPlant);
sideslipHistory_deg = zeros(nTime,nPlant);
riseTime_s = zeros(nPlant,1);
overshoot_pct = zeros(nPlant,1);
settlingTime_s = zeros(nPlant,1);
finalHeadingError_deg = zeros(nPlant,1);
peakBank_deg = zeros(nPlant,1);
peakRollRate_degps = zeros(nPlant,1);
peakYawRate_degps = zeros(nPlant,1);
peakSideslip_deg = zeros(nPlant,1);
peakAutomaticAileron_deg = zeros(nPlant,1);
peakAutomaticRudder_deg = zeros(nPlant,1);
peakAileronRate_degps = zeros(nPlant,1);

for iPlant = 1:nPlant
    [Aclosed,Bclosed] = headingCommandSystem(plants{iPlant}, ...
        yawDamper,rudder,rollController,aileron,headingSensor,Kpsi);
    z = lsim(ss(Aclosed,Bclosed,eye(14),zeros(14,1)), ...
        command_rad*ones(size(t)),t);
    heading = z(:,5);
    step = positiveStepMetrics(t,heading,command_rad);
    rawAileron = rollController.bankProportionalGain* ...
        (z(:,13)-z(:,10)) + rollController.bankIntegralGain_per_s* ...
        z(:,11) - rollController.rollRateGain_s*z(:,9);
    aileronRate = (rawAileron-z(:,12))/ ...
        aileron.dynamics.timeConstant_s;
    headingHistory_deg(:,iPlant) = rad2deg(heading);
    bankHistory_deg(:,iPlant) = rad2deg(z(:,4));
    rollRateHistory_degps(:,iPlant) = rad2deg(z(:,2));
    sideslipHistory_deg(:,iPlant) = rad2deg(z(:,1));
    riseTime_s(iPlant) = step.riseTime_s;
    overshoot_pct(iPlant) = step.overshoot_pct;
    settlingTime_s(iPlant) = step.settlingTime_s;
    finalHeadingError_deg(iPlant) = abs(rad2deg(command_rad-heading(end)));
    peakBank_deg(iPlant) = max(abs(rad2deg(z(:,4))));
    peakRollRate_degps(iPlant) = max(abs(rad2deg(z(:,2))));
    peakYawRate_degps(iPlant) = max(abs(rad2deg(z(:,3))));
    peakSideslip_deg(iPlant) = max(abs(rad2deg(z(:,1))));
    peakAutomaticAileron_deg(iPlant) = max(abs(rad2deg(z(:,12))));
    peakAutomaticRudder_deg(iPlant) = max(abs(rad2deg(z(:,8))));
    peakAileronRate_degps(iPlant) = max(abs(rad2deg(aileronRate)));
end

commandResults = [plantLedger,table(riseTime_s,overshoot_pct, ...
    settlingTime_s,finalHeadingError_deg,peakBank_deg, ...
    peakRollRate_degps,peakYawRate_degps,peakSideslip_deg, ...
    peakAutomaticAileron_deg,peakAutomaticRudder_deg, ...
    peakAileronRate_degps)];
assert(all(commandResults.riseTime_s >= req.minimumRiseTime_s & ...
    commandResults.riseTime_s <= req.maximumRiseTime_s), ...
    'At least one heading case failed the rise-time gate.');
assert(all(commandResults.overshoot_pct <= ...
    req.maximumStepOvershoot_pct), ...
    'At least one heading case failed the overshoot gate.');
assert(all(commandResults.settlingTime_s <= req.maximumSettlingTime_s), ...
    'At least one heading case failed the settling-time gate.');
assert(all(commandResults.finalHeadingError_deg <= ...
    req.maximumFinalHeadingError_deg), ...
    'At least one heading case failed the final-error gate.');
assert(all(commandResults.peakBank_deg <= req.maximumOperationalBank_deg), ...
    'At least one heading case exceeded operational bank.');
assert(all(commandResults.peakRollRate_degps <= ...
    req.maximumOperationalRollRate_degps), ...
    'At least one heading case exceeded operational roll rate.');
assert(all(commandResults.peakYawRate_degps <= ...
    req.maximumOperationalYawRate_degps), ...
    'At least one heading case exceeded operational yaw rate.');
assert(all(commandResults.peakSideslip_deg <= ...
    req.maximumOperationalSideslip_deg), ...
    'At least one heading case exceeded the sideslip gate.');
assert(all(commandResults.peakAutomaticAileron_deg <= ...
    req.maximumAutomaticAileron_deg), ...
    'At least one heading case exceeded aileron effort.');
assert(all(commandResults.peakAutomaticRudder_deg <= ...
    req.maximumAutomaticRudder_deg), ...
    'At least one heading case exceeded rudder effort.');
assert(all(commandResults.peakAileronRate_degps <= ...
    req.maximumOperationalAileronRate_degps), ...
    'At least one heading case exceeded the aileron-rate gate.');

%% Nonlinear authority, wraparound, bias and noise captures

operational = simulateHeadingCase(nominalPlant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,controller, ...
    controller.validation.commandStep_deg,0.0,0.0,0.0, ...
    controller.validation.stopTime_s);
largeTurn = simulateHeadingCase(nominalPlant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,controller, ...
    controller.validation.largeCommandStep_deg,0.0,0.0,0.0, ...
    controller.validation.nonlinearStopTime_s);
wrapRight = simulateHeadingCase(nominalPlant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,controller, ...
    controller.validation.wrapRightCommand_deg, ...
    controller.validation.wrapRightInitial_deg,0.0,0.0, ...
    controller.validation.stopTime_s);
wrapLeft = simulateHeadingCase(nominalPlant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,controller, ...
    controller.validation.wrapLeftCommand_deg, ...
    controller.validation.wrapLeftInitial_deg,0.0,0.0, ...
    controller.validation.stopTime_s);
biasCase = simulateHeadingCase(nominalPlant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,controller,0.0,0.0, ...
    headingSensor.biasStress_deg,0.0,controller.validation.stopTime_s);
noiseCase = simulateHeadingCase(nominalPlant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,controller,0.0,0.0,0.0, ...
    headingSensor.noiseStressRms_deg,headingSensor.noiseStopTime_s);

assert(~operational.bankCommandLimitActive && ...
    ~operational.aileronPositionLimitActive && ...
    ~operational.aileronRateLimitActive, ...
    'The operational heading command unexpectedly activated protection.');
assert(largeTurn.bankCommandLimitActive, ...
    'The 90-deg command did not activate bank-command limiting.');
assert(~largeTurn.aileronPositionLimitActive && ...
    ~largeTurn.aileronRateLimitActive && ...
    ~largeTurn.aileronPhysicalLimitActive, ...
    'The large heading command unexpectedly required aileron protection.');
assert(largeTurn.peakBank_deg <= ...
    controller.largeTurn.maximumActualBank_deg, ...
    'The large-turn actual bank exceeded its gate.');
assert(largeTurn.peakRollRate_degps <= ...
    controller.largeTurn.maximumRollRate_degps, ...
    'The large-turn roll rate exceeded its gate.');
assert(largeTurn.peakYawRate_degps <= ...
    controller.largeTurn.maximumYawRate_degps, ...
    'The large-turn yaw rate exceeded its gate.');
assert(largeTurn.peakSideslip_deg <= ...
    controller.largeTurn.maximumSideslip_deg, ...
    'The large-turn sideslip exceeded its gate.');
assert(abs(largeTurn.finalHeadingError_deg) <= ...
    controller.largeTurn.maximumFinalHeadingError_deg, ...
    'The large-turn final heading error exceeded its gate.');
assert(largeTurn.settlingTime_s <= ...
    controller.largeTurn.maximumSettlingTime_s, ...
    'The large-turn settling time exceeded its gate.');

assert(wrapRight.initialLimitedBankCommand_deg > 0 && ...
    wrapLeft.initialLimitedBankCommand_deg < 0, ...
    'The north-crossing wrap cases selected the wrong turn directions.');
assert(abs(wrapRight.finalHeadingError_deg) <= ...
    headingSensor.requirements.maximumWrapCaptureError_deg && ...
    abs(wrapLeft.finalHeadingError_deg) <= ...
    headingSensor.requirements.maximumWrapCaptureError_deg, ...
    'A north-crossing wrap capture failed.');
assert(abs(biasCase.finalMeasuredHeadingError_deg) <= ...
    headingSensor.requirements.maximumMeasuredBiasCaseError_deg, ...
    'The heading-bias case did not null measured heading error.');
assert(abs(abs(biasCase.finalTrueHeadingError_deg)- ...
    headingSensor.biasStress_deg) <= ...
    headingSensor.requirements.maximumBiasResidualMismatch_deg, ...
    'The bias case did not expose the expected true-heading residual.');

noiseRows = noiseCase.t >= headingSensor.noiseDiscardTime_s;
noiseBankRms_deg = sqrt(mean(noiseCase.bank_deg(noiseRows).^2));
noiseAileronRms_deg = sqrt(mean( ...
    noiseCase.automaticAileron_deg(noiseRows).^2));
noiseAileronPeak_deg = max(abs( ...
    noiseCase.automaticAileron_deg(noiseRows)));
assert(noiseBankRms_deg <= ...
    headingSensor.requirements.maximumNoiseBankRms_deg, ...
    'Heading noise drove excessive bank response.');
assert(noiseAileronRms_deg <= ...
    headingSensor.requirements.maximumNoiseAileronRms_deg, ...
    'Heading noise drove excessive aileron RMS.');
assert(noiseAileronPeak_deg <= ...
    headingSensor.requirements.maximumNoiseAileronPeak_deg, ...
    'Heading noise drove excessive peak aileron.');

%% Coordinated-turn analytical reference

Vtrim = baselineData.trim.V_mps;
g = baselineData.constants.g;
standardRate_radps = deg2rad( ...
    controller.turnReference.standardRate_degps);
standardRateBank_deg = rad2deg(atan(Vtrim*standardRate_radps/g));
maximumReferenceTurnRate_degps = rad2deg( ...
    g*tan(deg2rad(controller.bankCommandAuthority_deg))/Vtrim);
maximumReferenceTurnRadius_m = Vtrim^2/( ...
    g*tan(deg2rad(controller.bankCommandAuthority_deg)));
assert(standardRateBank_deg < controller.bankCommandAuthority_deg, ...
    'The frozen bank-command authority cannot support rate-one reference.');

%% Save ledgers

outputFolder = fileparts(mfilename('fullpath'));
linearLedgerFile = fullfile(outputFolder, ...
    'heading_hold_v1_linear_robustness_results.csv');
commandLedgerFile = fullfile(outputFolder, ...
    'heading_hold_v1_command_robustness_results.csv');
delayLedgerFile = fullfile(outputFolder, ...
    'heading_hold_v1_delay_sensitivity_results.csv');
writetable(linearResults,linearLedgerFile);
writetable(commandResults,commandLedgerFile);
writetable(delayResults,delayLedgerFile);

%% Figures

figure('Name','C172S Heading Hold V1 - Robust Captures','Color','w');
tiledlayout(2,2);
nexttile;
plotEnvelope(t,headingHistory_deg);
hold on;
plot(t,mean(headingHistory_deg,2),'LineWidth',1.5);
yline(controller.validation.commandStep_deg,'--','HandleVisibility','off');
grid on;
xlabel('Time (s)'); ylabel('\psi (deg)'); title('30-deg Heading Command');
nexttile;
plotEnvelope(t,bankHistory_deg);
hold on;
plot(t,mean(bankHistory_deg,2),'LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('\phi (deg)'); title('Bank Angle');
nexttile;
plotEnvelope(t,rollRateHistory_degps);
hold on;
plot(t,mean(rollRateHistory_degps,2),'LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('p (deg/s)'); title('Roll Rate');
nexttile;
plotEnvelope(t,sideslipHistory_deg);
hold on;
plot(t,mean(sideslipHistory_deg,2),'LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('\beta (deg)'); title('Sideslip');
sgtitle('Heading Hold V1: 729-Case Command Envelope');

figure('Name','C172S Heading Hold V1 - Logic and Sensor Audit','Color','w');
tiledlayout(2,2);
nexttile;
plot(largeTurn.t,largeTurn.headingDisplay_deg,'LineWidth',1.4);
hold on;
yline(controller.validation.largeCommandStep_deg,'--', ...
    'HandleVisibility','off');
grid on;
xlabel('Time (s)'); ylabel('Heading (deg)');
title('90-deg Protected Acquisition');
nexttile;
plot(largeTurn.t,largeTurn.rawBankCommand_deg,'--','LineWidth',1.2);
hold on;
plot(largeTurn.t,largeTurn.limitedBankCommand_deg,'LineWidth',1.4);
plot(largeTurn.t,largeTurn.bank_deg,'LineWidth',1.4);
grid on;
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Raw bank command','Limited bank command','Bank', ...
    'Location','best');
title('Bank-Command Authority');
nexttile;
plot(wrapRight.t,wrapRight.headingDisplay_deg,'LineWidth',1.4);
hold on;
plot(wrapLeft.t,wrapLeft.headingDisplay_deg,'LineWidth',1.4);
grid on;
xlabel('Time (s)'); ylabel('Wrapped heading (deg)');
legend('350 to 10 deg','10 to 350 deg','Location','best');
title('North-Crossing Wrap Logic');
nexttile;
yyaxis left;
plot(noiseCase.t,noiseCase.bank_deg,'LineWidth',1.0);
ylabel('Bank angle (deg)');
yyaxis right;
plot(noiseCase.t,noiseCase.automaticAileron_deg,'LineWidth',1.0);
ylabel('Automatic aileron (deg)');
grid on;
xlabel('Time (s)'); title('0.5-deg RMS Heading-Noise Stress');
sgtitle('Heading Hold V1: Nonlinear Logic and Sensor Audit');

figure('Name','C172S Heading Hold V1 - Delay Sensitivity','Color','w');
yyaxis left;
plot(1000*delayResults.headingDelay_s, ...
    delayResults.delayPhaseMargin_deg,'-o','LineWidth',1.5);
ylabel('Minimum balanced disk phase margin (deg)');
yyaxis right;
plot(1000*delayResults.headingDelay_s, ...
    delayResults.delayGainMargin_dB,'-s','LineWidth',1.5);
ylabel('Minimum balanced disk gain margin (dB)');
grid on;
xlabel('Pure heading-sensor/processing delay (ms)');
title('Exact Outer-Loop Delay Sensitivity');

%% Console release report

fprintf('\n============================================================\n');
fprintf(' C172S HEADING HOLD V1 - MATLAB DESIGN AND SOURCE AUDIT\n');
fprintf('============================================================\n\n');
fprintf('Controller artifact                  : %s\n', ...
    controller.artifactRevision);
fprintf('Heading-sensor audit                 : %s\n', ...
    headingSensor.artifactRevision);
fprintf('Frozen roll controller               : %s\n', ...
    rollController.artifactRevision);
fprintf('Frozen yaw controller/audit          : %s / %s\n', ...
    yawDamper.artifactRevision,rudder.artifactRevision);
fprintf('Frozen plant                          : %s\n', ...
    baselineData.model.version);
fprintf('Control law                           : phiCmd = sat(Kpsi*wrap(ePsi),+/-20 deg)\n');
fprintf('Kpsi                                  : %.3f rad/rad\n',Kpsi);
fprintf('Heading filter / nominal delay        : %.3f s / %.0f ms\n', ...
    headingSensor.filterTimeConstant_s,1000*headingSensor.nominalDelay_s);
fprintf('Conservative inherited roll delay     : %.0f ms\n', ...
    1000*inheritedRollDelay_s);
fprintf('Bank-command authority                : +/-%.1f deg\n\n', ...
    controller.bankCommandAuthority_deg);

fprintf('Gain-search largest passing Kpsi      : %.3f\n', ...
    gainGrid(selectedGainIndex));
fprintf('Next Kpsi damping / phase / gain      : %.4f / %.2f deg / %.2f dB\n', ...
    gainMinimumDampingRatio(selectedGainIndex+1), ...
    gainMinimumPhaseMargin_deg(selectedGainIndex+1), ...
    gainMinimumGainMargin_dB(selectedGainIndex+1));
fprintf('Stable plant/aileron cases            : %d/%d\n', ...
    sum(linearResults.isStable),height(linearResults));
fprintf('Robust minimum dynamic decay          : %.4f to %.4f 1/s\n', ...
    min(linearResults.minimumDecayRate_per_s), ...
    max(linearResults.minimumDecayRate_per_s));
fprintf('Robust minimum oscillatory damping    : %.4f to %.4f\n', ...
    min(linearResults.minimumOscillatoryDampingRatio), ...
    max(linearResults.minimumOscillatoryDampingRatio));
fprintf('Nominal-delay disk phase margin       : %.2f to %.2f deg\n', ...
    min(linearResults.balancedDiskPhaseMargin_deg), ...
    max(linearResults.balancedDiskPhaseMargin_deg));
fprintf('Nominal-delay minimum gain margin     : %.2f dB\n\n', ...
    min(linearResults.balancedDiskGainMargin_dB));

fprintf('30-deg heading rise time              : %.3f to %.3f s\n', ...
    min(commandResults.riseTime_s),max(commandResults.riseTime_s));
fprintf('30-deg heading overshoot              : %.3f to %.3f %%\n', ...
    min(commandResults.overshoot_pct),max(commandResults.overshoot_pct));
fprintf('30-deg heading settling time          : %.3f to %.3f s\n', ...
    min(commandResults.settlingTime_s),max(commandResults.settlingTime_s));
fprintf('Peak bank / roll rate                 : %.4f / %.4f deg, deg/s\n', ...
    max(commandResults.peakBank_deg), ...
    max(commandResults.peakRollRate_degps));
fprintf('Peak yaw rate / sideslip              : %.4f deg/s / %.4f deg\n', ...
    max(commandResults.peakYawRate_degps), ...
    max(commandResults.peakSideslip_deg));
fprintf('Peak automatic aileron / rudder       : %.4f / %.4f deg\n\n', ...
    max(commandResults.peakAutomaticAileron_deg), ...
    max(commandResults.peakAutomaticRudder_deg));

fprintf('Accepted heading-delay envelope       : 0 to %.0f ms\n', ...
    1000*headingSensor.maximumAcceptedDelay_s);
for iDelay = 1:nDelay
    fprintf('  %3.0f ms heading (%3.0f total): phase %.2f deg, gain %.2f dB\n', ...
        1000*delayResults.headingDelay_s(iDelay), ...
        1000*delayResults.auditedTotalDelay_s(iDelay), ...
        delayResults.delayPhaseMargin_deg(iDelay), ...
        delayResults.delayGainMargin_dB(iDelay));
end

fprintf('\n90-deg bank-command limit active      : %d\n', ...
    largeTurn.bankCommandLimitActive);
fprintf('90-deg peak bank / roll rate          : %.4f deg / %.4f deg/s\n', ...
    largeTurn.peakBank_deg,largeTurn.peakRollRate_degps);
fprintf('90-deg peak yaw rate / sideslip       : %.4f deg/s / %.4f deg\n', ...
    largeTurn.peakYawRate_degps,largeTurn.peakSideslip_deg);
fprintf('90-deg settling / final error         : %.3f s / %.6f deg\n', ...
    largeTurn.settlingTime_s,largeTurn.finalHeadingError_deg);
fprintf('North-wrap final errors, right / left : %.6f / %.6f deg\n', ...
    wrapRight.finalHeadingError_deg,wrapLeft.finalHeadingError_deg);
fprintf('2-deg bias measured / true error      : %.6f / %.6f deg\n', ...
    biasCase.finalMeasuredHeadingError_deg, ...
    biasCase.finalTrueHeadingError_deg);
fprintf('Noise bank / aileron RMS              : %.5f / %.5f deg\n', ...
    noiseBankRms_deg,noiseAileronRms_deg);
fprintf('Noise peak automatic aileron          : %.5f deg\n\n', ...
    noiseAileronPeak_deg);

fprintf('Rate-one reference bank at 110 KTAS   : %.3f deg\n', ...
    standardRateBank_deg);
fprintf('20-deg reference turn rate / radius   : %.3f deg/s / %.1f m\n\n', ...
    maximumReferenceTurnRate_degps,maximumReferenceTurnRadius_m);

fprintf('PASS: Kpsi is the largest passing gain on the declared grid.\n');
fprintf('PASS: every structured plant/actuator heading case is stable.\n');
fprintf('PASS: modal, disk-margin and 30-deg response gates pass.\n');
fprintf('PASS: 90-deg bank authority and north-crossing wrap logic pass.\n');
fprintf('PASS: heading bias behavior is exposed and noise remains bounded.\n');
fprintf(['NOTE: Heading Hold V1 uses shortest-path wrap logic, not Garmin ', ...
    'heading-bug event history.\n']);
fprintf(['NOTE: heading sensing, delay, noise and bias values remain ', ...
    'explicit project assumptions.\n']);
fprintf(['NOTE: time-domain captures omit pure transport delay; exact ', ...
    'delay is audited in frequency response.\n']);
fprintf(['NOTE: this validates an outer loop on provisional Plant V0.1; ', ...
    'it is not aircraft approval.\n\n']);
fprintf('Saved linear ledger                 : %s\n',linearLedgerFile);
fprintf('Saved command ledger                : %s\n',commandLedgerFile);
fprintf('Saved delay ledger                  : %s\n',delayLedgerFile);

results.controller = controller;
results.headingSensor = headingSensor;
results.gainSearch = gainSearchResults;
results.linear = linearResults;
results.sensor = sensorResults;
results.delay = delayResults;
results.command = commandResults;
results.operational = operational;
results.largeTurn = largeTurn;
results.wrapRight = wrapRight;
results.wrapLeft = wrapLeft;
results.bias = biasCase;
results.noise = noiseCase;
results.turnReference.standardRateBank_deg = standardRateBank_deg;
results.turnReference.maximumRate_degps = maximumReferenceTurnRate_degps;
results.turnReference.turnRadiusAtMaximumBank_m = ...
    maximumReferenceTurnRadius_m;
assignin('base','headingHoldV1Results',results);

%% Local functions

function assertDependencies(data,yawDamper,rudder,rollController, ...
    aileron,headingSensor,controller)
assert(strcmp(data.model.version,controller.requiredPlantVersion), ...
    'Heading Hold V1 plant dependency mismatch.');
assert(strcmp(yawDamper.artifactRevision, ...
    controller.requiredYawDamperRevision), ...
    'Heading Hold V1 yaw-controller dependency mismatch.');
assert(strcmp(rudder.artifactRevision, ...
    controller.requiredRudderAuditRevision), ...
    'Heading Hold V1 rudder-audit dependency mismatch.');
assert(strcmp(aileron.artifactRevision, ...
    controller.requiredAileronAuditRevision), ...
    'Heading Hold V1 aileron-audit dependency mismatch.');
assert(strcmp(rollController.artifactRevision, ...
    controller.requiredRollControllerRevision), ...
    'Heading Hold V1 roll-controller dependency mismatch.');
assert(strcmp(headingSensor.artifactRevision, ...
    controller.requiredHeadingSensorRevision), ...
    'Heading Hold V1 heading-sensor dependency mismatch.');
end

function [plants,ledger] = buildHeadingUncertaintyFamily(baseline,controller)
scale = controller.uncertainty.derivativeScaleValues;
inertiaScaleValues = controller.uncertainty.inertiaScaleValues;
ixzValues = controller.uncertainty.IxzNormalizedValues;
n = numel(scale)^4*numel(inertiaScaleValues)*numel(ixzValues);
plants = cell(n,1);
sideForceScale = zeros(n,1);
rollMomentScale = zeros(n,1);
yawMomentScale = zeros(n,1);
aileronEffectivenessScale = zeros(n,1);
inertiaScale = zeros(n,1);
IxzNormalized = zeros(n,1);
caseID = (1:n).';
row = 0;
for sY = scale
    for sL = scale
        for sN = scale
            for sA = scale
                for sI = inertiaScaleValues
                    for eta = ixzValues
                        row = row + 1;
                        data = baseline;
                        data.aero.CY_beta = sY*baseline.aero.CY_beta;
                        data.aero.CY_p = sY*baseline.aero.CY_p;
                        data.aero.CY_r = sY*baseline.aero.CY_r;
                        data.aero.Cl_beta = sL*baseline.aero.Cl_beta;
                        data.aero.Cl_p = sL*baseline.aero.Cl_p;
                        data.aero.Cl_r = sL*baseline.aero.Cl_r;
                        data.aero.Cn_beta = sN*baseline.aero.Cn_beta;
                        data.aero.Cn_p = sN*baseline.aero.Cn_p;
                        data.aero.Cn_r = sN*baseline.aero.Cn_r;
                        data.aero.CY_deltaA = sA*baseline.aero.CY_deltaA;
                        data.aero.Cl_deltaA = sA*baseline.aero.Cl_deltaA;
                        data.aero.Cn_deltaA = sA*baseline.aero.Cn_deltaA;
                        data.inertia.Ixx_kgm2 = ...
                            sI*baseline.inertia.Ixx_kgm2;
                        data.inertia.Izz_kgm2 = ...
                            sI*baseline.inertia.Izz_kgm2;
                        data.inertia.Ixz_kgm2 = eta*sqrt( ...
                            data.inertia.Ixx_kgm2*data.inertia.Izz_kgm2);
                        [~,plants{row}] = ...
                            c172sLateralDirectionalPlant(data);
                        sideForceScale(row) = sY;
                        rollMomentScale(row) = sL;
                        yawMomentScale(row) = sN;
                        aileronEffectivenessScale(row) = sA;
                        inertiaScale(row) = sI;
                        IxzNormalized(row) = eta;
                    end
                end
            end
        end
    end
end
assert(row == n,'Heading uncertainty loop count is incorrect.');
ledger = table(caseID,sideForceScale,rollMomentScale,yawMomentScale, ...
    aileronEffectivenessScale,inertiaScale,IxzNormalized);
end

function [A,B,C] = openHeadingLoopSystem(plant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,tauA,headingFilterTau_s)
% States: beta,p,r,phi,psi,rSensor,xw,deltaR,pSensor,phiSensor,
%         xi,deltaA,filteredBankCommand,headingSensor.
A = zeros(14,14);
B = zeros(14,1);
C = zeros(1,14);
A(1:5,1:5) = plant.A;
A(1:5,8) = plant.B(:,2);
A(1:5,12) = plant.B(:,1);
TsR = rudder.sensor.filterTimeConstant_s;
Tw = yawDamper.washoutTimeConstant_s;
Kr = yawDamper.rudderGain_s;
tauR = rudder.dynamics.timeConstant_s;
TsRoll = aileron.sensor.filterTimeConstant_s;
Kphi = rollController.bankProportionalGain;
Ki = rollController.bankIntegralGain_per_s;
Kp = rollController.rollRateGain_s;
Tcmd = rollController.commandFilterTimeConstant_s;
A(6,3) = 1/TsR;
A(6,6) = -1/TsR;
A(7,6) = 1/Tw;
A(7,7) = -1/Tw;
A(8,6) = -Kr/tauR;
A(8,7) = Kr/tauR;
A(8,8) = -1/tauR;
A(9,2) = 1/TsRoll;
A(9,9) = -1/TsRoll;
A(10,4) = 1/TsRoll;
A(10,10) = -1/TsRoll;
A(11,10) = -1;
A(11,13) = 1;
A(12,9) = -Kp/tauA;
A(12,10) = -Kphi/tauA;
A(12,11) = Ki/tauA;
A(12,12) = -1/tauA;
A(12,13) = Kphi/tauA;
A(13,13) = -1/Tcmd;
B(13) = 1/Tcmd;
A(14,5) = 1/headingFilterTau_s;
A(14,14) = -1/headingFilterTau_s;
C(14) = 1;
end

function Aclosed = headingClosedLoopMatrix(plant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,tauA,headingFilterTau_s,Kpsi)
[Aopen,Bopen,Copen] = openHeadingLoopSystem(plant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,tauA,headingFilterTau_s);
Aclosed = Aopen-Bopen*Kpsi*Copen;
end

function [Aclosed,Bclosed] = headingCommandSystem(plant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,Kpsi)
[Aopen,Bopen,Copen] = openHeadingLoopSystem(plant,yawDamper,rudder, ...
    rollController,aileron,headingSensor, ...
    aileron.dynamics.timeConstant_s,headingSensor.filterTimeConstant_s);
Aclosed = Aopen-Bopen*Kpsi*Copen;
Bclosed = Bopen*Kpsi;
end

function modes = classifyHeadingClosedLoop(Aclosed)
poles = eig(Aclosed);
modes.stable = all(real(poles) < -1.0e-10);
modes.minimumDecayRate_per_s = min(-real(poles));
oscillatory = poles(imag(poles) > 1.0e-8);
assert(~isempty(oscillatory), ...
    'The heading closed loop unexpectedly has no oscillatory modes.');
modes.minimumOscillatoryDampingRatio = min( ...
    -real(oscillatory)./abs(oscillatory));
end

function result = headingLoopMargins(Gheading,w,Kpsi,totalDelay_s)
loop = Kpsi*Gheading.*exp(-1i*w*totalDelay_s);
alpha = abs(2*(1+loop)./(1-loop));
[diskMargin,index] = min(alpha);
result.diskMargin = diskMargin;
result.phaseMargin_deg = 2*atand(diskMargin/2);
if diskMargin >= 2
    result.gainMargin_dB = inf;
else
    result.gainMargin_dB = ...
        20*log10((2+diskMargin)/(2-diskMargin));
end
result.criticalFrequency_radps = w(index);
end

function metrics = positiveStepMetrics(t,response,command)
i10 = find(response >= 0.10*command,1,'first');
i90 = find(response >= 0.90*command,1,'first');
if isempty(i10) || isempty(i90)
    metrics.riseTime_s = inf;
else
    metrics.riseTime_s = t(i90)-t(i10);
end
metrics.overshoot_pct = max(0,100*(max(response)-command)/command);
outside = find(abs(response-command) > 0.02*abs(command));
if isempty(outside)
    metrics.settlingTime_s = 0;
elseif outside(end) == numel(t)
    metrics.settlingTime_s = inf;
else
    metrics.settlingTime_s = t(outside(end)+1);
end
end

function output = simulateHeadingCase(plant,yawDamper,rudder, ...
    rollController,aileron,headingSensor,controller,command_deg, ...
    initialHeading_deg,bias_deg,noiseRms_deg,stopTime_s)
dt = controller.validation.nonlinearTimeStep_s;
t = (0:dt:stopTime_s).';
z = zeros(14,1);
z(5) = deg2rad(initialHeading_deg);
z(14) = z(5)+deg2rad(bias_deg);
history = zeros(numel(t),14);
diagnostic = zeros(numel(t),11);
for k = 1:numel(t)
    history(k,:) = z.';
    [~,diagnostic(k,:)] = headingDerivative(t(k),z,plant,yawDamper, ...
        rudder,rollController,aileron,headingSensor,controller, ...
        command_deg,bias_deg,noiseRms_deg);
    if k < numel(t)
        h = t(k+1)-t(k);
        k1 = headingDerivative(t(k),z,plant,yawDamper,rudder, ...
            rollController,aileron,headingSensor,controller, ...
            command_deg,bias_deg,noiseRms_deg);
        k2 = headingDerivative(t(k)+0.5*h,z+0.5*h*k1,plant, ...
            yawDamper,rudder,rollController,aileron,headingSensor, ...
            controller,command_deg,bias_deg,noiseRms_deg);
        k3 = headingDerivative(t(k)+0.5*h,z+0.5*h*k2,plant, ...
            yawDamper,rudder,rollController,aileron,headingSensor, ...
            controller,command_deg,bias_deg,noiseRms_deg);
        k4 = headingDerivative(t(k)+h,z+h*k3,plant,yawDamper, ...
            rudder,rollController,aileron,headingSensor,controller, ...
            command_deg,bias_deg,noiseRms_deg);
        z = z+(h/6)*(k1+2*k2+2*k3+k4);
    end
end

trueError_deg = rad2deg(wrapAngle(deg2rad(command_deg)-history(:,5)));
measuredError_deg = rad2deg( ...
    wrapAngle(deg2rad(command_deg)-history(:,14)));
settled = abs(trueError_deg) <= ...
    0.02*max(abs(wrapAngle(deg2rad(command_deg-initialHeading_deg))), ...
    deg2rad(1.0))*180/pi;
settlingIndex = findPermanentEntry(settled);
if isempty(settlingIndex)
    settlingTime_s = inf;
else
    settlingTime_s = t(settlingIndex);
end

output.t = t;
output.headingTrue_deg = rad2deg(history(:,5));
output.headingDisplay_deg = mod(output.headingTrue_deg,360);
output.headingSensorDisplay_deg = mod(rad2deg(history(:,14)),360);
output.bank_deg = rad2deg(history(:,4));
output.automaticAileron_deg = rad2deg(history(:,12));
output.automaticRudder_deg = rad2deg(history(:,8));
output.rawBankCommand_deg = rad2deg(diagnostic(:,2));
output.limitedBankCommand_deg = rad2deg(diagnostic(:,3));
output.trueHeadingError_deg = trueError_deg;
output.measuredHeadingError_deg = measuredError_deg;
output.initialLimitedBankCommand_deg = output.limitedBankCommand_deg(1);
output.finalHeadingError_deg = trueError_deg(end);
output.finalTrueHeadingError_deg = trueError_deg(end);
output.finalMeasuredHeadingError_deg = measuredError_deg(end);
output.settlingTime_s = settlingTime_s;
output.peakBank_deg = max(abs(rad2deg(history(:,4))));
output.peakRollRate_degps = max(abs(rad2deg(history(:,2))));
output.peakYawRate_degps = max(abs(rad2deg(history(:,3))));
output.peakSideslip_deg = max(abs(rad2deg(history(:,1))));
output.peakAutomaticAileron_deg = max(abs(rad2deg(history(:,12))));
output.peakAutomaticRudder_deg = max(abs(rad2deg(history(:,8))));
output.peakAileronRate_degps = max(abs(rad2deg(diagnostic(:,7))));
output.finalBank_deg = rad2deg(history(end,4));
output.finalAutomaticAileron_deg = rad2deg(history(end,12));
output.bankCommandLimitActive = any(abs(diagnostic(:,2)- ...
    diagnostic(:,3)) > 1.0e-10);
output.aileronPositionLimitActive = any(abs(diagnostic(:,4)- ...
    diagnostic(:,5)) > 1.0e-10);
output.aileronRateLimitActive = any(abs(diagnostic(:,6)- ...
    diagnostic(:,7)) > 1.0e-10);
output.aileronPhysicalLimitActive = any(abs(diagnostic(:,10)- ...
    diagnostic(:,11)) > 1.0e-10);
end

function [dz,diagnostic] = headingDerivative(t,z,plant,yawDamper, ...
    rudder,rollController,aileron,headingSensor,controller,command_deg, ...
    bias_deg,noiseRms_deg)
noise = deg2rad(sqrt(2)*noiseRms_deg)*sin( ...
    headingSensor.noiseFrequency_radps*t);
headingError = wrapAngle(deg2rad(command_deg)-z(14));
rawBankCommand = controller.headingErrorGain*headingError;
bankLimit = deg2rad(controller.bankCommandAuthority_deg);
limitedBankCommand = clip(rawBankCommand,-bankLimit,bankLimit);
bankError = z(13)-z(10);
rawAileron = rollController.bankProportionalGain*bankError + ...
    rollController.bankIntegralGain_per_s*z(11) - ...
    rollController.rollRateGain_s*z(9);
positionAileron = clip(rawAileron,aileron.authority.lowerLimit_rad, ...
    aileron.authority.upperLimit_rad);
rawAileronRate = (positionAileron-z(12))/ ...
    aileron.dynamics.timeConstant_s;
aileronRateLimit = deg2rad(aileron.dynamics.rateLimit_degps);
aileronRate = clip(rawAileronRate,-aileronRateLimit,aileronRateLimit);
trackingAileron = z(12) + ...
    aileron.dynamics.timeConstant_s*aileronRate;
rawRudder = -yawDamper.rudderGain_s*(z(6)-z(7));
positionRudder = clip(rawRudder,rudder.authority.lowerLimit_rad, ...
    rudder.authority.upperLimit_rad);
rawRudderRate = (positionRudder-z(8))/rudder.dynamics.timeConstant_s;
rudderRateLimit = deg2rad(rudder.dynamics.rateLimit_degps);
rudderRate = clip(rawRudderRate,-rudderRateLimit,rudderRateLimit);
rawTotalAileron = z(12);
totalAileron = clip(rawTotalAileron, ...
    aileron.physicalTravel.lowerLimit_rad, ...
    aileron.physicalTravel.upperLimit_rad);
totalRudder = clip(z(8),rudder.physicalTravel.lowerLimit_rad, ...
    rudder.physicalTravel.upperLimit_rad);

dz = zeros(14,1);
dz(1:5) = plant.A*z(1:5) + plant.B*[totalAileron;totalRudder];
dz(6) = (z(3)-z(6))/rudder.sensor.filterTimeConstant_s;
dz(7) = (z(6)-z(7))/yawDamper.washoutTimeConstant_s;
dz(8) = rudderRate;
dz(9) = (z(2)-z(9))/aileron.sensor.filterTimeConstant_s;
dz(10) = (z(4)-z(10))/aileron.sensor.filterTimeConstant_s;
dz(11) = bankError + (trackingAileron-rawAileron)/( ...
    rollController.bankIntegralGain_per_s* ...
    aileron.antiWindup.trackingTimeConstant_s);
dz(12) = aileronRate;
dz(13) = (limitedBankCommand-z(13))/ ...
    rollController.commandFilterTimeConstant_s;
dz(14) = (z(5)+deg2rad(bias_deg)+noise-z(14))/ ...
    headingSensor.filterTimeConstant_s;

diagnostic = [headingError,rawBankCommand,limitedBankCommand, ...
    rawAileron,positionAileron,rawAileronRate,aileronRate, ...
    rawRudder,rudderRate,rawTotalAileron,totalAileron];
end

function value = wrapAngle(value)
value = atan2(sin(value),cos(value));
end

function value = clip(value,lowerLimit,upperLimit)
value = min(max(value,lowerLimit),upperLimit);
end

function index = findPermanentEntry(condition)
index = [];
lastFalse = find(~condition,1,'last');
if isempty(lastFalse)
    index = 1;
elseif lastFalse < numel(condition)
    index = lastFalse+1;
end
end

function plotEnvelope(t,y)
lower = min(y,[],2);
upper = max(y,[],2);
fill([t;flipud(t)],[lower;flipud(upper)], ...
    [0.80,0.88,1.00],'EdgeColor','none','FaceAlpha',0.55);
end
