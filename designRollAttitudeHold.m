%% C172S Roll-Attitude Hold V1 - protected MATLAB design and audit
%
% Frozen dependencies:
%   * Lateral-Directional Plant V0.1
%   * Yaw Damper V1 controller, artifact YD-V1-MATLAB-R2
%   * Yaw Damper V2 actuator audit, artifact YD-V2-AUDIT-MATLAB-R1
%
% This stage adds a bank-angle PI loop, roll-rate damping, filtered roll
% sensors, command shaping, a protected aileron actuator and anti-windup.
% The frozen protected yaw damper remains active in every analysis case.
% No existing controller or plant artifact is modified by this script.

clc;
clear;
close all;

baselineData = c172sLateralDirectionalData();
yawDamper = c172sYawDamperData();
rudderActuator = c172sRudderActuatorData();
controller = c172sRollAttitudeControllerData();
aileronActuator = c172sAileronActuatorData();
[~,nominalPlant] = c172sLateralDirectionalPlant(baselineData);

assert(exist('ss','file') == 2 && exist('lsim','file') == 2, ...
    'Control System Toolbox is required for this design audit.');
assert(strcmp(baselineData.model.version,controller.requiredPlantVersion), ...
    'Roll-controller plant dependency mismatch.');
assert(strcmp(yawDamper.artifactRevision, ...
    controller.requiredYawDamperRevision), ...
    'The frozen YD-V1-MATLAB-R2 controller is required.');
assert(strcmp(rudderActuator.artifactRevision, ...
    controller.requiredRudderAuditRevision), ...
    'The frozen YD-V2-AUDIT-MATLAB-R1 actuator audit is required.');
assert(strcmp(aileronActuator.artifactRevision, ...
    controller.requiredAileronAuditRevision), ...
    'The declared aileron actuator audit is required.');
assert(yawDamper.feedbackSign == -1.0, ...
    'The frozen yaw damper must retain negative feedback.');
assert(aileronActuator.authority.upperLimit_rad < ...
    aileronActuator.physicalTravel.upperLimit_rad, ...
    'Automatic aileron authority must remain inside the hard stop.');

Kphi = controller.bankProportionalGain;
Ki = controller.bankIntegralGain_per_s;
Kp = controller.rollRateGain_s;
Tcmd = controller.commandFilterTimeConstant_s;
Kr = yawDamper.rudderGain_s;
Tw = yawDamper.washoutTimeConstant_s;
TsRudder = rudderActuator.sensor.filterTimeConstant_s;
tauRudder = rudderActuator.dynamics.timeConstant_s;
TsRoll = aileronActuator.sensor.filterTimeConstant_s;
tauAileronCases_s = aileronActuator.dynamics.timeConstantCases_s;

[plants,plantLedger] = buildRollUncertaintyFamily( ...
    baselineData,controller);
nPlant = numel(plants);
assert(nPlant == controller.uncertainty.caseCount, ...
    'The 729-case roll-plant ledger is incomplete.');

%% Precompute yaw-protected plant frequency responses

w = logspace(log10(controller.validation. ...
    diskMarginFrequencyRange_radps(1)), ...
    log10(controller.validation.diskMarginFrequencyRange_radps(2)), ...
    controller.validation.diskMarginGridPointCount).';
Gp = complex(zeros(numel(w),nPlant));
Gphi = complex(zeros(numel(w),nPlant));
for iPlant = 1:nPlant
    [Gp(:,iPlant),Gphi(:,iPlant)] = yawProtectedRollResponse( ...
        plants{iPlant},yawDamper,rudderActuator,w);
end

%% Reproducible Kp search

KpGrid = (controller.design.rollRateGainSearchRange_s(1): ...
    controller.design.rollRateGainIncrement_s: ...
    controller.design.rollRateGainSearchRange_s(2)).';
nGain = numel(KpGrid);
gainAllStable = false(nGain,1);
gainMinimumDecayRate_per_s = zeros(nGain,1);
gainMinimumDampingRatio = zeros(nGain,1);
gainMinimumPhaseMargin_deg = zeros(nGain,1);
gainMinimumGainMargin_dB = zeros(nGain,1);
gainPass = false(nGain,1);

acceptedDelay = aileronActuator.sensor.maximumAcceptedDelay_s;
acceptedDelayCases = aileronActuator.sensor.delayCases_s( ...
    aileronActuator.sensor.delayCases_s <= acceptedDelay + 1.0e-12);
for iGain = 1:nGain
    candidateKp = KpGrid(iGain);
    minDecay = inf;
    minDamping = inf;
    minPhase = inf;
    minGain = inf;
    allStable = true;
    for iPlant = 1:nPlant
        for iTau = 1:numel(tauAileronCases_s)
            tauA = tauAileronCases_s(iTau);
            Acl = rollClosedLoopMatrix(plants{iPlant},yawDamper, ...
                rudderActuator,Kphi,Ki,candidateKp,TsRoll,tauA);
            modes = classifyRollClosedLoop(Acl);
            allStable = allStable && modes.dynamicStable;
            minDecay = min(minDecay,modes.minimumDecayRate_per_s);
            minDamping = min(minDamping, ...
                modes.minimumOscillatoryDampingRatio);
            for candidateDelay = acceptedDelayCases
                marginResult = rollLoopMargins(Gp(:,iPlant), ...
                    Gphi(:,iPlant),w,Kphi,Ki,candidateKp,TsRoll,tauA, ...
                    candidateDelay);
                minPhase = min(minPhase,marginResult.phaseMargin_deg);
                minGain = min(minGain,marginResult.gainMargin_dB);
            end
        end
    end
    gainAllStable(iGain) = allStable;
    gainMinimumDecayRate_per_s(iGain) = minDecay;
    gainMinimumDampingRatio(iGain) = minDamping;
    gainMinimumPhaseMargin_deg(iGain) = minPhase;
    gainMinimumGainMargin_dB(iGain) = minGain;
    gainPass(iGain) = allStable && ...
        minDecay >= controller.design.minimumDynamicDecayRate_per_s && ...
        minDamping >= controller.design. ...
            minimumOscillatoryDampingRatio && ...
        minPhase >= controller.design. ...
            minimumBalancedDiskPhaseMargin_deg && ...
        minGain >= controller.design.minimumBalancedDiskGainMargin_dB;
end

gainSearchResults = table(KpGrid,gainAllStable, ...
    gainMinimumDecayRate_per_s,gainMinimumDampingRatio, ...
    gainMinimumPhaseMargin_deg,gainMinimumGainMargin_dB,gainPass);
firstPassingGain = find(gainPass,1,'first');
assert(~isempty(firstPassingGain), ...
    'No Kp candidate satisfies the declared complete-family gates.');
assert(abs(KpGrid(firstPassingGain)-Kp) < 1.0e-12, ...
    'The frozen Kp is not the first passing point on the declared grid.');

%% Complete 729-plant x 3-aileron-time-constant linear audit

nTau = numel(tauAileronCases_s);
nLinear = nPlant*nTau;
caseID = (1:nLinear).';
plantCaseID = zeros(nLinear,1);
aileronTau_s = zeros(nLinear,1);
isDynamicStable = false(nLinear,1);
neutralHeadingPoleMagnitude = zeros(nLinear,1);
minimumDynamicDecayRate_per_s = zeros(nLinear,1);
minimumOscillatoryDampingRatio = zeros(nLinear,1);
balancedDiskMargin = zeros(nLinear,1);
balancedDiskPhaseMargin_deg = zeros(nLinear,1);
balancedDiskGainMargin_dB = zeros(nLinear,1);
diskMarginCriticalFrequency_radps = zeros(nLinear,1);

row = 0;
for iPlant = 1:nPlant
    for iTau = 1:nTau
        row = row + 1;
        tauA = tauAileronCases_s(iTau);
        Acl = rollClosedLoopMatrix(plants{iPlant},yawDamper, ...
            rudderActuator,Kphi,Ki,Kp,TsRoll,tauA);
        modes = classifyRollClosedLoop(Acl);
        margins = rollLoopMargins(Gp(:,iPlant),Gphi(:,iPlant),w, ...
            Kphi,Ki,Kp,TsRoll,tauA, ...
            aileronActuator.sensor.nominalDelay_s);

        plantCaseID(row) = iPlant;
        aileronTau_s(row) = tauA;
        isDynamicStable(row) = modes.dynamicStable;
        neutralHeadingPoleMagnitude(row) = abs(modes.headingPole);
        minimumDynamicDecayRate_per_s(row) = ...
            modes.minimumDecayRate_per_s;
        minimumOscillatoryDampingRatio(row) = ...
            modes.minimumOscillatoryDampingRatio;
        balancedDiskMargin(row) = margins.diskMargin;
        balancedDiskPhaseMargin_deg(row) = margins.phaseMargin_deg;
        balancedDiskGainMargin_dB(row) = margins.gainMargin_dB;
        diskMarginCriticalFrequency_radps(row) = ...
            margins.criticalFrequency_radps;
    end
end

linearResults = table(caseID,plantCaseID,aileronTau_s, ...
    isDynamicStable,neutralHeadingPoleMagnitude, ...
    minimumDynamicDecayRate_per_s,minimumOscillatoryDampingRatio, ...
    balancedDiskMargin,balancedDiskPhaseMargin_deg, ...
    balancedDiskGainMargin_dB,diskMarginCriticalFrequency_radps);

req = controller.requirements;
assert(all(linearResults.isDynamicStable), ...
    'At least one roll plant/actuator case is dynamically unstable.');
assert(all(linearResults.neutralHeadingPoleMagnitude < 1.0e-9), ...
    'The neutral heading kinematic pole was not preserved.');
assert(all(linearResults.minimumDynamicDecayRate_per_s >= ...
    req.minimumDynamicDecayRate_per_s), ...
    'At least one case failed the dynamic decay-rate gate.');
assert(all(linearResults.minimumOscillatoryDampingRatio >= ...
    req.minimumOscillatoryDampingRatio), ...
    'At least one case failed the oscillatory damping gate.');
assert(all(linearResults.balancedDiskPhaseMargin_deg >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'At least one nominal-delay case failed the disk phase gate.');
assert(all(linearResults.balancedDiskGainMargin_dB >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'At least one nominal-delay case failed the disk gain gate.');

%% Roll-sensor filter sensitivity at the nominal plant

sensorCases_s = aileronActuator.sensor.filterTimeConstantCases_s;
nSensor = numel(sensorCases_s)*nTau;
sensorFilterTau_s = zeros(nSensor,1);
sensorAileronTau_s = zeros(nSensor,1);
sensorStable = false(nSensor,1);
sensorMinimumDecayRate_per_s = zeros(nSensor,1);
sensorMinimumDampingRatio = zeros(nSensor,1);
sensorPhaseMargin_deg = zeros(nSensor,1);
sensorGainMargin_dB = zeros(nSensor,1);
row = 0;
[GpNom,GphiNom] = yawProtectedRollResponse( ...
    nominalPlant,yawDamper,rudderActuator,w);
for iSensor = 1:numel(sensorCases_s)
    for iTau = 1:nTau
        row = row + 1;
        currentTs = sensorCases_s(iSensor);
        currentTau = tauAileronCases_s(iTau);
        Acl = rollClosedLoopMatrix(nominalPlant,yawDamper, ...
            rudderActuator,Kphi,Ki,Kp,currentTs,currentTau);
        modes = classifyRollClosedLoop(Acl);
        margins = rollLoopMargins(GpNom,GphiNom,w,Kphi,Ki,Kp, ...
            currentTs,currentTau,aileronActuator.sensor.nominalDelay_s);
        sensorFilterTau_s(row) = currentTs;
        sensorAileronTau_s(row) = currentTau;
        sensorStable(row) = modes.dynamicStable;
        sensorMinimumDecayRate_per_s(row) = ...
            modes.minimumDecayRate_per_s;
        sensorMinimumDampingRatio(row) = ...
            modes.minimumOscillatoryDampingRatio;
        sensorPhaseMargin_deg(row) = margins.phaseMargin_deg;
        sensorGainMargin_dB(row) = margins.gainMargin_dB;
    end
end
sensorResults = table(sensorFilterTau_s,sensorAileronTau_s, ...
    sensorStable,sensorMinimumDecayRate_per_s, ...
    sensorMinimumDampingRatio,sensorPhaseMargin_deg, ...
    sensorGainMargin_dB);
assert(all(sensorResults.sensorStable), ...
    'A roll-sensor sensitivity case is unstable.');
assert(all(sensorResults.sensorMinimumDecayRate_per_s >= ...
    req.minimumDynamicDecayRate_per_s), ...
    'A sensor-filter sensitivity case failed the decay gate.');
assert(all(sensorResults.sensorMinimumDampingRatio >= ...
    req.minimumOscillatoryDampingRatio), ...
    'A sensor-filter sensitivity case failed the damping gate.');
assert(all(sensorResults.sensorPhaseMargin_deg >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'A sensor-filter sensitivity case failed the phase gate.');
assert(all(sensorResults.sensorGainMargin_dB >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'A sensor-filter sensitivity case failed the gain gate.');

%% Exact-delay sensitivity across all plants and aileron time constants

delayCases_s = aileronActuator.sensor.delayCases_s;
nDelay = nPlant*nTau*numel(delayCases_s);
delayPlantCaseID = zeros(nDelay,1);
delayAileronTau_s = zeros(nDelay,1);
sensorDelay_s = zeros(nDelay,1);
delayDiskMargin = zeros(nDelay,1);
delayPhaseMargin_deg = zeros(nDelay,1);
delayGainMargin_dB = zeros(nDelay,1);
delayCriticalFrequency_radps = zeros(nDelay,1);
row = 0;
for iPlant = 1:nPlant
    for iTau = 1:nTau
        for iDelay = 1:numel(delayCases_s)
            row = row + 1;
            tauA = tauAileronCases_s(iTau);
            delay = delayCases_s(iDelay);
            margins = rollLoopMargins(Gp(:,iPlant),Gphi(:,iPlant),w, ...
                Kphi,Ki,Kp,TsRoll,tauA,delay);
            delayPlantCaseID(row) = iPlant;
            delayAileronTau_s(row) = tauA;
            sensorDelay_s(row) = delay;
            delayDiskMargin(row) = margins.diskMargin;
            delayPhaseMargin_deg(row) = margins.phaseMargin_deg;
            delayGainMargin_dB(row) = margins.gainMargin_dB;
            delayCriticalFrequency_radps(row) = ...
                margins.criticalFrequency_radps;
        end
    end
end
delayResults = table(delayPlantCaseID,delayAileronTau_s,sensorDelay_s, ...
    delayDiskMargin,delayPhaseMargin_deg,delayGainMargin_dB, ...
    delayCriticalFrequency_radps);
acceptedRows = delayResults.sensorDelay_s <= ...
    aileronActuator.sensor.maximumAcceptedDelay_s + 1.0e-12;
assert(all(delayResults.delayPhaseMargin_deg(acceptedRows) >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'The accepted exact-delay envelope failed the disk phase gate.');
assert(all(delayResults.delayGainMargin_dB(acceptedRows) >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'The accepted exact-delay envelope failed the disk gain gate.');
outsideRows = ~acceptedRows;
assert(any(delayResults.delayPhaseMargin_deg(outsideRows) < ...
    req.minimumBalancedDiskPhaseMargin_deg | ...
    delayResults.delayGainMargin_dB(outsideRows) < ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'The declared delay envelope is unnecessarily conservative.');

%% 729-case linear command-response audit at nominal actuator settings

t = (0:controller.validation.timeStep_s: ...
    controller.validation.stopTime_s).';
command_rad = deg2rad(controller.validation.commandStep_deg);
nTime = numel(t);
phiHistory = zeros(nTime,nPlant);
pHistory = zeros(nTime,nPlant);
autoAileronHistory = zeros(nTime,nPlant);
betaHistory = zeros(nTime,nPlant);
riseTime_s = zeros(nPlant,1);
overshoot_pct = zeros(nPlant,1);
settlingTime_s = zeros(nPlant,1);
finalBankError_deg = zeros(nPlant,1);
peakRollRate_degps = zeros(nPlant,1);
peakAutomaticAileron_deg = zeros(nPlant,1);
peakAutomaticRudder_deg = zeros(nPlant,1);
peakSideslip_deg = zeros(nPlant,1);
peakYawRate_degps = zeros(nPlant,1);
peakAileronRate_degps = zeros(nPlant,1);

for iPlant = 1:nPlant
    [Acommand,Bcommand] = rollCommandSystem(plants{iPlant}, ...
        yawDamper,rudderActuator,controller,aileronActuator);
    z = lsim(ss(Acommand,Bcommand,eye(13),zeros(13,1)), ...
        command_rad*ones(size(t)),t);
    phi = z(:,4);
    p = z(:,2);
    beta = z(:,1);
    r = z(:,3);
    deltaR = z(:,8);
    deltaA = z(:,12);
    rawAileron = Kphi*(z(:,13)-z(:,10)) + Ki*z(:,11) - Kp*z(:,9);
    deltaARate = (rawAileron-deltaA)/ ...
        aileronActuator.dynamics.timeConstant_s;
    step = positiveStepMetrics(t,phi,command_rad);

    phiHistory(:,iPlant) = phi;
    pHistory(:,iPlant) = p;
    autoAileronHistory(:,iPlant) = deltaA;
    betaHistory(:,iPlant) = beta;
    riseTime_s(iPlant) = step.riseTime_s;
    overshoot_pct(iPlant) = step.overshoot_pct;
    settlingTime_s(iPlant) = step.settlingTime_s;
    finalBankError_deg(iPlant) = abs(rad2deg(command_rad-phi(end)));
    peakRollRate_degps(iPlant) = max(abs(rad2deg(p)));
    peakAutomaticAileron_deg(iPlant) = max(abs(rad2deg(deltaA)));
    peakAutomaticRudder_deg(iPlant) = max(abs(rad2deg(deltaR)));
    peakSideslip_deg(iPlant) = max(abs(rad2deg(beta)));
    peakYawRate_degps(iPlant) = max(abs(rad2deg(r)));
    peakAileronRate_degps(iPlant) = max(abs(rad2deg(deltaARate)));
end

commandResults = [plantLedger,table(riseTime_s,overshoot_pct, ...
    settlingTime_s,finalBankError_deg,peakRollRate_degps, ...
    peakAutomaticAileron_deg,peakAutomaticRudder_deg, ...
    peakSideslip_deg,peakYawRate_degps,peakAileronRate_degps)];

assert(all(commandResults.riseTime_s >= req.minimumRiseTime_s & ...
    commandResults.riseTime_s <= req.maximumRiseTime_s), ...
    'At least one command case failed the rise-time gate.');
assert(all(commandResults.overshoot_pct <= ...
    req.maximumStepOvershoot_pct), ...
    'At least one command case failed the overshoot gate.');
assert(all(commandResults.settlingTime_s <= ...
    req.maximumSettlingTime_s), ...
    'At least one command case failed the settling-time gate.');
assert(all(commandResults.finalBankError_deg <= ...
    req.maximumFinalBankError_deg), ...
    'At least one command case failed the final-error gate.');
assert(all(commandResults.peakRollRate_degps <= ...
    req.maximumRollRate_degps), ...
    'At least one command case exceeded the roll-rate gate.');
assert(all(commandResults.peakAutomaticAileron_deg <= ...
    req.maximumAutomaticAileron_deg), ...
    'At least one command case exceeded the automatic-aileron gate.');
assert(all(commandResults.peakAutomaticRudder_deg <= ...
    req.maximumAutomaticRudder_deg), ...
    'At least one command case exceeded the automatic-rudder gate.');
assert(all(commandResults.peakSideslip_deg <= ...
    req.maximumSideslip_deg), ...
    'At least one command case exceeded the sideslip gate.');
assert(all(commandResults.peakYawRate_degps <= ...
    req.maximumYawRate_degps), ...
    'At least one command case exceeded the yaw-rate gate.');
assert(all(commandResults.peakAileronRate_degps <= ...
    aileronActuator.requirements.maximumOperationalActuatorRate_degps), ...
    'At least one command case exceeded the operational servo-rate gate.');

%% Nominal regulation, nonlinear protection and deterministic noise

[AcommandNom,BcommandNom] = rollCommandSystem(nominalPlant, ...
    yawDamper,rudderActuator,controller,aileronActuator);
initialState = zeros(13,1);
initialState(4) = deg2rad(controller.validation.initialBankDisturbance_deg);
initialResponse = initial(ss(AcommandNom,zeros(13,1),eye(13), ...
    zeros(13,1)),initialState,t);
initialFinalBank_deg = abs(rad2deg(initialResponse(end,4)));

Bpilot = zeros(13,1);
Bpilot(1:5) = nominalPlant.B(:,1);
biasResponse = lsim(ss(AcommandNom,Bpilot,eye(13),zeros(13,1)), ...
    deg2rad(controller.validation.constantAileronBias_deg)*ones(size(t)),t);
biasFinalBank_deg = abs(rad2deg(biasResponse(end,4)));
assert(initialFinalBank_deg <= req.maximumRegulationResidual_deg, ...
    'The nominal initial-bank regulation residual is excessive.');
assert(biasFinalBank_deg <= req.maximumRegulationResidual_deg, ...
    'The nominal constant-aileron-bias regulation residual is excessive.');

normalCase = simulateProtectedRollCase(nominalPlant,yawDamper, ...
    rudderActuator,controller,aileronActuator, ...
    controller.validation.commandStep_deg,0.0,true,false);
stressCase = simulateProtectedRollCase(nominalPlant,yawDamper, ...
    rudderActuator,controller,aileronActuator,0.0, ...
    aileronActuator.stress.initialBank_deg,true,false);
stressNoAntiWindup = simulateProtectedRollCase(nominalPlant,yawDamper, ...
    rudderActuator,controller,aileronActuator,0.0, ...
    aileronActuator.stress.initialBank_deg,false,false);
noiseCase = simulateProtectedRollCase(nominalPlant,yawDamper, ...
    rudderActuator,controller,aileronActuator,0.0,0.0,true,true);

assert(~normalCase.positionLimitActive && ~normalCase.rateLimitActive, ...
    'The operational command unexpectedly activates aileron protection.');
assert(abs(controller.validation.commandStep_deg-normalCase.finalBank_deg) <= ...
    req.maximumFinalBankError_deg, ...
    'The nonlinear operational command failed the final-error gate.');
assert(normalCase.peakAutomaticAileron_deg <= ...
    req.maximumAutomaticAileron_deg, ...
    'The nonlinear operational command exceeded aileron effort.');
assert(stressCase.positionLimitActive && stressCase.rateLimitActive, ...
    'The 30-deg logic stress did not activate both protection layers.');
assert(stressCase.peakAutomaticAileron_deg <= ...
    aileronActuator.authority.rightWingDown_deg + 1.0e-8, ...
    'The nonlinear automatic-aileron authority was violated.');
assert(stressCase.peakAileronRate_degps <= ...
    aileronActuator.dynamics.rateLimit_degps + 1.0e-8, ...
    'The nonlinear aileron-rate limit was violated.');
assert(stressCase.peakTotalAileron_deg <= ...
    aileronActuator.physicalTravel.equivalentMagnitude_deg + 1.0e-8, ...
    'The nonlinear physical aileron hard stop was violated.');
assert(stressCase.recoveryTime_s <= ...
    aileronActuator.stress.maximumRecoveryTime_s, ...
    'The logic-stress bank recovery is too slow.');
assert(abs(stressCase.finalBank_deg) <= ...
    aileronActuator.stress.maximumFinalBank_deg, ...
    'The logic-stress final bank residual is excessive.');
assert(abs(stressCase.finalAutomaticAileron_deg) <= ...
    aileronActuator.requirements.maximumReleasedAileron_deg, ...
    'The automatic aileron did not release after the logic stress.');
antiWindupIntegratorRatio = stressCase.peakIntegratorMagnitude/ ...
    stressNoAntiWindup.peakIntegratorMagnitude;
assert(antiWindupIntegratorRatio <= ...
    aileronActuator.stress.maximumAntiWindupIntegratorRatio, ...
    'Back-calculation did not reduce peak integrator accumulation enough.');

noiseRows = noiseCase.t >= aileronActuator.noise.discardTime_s;
noiseAileronRms_deg = sqrt(mean( ...
    noiseCase.automaticAileron_deg(noiseRows).^2));
noiseAileronPeak_deg = max(abs( ...
    noiseCase.automaticAileron_deg(noiseRows)));
assert(noiseAileronRms_deg <= ...
    aileronActuator.requirements.maximumNoiseActuatorRms_deg, ...
    'Deterministic roll-sensor noise exceeded the RMS gate.');
assert(noiseAileronPeak_deg <= ...
    aileronActuator.requirements.maximumNoiseActuatorPeak_deg, ...
    'Deterministic roll-sensor noise exceeded the peak gate.');

%% Save ledgers

linearLedgerFile = fullfile(pwd, ...
    'roll_attitude_hold_v1_linear_robustness_results.csv');
commandLedgerFile = fullfile(pwd, ...
    'roll_attitude_hold_v1_command_robustness_results.csv');
delayLedgerFile = fullfile(pwd, ...
    'roll_attitude_hold_v1_delay_sensitivity_results.csv');
writetable(linearResults,linearLedgerFile);
writetable(commandResults,commandLedgerFile);
writetable(delayResults,delayLedgerFile);

%% Figures

nominalIndex = find(plantLedger.sideForceScale == 1.0 & ...
    plantLedger.rollMomentScale == 1.0 & ...
    plantLedger.yawMomentScale == 1.0 & ...
    plantLedger.aileronEffectivenessScale == 1.0 & ...
    plantLedger.inertiaScale == 1.0 & ...
    plantLedger.IxzNormalized == 0.0,1,'first');

figure('Color','w','Name','C172S Roll-Attitude Hold V1 - Robust Captures');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
plotEnvelope(t,rad2deg(phiHistory)); hold on;
plot(t,rad2deg(phiHistory(:,nominalIndex)),'LineWidth',1.8);
yline(controller.validation.commandStep_deg,'--'); grid on;
xlabel('Time (s)'); ylabel('\phi (deg)'); title('10-deg Bank Command');
nexttile;
plotEnvelope(t,rad2deg(pHistory)); hold on;
plot(t,rad2deg(pHistory(:,nominalIndex)),'LineWidth',1.8); grid on;
xlabel('Time (s)'); ylabel('p (deg/s)'); title('Roll Rate');
nexttile;
plotEnvelope(t,rad2deg(autoAileronHistory)); hold on;
plot(t,rad2deg(autoAileronHistory(:,nominalIndex)),'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('\delta_a (deg)');
title('Automatic Aileron');
nexttile;
plotEnvelope(t,rad2deg(betaHistory)); hold on;
plot(t,rad2deg(betaHistory(:,nominalIndex)),'LineWidth',1.8); grid on;
xlabel('Time (s)'); ylabel('\beta (deg)'); title('Sideslip');
sgtitle('Roll-Attitude Hold V1: 729-Case Command Envelope');

figure('Color','w','Name','C172S Roll-Attitude Hold V1 - Protection');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
plot(normalCase.t,normalCase.commandFiltered_deg,'--','LineWidth',1.4);
hold on; plot(normalCase.t,normalCase.bank_deg,'LineWidth',1.6); grid on;
xlabel('Time (s)'); ylabel('Bank angle (deg)');
legend('Filtered command','Bank','Location','best');
title('Protected Operational Command');
nexttile;
plot(stressCase.t,stressCase.bank_deg,'LineWidth',1.6); hold on;
yline(aileronActuator.stress.recoveryThreshold_deg,'--');
yline(-aileronActuator.stress.recoveryThreshold_deg,'--'); grid on;
xlabel('Time (s)'); ylabel('Bank angle (deg)'); title('30-deg Logic Stress');
nexttile;
yyaxis left;
plot(stressCase.t,stressCase.automaticAileron_deg,'LineWidth',1.5);
ylabel('\delta_a (deg)');
yyaxis right;
plot(stressCase.t,stressCase.aileronRate_degps,'LineWidth',1.2);
ylabel('d\delta_a/dt (deg/s)'); grid on; xlabel('Time (s)');
title('Stress-Case Aileron Protection');
nexttile;
plot(noiseCase.t,noiseCase.automaticAileron_deg,'LineWidth',0.8); grid on;
xlabel('Time (s)'); ylabel('\delta_a (deg)');
title('Deterministic Roll-Sensor Noise');
sgtitle('Roll-Attitude Hold V1: Nonlinear Protection Captures');

summaryDelay_s = delayCases_s(:);
summaryMinimumPhaseMargin_deg = zeros(size(summaryDelay_s));
summaryMinimumGainMargin_dB = zeros(size(summaryDelay_s));
for iDelay = 1:numel(summaryDelay_s)
    theseRows = abs(delayResults.sensorDelay_s-summaryDelay_s(iDelay)) < ...
        1.0e-12;
    summaryMinimumPhaseMargin_deg(iDelay) = min( ...
        delayResults.delayPhaseMargin_deg(theseRows));
    summaryMinimumGainMargin_dB(iDelay) = min( ...
        delayResults.delayGainMargin_dB(theseRows));
end
figure('Color','w','Name','C172S Roll-Attitude Hold V1 - Delay');
yyaxis left;
plot(1000*summaryDelay_s,summaryMinimumPhaseMargin_deg, ...
    '-o','LineWidth',1.7);
ylabel('Minimum balanced disk phase margin (deg)');
yline(req.minimumBalancedDiskPhaseMargin_deg,'--','Phase gate');
yyaxis right;
plot(1000*summaryDelay_s,summaryMinimumGainMargin_dB, ...
    '-s','LineWidth',1.7);
ylabel('Minimum balanced disk gain margin (dB)');
yline(req.minimumBalancedDiskGainMargin_dB,'--','Gain gate');
grid on; xlabel('Pure roll-sensor/processing delay (ms)');
title('Exact Frequency-Response Delay Sensitivity');

%% Console release report

fprintf('\n============================================================\n');
fprintf(' C172S ROLL-ATTITUDE HOLD V1 - MATLAB DESIGN AND AUDIT\n');
fprintf('============================================================\n\n');
fprintf('Controller artifact                  : %s\n', ...
    controller.artifactRevision);
fprintf('Frozen yaw controller                : %s\n', ...
    yawDamper.artifactRevision);
fprintf('Frozen rudder actuator audit          : %s\n', ...
    rudderActuator.artifactRevision);
fprintf('Aileron/source audit                  : %s\n', ...
    aileronActuator.artifactRevision);
fprintf('Frozen plant                          : %s\n', ...
    baselineData.model.version);
fprintf('Control law                           : deltaA = Kphi*ePhi + Ki*xi - Kp*pSensor\n');
fprintf('Kphi / Ki / Kp                        : %.3f / %.3f 1/s / %.3f s\n', ...
    Kphi,Ki,Kp);
fprintf('Command filter / bank authority       : %.2f s / +/-%.1f deg\n', ...
    Tcmd,controller.commandAuthority.rightWingDown_deg);
fprintf('Aileron tau / authority / rate        : %.3f s / +/-%.1f deg / +/-%.1f deg/s\n', ...
    aileronActuator.dynamics.timeConstant_s, ...
    aileronActuator.authority.rightWingDown_deg, ...
    aileronActuator.dynamics.rateLimit_degps);
fprintf('Equivalent aileron hard stop          : +/-%.1f deg\n', ...
    aileronActuator.physicalTravel.equivalentMagnitude_deg);
fprintf('Roll filter / nominal pure delay      : %.3f s / %.0f ms\n\n', ...
    TsRoll,1000*aileronActuator.sensor.nominalDelay_s);

fprintf('Gain-search first passing Kp          : %.3f s\n',KpGrid(firstPassingGain));
if firstPassingGain > 1
    fprintf('Previous Kp phase / gain margin       : %.2f deg / %.2f dB\n', ...
        gainMinimumPhaseMargin_deg(firstPassingGain-1), ...
        gainMinimumGainMargin_dB(firstPassingGain-1));
end
fprintf('Stable plant/aileron cases            : %d/%d\n', ...
    sum(linearResults.isDynamicStable),height(linearResults));
fprintf('Robust minimum dynamic decay          : %.4f to %.4f 1/s\n', ...
    min(linearResults.minimumDynamicDecayRate_per_s), ...
    max(linearResults.minimumDynamicDecayRate_per_s));
fprintf('Robust minimum oscillatory damping    : %.4f to %.4f\n', ...
    min(linearResults.minimumOscillatoryDampingRatio), ...
    max(linearResults.minimumOscillatoryDampingRatio));
fprintf('Nominal-delay disk phase margin       : %.2f to %.2f deg\n', ...
    min(linearResults.balancedDiskPhaseMargin_deg), ...
    max(linearResults.balancedDiskPhaseMargin_deg));
fprintf('Nominal-delay minimum gain margin     : %.2f dB\n\n', ...
    min(linearResults.balancedDiskGainMargin_dB));

fprintf('10-deg command rise time              : %.3f to %.3f s\n', ...
    min(commandResults.riseTime_s),max(commandResults.riseTime_s));
fprintf('10-deg command overshoot              : %.3f to %.3f %%\n', ...
    min(commandResults.overshoot_pct),max(commandResults.overshoot_pct));
fprintf('10-deg command settling time          : %.3f to %.3f s\n', ...
    min(commandResults.settlingTime_s),max(commandResults.settlingTime_s));
fprintf('Peak automatic aileron                : %.4f to %.4f deg\n', ...
    min(commandResults.peakAutomaticAileron_deg), ...
    max(commandResults.peakAutomaticAileron_deg));
fprintf('Peak automatic rudder                 : %.4f to %.4f deg\n', ...
    min(commandResults.peakAutomaticRudder_deg), ...
    max(commandResults.peakAutomaticRudder_deg));
fprintf('Peak sideslip                         : %.4f to %.4f deg\n', ...
    min(commandResults.peakSideslip_deg), ...
    max(commandResults.peakSideslip_deg));
fprintf('Peak yaw rate                         : %.4f to %.4f deg/s\n\n', ...
    min(commandResults.peakYawRate_degps), ...
    max(commandResults.peakYawRate_degps));

fprintf('Accepted exact-delay envelope         : 0 to %.0f ms\n', ...
    1000*aileronActuator.sensor.maximumAcceptedDelay_s);
for iDelay = 1:numel(summaryDelay_s)
    fprintf('  %3.0f ms: min phase %.2f deg, min gain %.2f dB\n', ...
        1000*summaryDelay_s(iDelay), ...
        summaryMinimumPhaseMargin_deg(iDelay), ...
        summaryMinimumGainMargin_dB(iDelay));
end

fprintf('\nStress initial bank                   : %.1f deg\n', ...
    aileronActuator.stress.initialBank_deg);
fprintf('Stress peak automatic aileron         : %.4f deg\n', ...
    stressCase.peakAutomaticAileron_deg);
fprintf('Stress peak actuator rate             : %.4f deg/s\n', ...
    stressCase.peakAileronRate_degps);
fprintf('Stress position/rate limits active    : %d / %d\n', ...
    stressCase.positionLimitActive,stressCase.rateLimitActive);
fprintf('Stress physical hard stop active      : %d\n', ...
    stressCase.physicalLimitActive);
fprintf('Stress recovery / final bank          : %.3f s / %.6f deg\n', ...
    stressCase.recoveryTime_s,stressCase.finalBank_deg);
fprintf('Stress final automatic aileron        : %.6f deg\n', ...
    stressCase.finalAutomaticAileron_deg);
fprintf('Anti-windup peak-integrator ratio     : %.4f\n', ...
    antiWindupIntegratorRatio);
fprintf('Stress-noise automatic-aileron RMS    : %.5f deg\n', ...
    noiseAileronRms_deg);
fprintf('Stress-noise automatic-aileron peak   : %.5f deg\n\n', ...
    noiseAileronPeak_deg);

fprintf('PASS: the first passing Kp is selected reproducibly.\n');
fprintf('PASS: all structured plant/aileron cases retain stable dynamics.\n');
fprintf('PASS: modal, exact-delay disk-margin and command-response gates pass.\n');
fprintf('PASS: protected aileron limits and anti-windup pass the logic stress.\n');
fprintf('PASS: deterministic roll-sensor noise remains below its gates.\n');
fprintf(['NOTE: servo, sensor, automatic-authority and noise values remain ', ...
    'explicit project assumptions, not identified C172S hardware data.\n']);
fprintf(['NOTE: time-domain design captures omit pure transport delay; the ', ...
    'declared delay is audited exactly in the frequency domain.\n\n']);
fprintf('Saved linear ledger                  : %s\n',linearLedgerFile);
fprintf('Saved command ledger                 : %s\n',commandLedgerFile);
fprintf('Saved delay ledger                   : %s\n',delayLedgerFile);

results.controller = controller;
results.aileronActuator = aileronActuator;
results.gainSearch = gainSearchResults;
results.linear = linearResults;
results.sensor = sensorResults;
results.delay = delayResults;
results.command = commandResults;
results.normal = normalCase;
results.stress = stressCase;
results.noise = noiseCase;
assignin('base','rollAttitudeHoldV1Results',results);

%% Local functions

function [plants,ledger] = buildRollUncertaintyFamily(baseline,controller)
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
                        [~,plants{row}] = c172sLateralDirectionalPlant(data);
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
assert(row == n,'Roll uncertainty loop count is incorrect.');
ledger = table(caseID,sideForceScale,rollMomentScale,yawMomentScale, ...
    aileronEffectivenessScale,inertiaScale,IxzNormalized);
end

function [Gp,Gphi] = yawProtectedRollResponse(plant,yawDamper,rudder,w)
% States: beta, p, r, phi, rSensor, xw, deltaR.
A = zeros(7,7);
B = zeros(7,1);
A(1:4,1:4) = plant.A(1:4,1:4);
A(1:4,7) = plant.B(1:4,2);
B(1:4) = plant.B(1:4,1);
Ts = rudder.sensor.filterTimeConstant_s;
Tw = yawDamper.washoutTimeConstant_s;
Kr = yawDamper.rudderGain_s;
tauR = rudder.dynamics.timeConstant_s;
A(5,3) = 1/Ts;
A(5,5) = -1/Ts;
A(6,5) = 1/Tw;
A(6,6) = -1/Tw;
A(7,5) = -Kr/tauR;
A(7,6) = Kr/tauR;
A(7,7) = -1/tauR;
C = zeros(2,7);
C(1,2) = 1;
C(2,4) = 1;
response = freqresp(ss(A,B,C,zeros(2,1)),w);
Gp = reshape(response(1,1,:),[],1);
Gphi = reshape(response(2,1,:),[],1);
end

function A = rollClosedLoopMatrix(plant,yawDamper,rudder, ...
    Kphi,Ki,Kp,TsRoll,tauA)
% States: beta,p,r,phi,psi,rSensor,xw,deltaR,pSensor,phiSensor,xi,deltaA.
A = zeros(12,12);
A(1:5,1:5) = plant.A;
A(1:5,8) = plant.B(:,2);
A(1:5,12) = plant.B(:,1);
TsR = rudder.sensor.filterTimeConstant_s;
Tw = yawDamper.washoutTimeConstant_s;
Kr = yawDamper.rudderGain_s;
tauR = rudder.dynamics.timeConstant_s;
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
A(12,9) = -Kp/tauA;
A(12,10) = -Kphi/tauA;
A(12,11) = Ki/tauA;
A(12,12) = -1/tauA;
end

function modes = classifyRollClosedLoop(A)
lambda = eig(A);
[~,headingIndex] = min(abs(lambda));
modes.headingPole = lambda(headingIndex);
dynamic = lambda;
dynamic(headingIndex) = [];
modes.dynamicStable = all(real(dynamic) < -1.0e-10);
modes.minimumDecayRate_per_s = min(-real(dynamic));
oscillatory = dynamic(imag(dynamic) > 1.0e-8);
assert(~isempty(oscillatory), ...
    'The closed loop unexpectedly has no oscillatory dynamic modes.');
modes.minimumOscillatoryDampingRatio = min( ...
    -real(oscillatory)./abs(oscillatory));
end

function result = rollLoopMargins(Gp,Gphi,w,Kphi,Ki,Kp,Ts,tauA,delay)
sensor = 1./(1 + 1i*w*Ts);
servo = 1./(1 + 1i*w*tauA);
controllerResponse = Kp*Gp + (Kphi + Ki./(1i*w)).*Gphi;
L = servo.*sensor.*controllerResponse.*exp(-1i*w*delay);
alpha = abs(2*(1+L)./(1-L));
[diskMargin,index] = min(alpha);
result.diskMargin = diskMargin;
result.phaseMargin_deg = 2*atand(diskMargin/2);
if diskMargin >= 2
    result.gainMargin_dB = inf;
else
    result.gainMargin_dB = 20*log10((2+diskMargin)/(2-diskMargin));
end
result.criticalFrequency_radps = w(index);
end

function [A,B] = rollCommandSystem(plant,yawDamper,rudder,controller,aileron)
A = zeros(13,13);
A(1:12,1:12) = rollClosedLoopMatrix(plant,yawDamper,rudder, ...
    controller.bankProportionalGain,controller.bankIntegralGain_per_s, ...
    controller.rollRateGain_s,aileron.sensor.filterTimeConstant_s, ...
    aileron.dynamics.timeConstant_s);
A(11,13) = 1;
A(12,13) = controller.bankProportionalGain/ ...
    aileron.dynamics.timeConstant_s;
A(13,13) = -1/controller.commandFilterTimeConstant_s;
B = zeros(13,1);
B(13) = 1/controller.commandFilterTimeConstant_s;
end

function metrics = positiveStepMetrics(t,y,command)
finalValue = y(end);
lower = 0.10*command;
upper = 0.90*command;
iLower = find(y >= lower,1,'first');
iUpper = find(y >= upper,1,'first');
if isempty(iLower) || isempty(iUpper)
    metrics.riseTime_s = inf;
else
    metrics.riseTime_s = t(iUpper)-t(iLower);
end
metrics.overshoot_pct = max(0,100*(max(y)-command)/command);
tolerance = 0.02*abs(command);
outside = find(abs(y-finalValue) > tolerance);
if isempty(outside)
    metrics.settlingTime_s = 0;
elseif outside(end) == numel(t)
    metrics.settlingTime_s = inf;
else
    metrics.settlingTime_s = t(outside(end)+1);
end
end

function output = simulateProtectedRollCase(plant,yawDamper,rudder, ...
    controller,aileron,command_deg,initialBank_deg,antiWindup,noiseEnabled)
if noiseEnabled
    dt = aileron.noise.timeStep_s;
    stopTime = aileron.noise.stopTime_s;
else
    dt = aileron.stress.timeStep_s;
    stopTime = aileron.stress.stopTime_s;
end
t = (0:dt:stopTime).';
z = zeros(numel(t),13);
z(1,4) = deg2rad(initialBank_deg);
diagnostics = zeros(numel(t),8);
for k = 1:numel(t)-1
    [k1,d1] = nonlinearDerivative(t(k),z(k,:).',plant,yawDamper, ...
        rudder,controller,aileron,command_deg,antiWindup,noiseEnabled);
    k2 = nonlinearDerivative(t(k)+dt/2,z(k,:).'+dt*k1/2, ...
        plant,yawDamper,rudder,controller,aileron,command_deg, ...
        antiWindup,noiseEnabled);
    k3 = nonlinearDerivative(t(k)+dt/2,z(k,:).'+dt*k2/2, ...
        plant,yawDamper,rudder,controller,aileron,command_deg, ...
        antiWindup,noiseEnabled);
    k4 = nonlinearDerivative(t(k)+dt,z(k,:).'+dt*k3, ...
        plant,yawDamper,rudder,controller,aileron,command_deg, ...
        antiWindup,noiseEnabled);
    z(k+1,:) = (z(k,:).' + dt*(k1+2*k2+2*k3+k4)/6).';
    diagnostics(k,:) = d1;
end
[~,diagnostics(end,:)] = nonlinearDerivative(t(end),z(end,:).', ...
    plant,yawDamper,rudder,controller,aileron,command_deg, ...
    antiWindup,noiseEnabled);

output.t = t;
output.state = z;
output.bank_deg = rad2deg(z(:,4));
output.commandFiltered_deg = rad2deg(z(:,13));
output.automaticAileron_deg = rad2deg(z(:,12));
output.aileronRate_degps = rad2deg(diagnostics(:,3));
output.totalAileron_deg = rad2deg(diagnostics(:,4));
output.automaticRudder_deg = rad2deg(z(:,8));
output.rawAileron_deg = rad2deg(diagnostics(:,1));
output.positionLimitedCommand_deg = rad2deg(diagnostics(:,2));
output.positionLimitActive = any(abs(diagnostics(:,1)-diagnostics(:,2)) > 1e-10);
output.rateLimitActive = any(abs(diagnostics(:,5)-diagnostics(:,3)) > 1e-10);
output.physicalLimitActive = any(abs(diagnostics(:,6)-diagnostics(:,4)) > 1e-10);
output.peakAutomaticAileron_deg = max(abs(output.automaticAileron_deg));
output.peakAileronRate_degps = max(abs(output.aileronRate_degps));
output.peakTotalAileron_deg = max(abs(output.totalAileron_deg));
output.finalBank_deg = output.bank_deg(end);
output.finalAutomaticAileron_deg = output.automaticAileron_deg(end);
output.peakIntegratorMagnitude = max(abs(z(:,11)));
inside = abs(output.bank_deg) <= aileron.stress.recoveryThreshold_deg;
recoveryIndex = findPermanentEntry(inside);
if isempty(recoveryIndex)
    output.recoveryTime_s = inf;
else
    output.recoveryTime_s = t(recoveryIndex);
end
end

function [dz,diagnostic] = nonlinearDerivative(t,z,plant,yawDamper, ...
    rudder,controller,aileron,command_deg,antiWindup,noiseEnabled)
% State order matches rollCommandSystem.
if noiseEnabled
    multiplier = aileron.sensor.noiseStressMultiplier;
    pNoise = deg2rad(sqrt(2)*multiplier* ...
        aileron.sensor.rollRateNoiseRms_degps)*sin( ...
        aileron.noise.rollRateFrequency_radps*t);
    phiNoise = deg2rad(sqrt(2)*multiplier* ...
        aileron.sensor.bankAngleNoiseRms_deg)*sin( ...
        aileron.noise.bankAngleFrequency_radps*t);
else
    pNoise = 0;
    phiNoise = 0;
end

command = min(max(deg2rad(command_deg), ...
    controller.commandAuthority.lowerLimit_rad), ...
    controller.commandAuthority.upperLimit_rad);
bankError = z(13)-z(10);
rawAileron = controller.bankProportionalGain*bankError + ...
    controller.bankIntegralGain_per_s*z(11) - ...
    controller.rollRateGain_s*z(9);
positionCommand = min(max(rawAileron,aileron.authority.lowerLimit_rad), ...
    aileron.authority.upperLimit_rad);
rawAileronRate = (positionCommand-z(12))/ ...
    aileron.dynamics.timeConstant_s;
rateLimit = deg2rad(aileron.dynamics.rateLimit_degps);
aileronRate = min(max(rawAileronRate,-rateLimit),rateLimit);
trackingAileron = z(12) + aileron.dynamics.timeConstant_s*aileronRate;

rawRudder = -yawDamper.rudderGain_s*(z(6)-z(7));
rudderPositionCommand = min(max(rawRudder,rudder.authority.lowerLimit_rad), ...
    rudder.authority.upperLimit_rad);
rawRudderRate = (rudderPositionCommand-z(8))/ ...
    rudder.dynamics.timeConstant_s;
rudderRateLimit = deg2rad(rudder.dynamics.rateLimit_degps);
rudderRate = min(max(rawRudderRate,-rudderRateLimit),rudderRateLimit);

pilotAileron = 0;
pilotRudder = 0;
unlimitedTotalAileron = pilotAileron + z(12);
totalAileron = min(max(unlimitedTotalAileron, ...
    aileron.physicalTravel.lowerLimit_rad), ...
    aileron.physicalTravel.upperLimit_rad);
totalRudder = min(max(pilotRudder+z(8), ...
    rudder.physicalTravel.lowerLimit_rad), ...
    rudder.physicalTravel.upperLimit_rad);

dz = zeros(13,1);
dz(1:5) = plant.A*z(1:5) + plant.B*[totalAileron;totalRudder];
dz(6) = (z(3)-z(6))/rudder.sensor.filterTimeConstant_s;
dz(7) = (z(6)-z(7))/yawDamper.washoutTimeConstant_s;
dz(8) = rudderRate;
dz(9) = (z(2)+pNoise-z(9))/aileron.sensor.filterTimeConstant_s;
dz(10) = (z(4)+phiNoise-z(10))/aileron.sensor.filterTimeConstant_s;
dz(11) = bankError;
if antiWindup
    dz(11) = dz(11) + (trackingAileron-rawAileron)/( ...
        controller.bankIntegralGain_per_s* ...
        aileron.antiWindup.trackingTimeConstant_s);
end
dz(12) = aileronRate;
dz(13) = (command-z(13))/controller.commandFilterTimeConstant_s;

diagnostic = [rawAileron,positionCommand,aileronRate,totalAileron, ...
    rawAileronRate,unlimitedTotalAileron,rawRudder,rudderRate];
end

function index = findPermanentEntry(condition)
index = [];
lastFalse = find(~condition,1,'last');
if isempty(lastFalse)
    index = 1;
elseif lastFalse < numel(condition)
    index = lastFalse + 1;
end
end

function plotEnvelope(t,y)
lower = min(y,[],2);
upper = max(y,[],2);
fill([t;flipud(t)],[lower;flipud(upper)], ...
    [0.80,0.88,1.00],'EdgeColor','none','FaceAlpha',0.55);
end
