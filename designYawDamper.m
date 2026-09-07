%% C172S Yaw Damper V1 - robust small-disturbance design
%
% Architecture:
%
%   xwDot        = (r - xw)/Tw
%   rWashout     = r - xw
%   deltaRDamper = -Kr*rWashout
%
% The washout zero at the origin prevents the damper from opposing a
% sustained yaw rate indefinitely and limits interference with the slow
% spiral/turning behavior. V1 intentionally retains an ideal sensor and
% ideal rudder path; hardware protection is a separate later stage.

clc;
clear;
close all;

baselineData = c172sLateralDirectionalData();
yawDamper = c172sYawDamperData();
[~,nominalPlant] = c172sLateralDirectionalPlant(baselineData);

assert(exist('ss','file') == 2 && exist('tf','file') == 2, ...
    'Control System Toolbox is required for the yaw-damper design.');
assert(strcmp(baselineData.model.version, ...
    yawDamper.requiredPlantVersion), ...
    ['Plant dependency mismatch: Yaw Damper V1 requires ', ...
    yawDamper.requiredPlantVersion,'.']);
assert(nominalPlant.B(3,2) > 0, ...
    ['Plant sign mismatch: positive nose-right rudder must initially ', ...
    'produce positive nose-right yaw acceleration.']);
assert(yawDamper.feedbackSign == -1.0, ...
    ['Controller sign mismatch: positive yaw rate must command exactly ', ...
    'negative unit-sign rudder feedback before Kr is applied.']);

Tw = yawDamper.washoutTimeConstant_s;
[plants,caseLedger] = buildUncertaintyFamily( ...
    baselineData,yawDamper);
nCase = numel(plants);
assert(nCase == yawDamper.uncertainty.caseCount, ...
    'The yaw-damper uncertainty ledger is incomplete.');

%% Select the minimum qualifying yaw-rate gain

gainGrid_s = (yawDamper.design.gainSearchRange_s(1): ...
    yawDamper.design.gainSearchIncrement_s: ...
    yawDamper.design.gainSearchRange_s(2)).';
nGain = numel(gainGrid_s);

allDynamicModesStable = false(nGain,1);
minimumDutchDampingRatio = nan(nGain,1);
minimumRollPoleMagnitudeRatio = nan(nGain,1);
maximumRollPoleMagnitudeRatio = nan(nGain,1);
minimumSpiralDecayRateRatio = nan(nGain,1);

openModes = cell(nCase,1);
for iCase = 1:nCase
    openModes{iCase} = classifyPlantModes(plants{iCase}.A);
end

for iGain = 1:nGain
    KrCandidate = gainGrid_s(iGain);
    stableAtGain = true;
    dutchZeta = zeros(nCase,1);
    rollRatio = zeros(nCase,1);
    spiralRatio = zeros(nCase,1);

    for iCase = 1:nCase
        Aclosed = yawDamperClosedLoopMatrix( ...
            plants{iCase},KrCandidate,Tw);
        modes = classifyYawDamperModes(Aclosed);
        stableAtGain = stableAtGain && modes.dynamicStable;
        dutchZeta(iCase) = modes.dutchDampingRatio;
        rollRatio(iCase) = abs(modes.rollPole)/ ...
            abs(openModes{iCase}.rollPole);
        spiralRatio(iCase) = abs(modes.spiralPole)/ ...
            abs(openModes{iCase}.spiralPole);
    end

    allDynamicModesStable(iGain) = stableAtGain;
    if stableAtGain
        minimumDutchDampingRatio(iGain) = min(dutchZeta);
        minimumRollPoleMagnitudeRatio(iGain) = min(rollRatio);
        maximumRollPoleMagnitudeRatio(iGain) = max(rollRatio);
        minimumSpiralDecayRateRatio(iGain) = min(spiralRatio);
    end
end

qualifyingGain = allDynamicModesStable & ...
    minimumDutchDampingRatio >= ...
        yawDamper.design.minimumDutchDampingRatio & ...
    minimumRollPoleMagnitudeRatio >= ...
        yawDamper.design.minimumRollPoleMagnitudeRatio & ...
    maximumRollPoleMagnitudeRatio <= ...
        yawDamper.design.maximumRollPoleMagnitudeRatio & ...
    minimumSpiralDecayRateRatio >= ...
        yawDamper.design.minimumSpiralDecayRateRatio;

selectedIndex = find(qualifyingGain,1,'first');
assert(~isempty(selectedIndex), ...
    'No gain satisfies the declared robust yaw-damper design gates.');
minimumQualifyingGain_s = gainGrid_s(selectedIndex);
Kr = yawDamper.rudderGain_s;
assert(abs(Kr - minimumQualifyingGain_s) < 1.0e-12, ...
    'Stored Kr is not the minimum qualifying gain on the search grid.');

%% Detailed 729-case validation at the frozen gain

isDynamicStable = false(nCase,1);
neutralHeadingPoleMagnitude = zeros(nCase,1);
dutchNaturalFrequency_radps = zeros(nCase,1);
dutchDampingRatio = zeros(nCase,1);
dutchDecayRate_radps = zeros(nCase,1);
balancedDiskMargin = zeros(nCase,1);
balancedDiskPhaseMargin_deg = zeros(nCase,1);
balancedDiskGainMargin_dB = zeros(nCase,1);
diskMarginCriticalFrequency_radps = zeros(nCase,1);
unityGainCrossoverCount = zeros(nCase,1);
rollPole_per_s = zeros(nCase,1);
rollPoleMagnitudeRatio = zeros(nCase,1);
spiralPole_per_s = zeros(nCase,1);
spiralDecayRateRatio = zeros(nCase,1);
washoutPole_per_s = zeros(nCase,1);
initialYawRateIntegralRatio = zeros(nCase,1);
initialBetaIntegralRatio = zeros(nCase,1);
peakInitialDisturbanceRudder_deg = zeros(nCase,1);

t = (0:yawDamper.validation.timeStep_s: ...
    yawDamper.validation.stopTime_s).';
initialYawRate_radps = deg2rad( ...
    yawDamper.validation.initialYawRate_degps);

for iCase = 1:nCase
    plant = plants{iCase};
    Aclosed = yawDamperClosedLoopMatrix(plant,Kr,Tw);
    modes = classifyYawDamperModes(Aclosed);

    rudderToYawRate = ss(plant.A(1:4,1:4),plant.B(1:4,2), ...
        [0 0 1 0],0);
    washout = tf([Tw 0],[Tw 1]);
    loopTransfer = Kr*washout*rudderToYawRate;
    diskMargins = balancedDiskMargins(loopTransfer,yawDamper);

    disturbance = initialYawDisturbanceMetrics( ...
        plant,Kr,Tw,t,initialYawRate_radps);

    isDynamicStable(iCase) = modes.dynamicStable;
    neutralHeadingPoleMagnitude(iCase) = abs(modes.headingPole);
    dutchNaturalFrequency_radps(iCase) = modes.dutchNaturalFrequency;
    dutchDampingRatio(iCase) = modes.dutchDampingRatio;
    dutchDecayRate_radps(iCase) = modes.dutchDecayRate;
    balancedDiskMargin(iCase) = diskMargins.diskMargin;
    balancedDiskPhaseMargin_deg(iCase) = ...
        diskMargins.phaseMargin_deg;
    balancedDiskGainMargin_dB(iCase) = diskMargins.gainMargin_dB;
    diskMarginCriticalFrequency_radps(iCase) = ...
        diskMargins.criticalFrequency_radps;
    unityGainCrossoverCount(iCase) = ...
        diskMargins.unityGainCrossoverCount;
    rollPole_per_s(iCase) = modes.rollPole;
    rollPoleMagnitudeRatio(iCase) = abs(modes.rollPole)/ ...
        abs(openModes{iCase}.rollPole);
    spiralPole_per_s(iCase) = modes.spiralPole;
    spiralDecayRateRatio(iCase) = abs(modes.spiralPole)/ ...
        abs(openModes{iCase}.spiralPole);
    washoutPole_per_s(iCase) = modes.washoutPole;
    initialYawRateIntegralRatio(iCase) = ...
        disturbance.yawRateIntegralRatio;
    initialBetaIntegralRatio(iCase) = ...
        disturbance.betaIntegralRatio;
    peakInitialDisturbanceRudder_deg(iCase) = ...
        disturbance.peakAutomaticRudder_deg;
end

results = [caseLedger,table(isDynamicStable, ...
    neutralHeadingPoleMagnitude,dutchNaturalFrequency_radps, ...
    dutchDampingRatio,dutchDecayRate_radps,balancedDiskMargin, ...
    balancedDiskPhaseMargin_deg,balancedDiskGainMargin_dB, ...
    diskMarginCriticalFrequency_radps,unityGainCrossoverCount, ...
    rollPole_per_s, ...
    rollPoleMagnitudeRatio,spiralPole_per_s,spiralDecayRateRatio, ...
    washoutPole_per_s,initialYawRateIntegralRatio, ...
    initialBetaIntegralRatio,peakInitialDisturbanceRudder_deg)];

req = yawDamper.requirements;
assert(all(results.isDynamicStable), ...
    'At least one uncertainty case has an unstable dynamic mode.');
assert(all(results.neutralHeadingPoleMagnitude < 1.0e-10), ...
    'The expected neutral kinematic heading pole was not preserved.');
assert(all(results.dutchDampingRatio >= ...
    req.minimumDutchDampingRatio), ...
    'At least one case failed the Dutch-roll damping target.');
assert(all(results.dutchNaturalFrequency_radps >= ...
    req.minimumDutchNaturalFrequency_radps), ...
    'At least one case failed the Dutch-roll frequency guidance gate.');
assert(all(results.dutchDecayRate_radps >= ...
    req.minimumDutchDecayRate_radps), ...
    'At least one case failed the Dutch-roll decay-rate guidance gate.');
assert(all(results.balancedDiskPhaseMargin_deg >= ...
    req.minimumBalancedDiskPhaseMargin_deg), ...
    'At least one case failed the balanced disk phase-margin gate.');
assert(all(results.balancedDiskGainMargin_dB >= ...
    req.minimumBalancedDiskGainMargin_dB), ...
    'At least one case failed the balanced disk gain-margin gate.');
assert(all(results.rollPoleMagnitudeRatio >= ...
    req.minimumRollPoleMagnitudeRatio & ...
    results.rollPoleMagnitudeRatio <= ...
    req.maximumRollPoleMagnitudeRatio), ...
    'Yaw feedback changed the roll-subsidence mode excessively.');
assert(all(results.spiralPole_per_s < 0), ...
    'At least one case lost stable spiral behavior.');
assert(all(results.spiralDecayRateRatio >= ...
    req.minimumSpiralDecayRateRatio), ...
    'Yaw feedback degraded spiral decay more than allowed.');
assert(all(results.initialYawRateIntegralRatio <= ...
    req.maximumInitialYawRateIntegralRatio), ...
    'At least one case failed the initial-yaw-rate attenuation gate.');
assert(all(results.initialBetaIntegralRatio <= ...
    req.maximumInitialBetaIntegralRatio), ...
    'At least one case failed the sideslip attenuation gate.');
assert(all(results.peakInitialDisturbanceRudder_deg <= ...
    req.maximumInitialDisturbanceRudder_deg), ...
    'At least one case exceeded the V1 rudder-demand gate.');

%% Washout frequency separation and steady-turn rejection

nominalOpenModes = classifyPlantModes(nominalPlant.A);
nominalAclosed = yawDamperClosedLoopMatrix(nominalPlant,Kr,Tw);
nominalClosedModes = classifyYawDamperModes(nominalAclosed);

% If Robust Control Toolbox is available, cross-check the independent
% balanced-disk calculation against MathWorks DISKMARGIN for the nominal
% loop. The release does not require that additional toolbox.
nominalRudderToYawRate = ss(nominalPlant.A(1:4,1:4), ...
    nominalPlant.B(1:4,2),[0 0 1 0],0);
nominalLoopTransfer = Kr*tf([Tw 0],[Tw 1])* ...
    nominalRudderToYawRate;
nominalIndependentDiskMargins = balancedDiskMargins( ...
    nominalLoopTransfer,yawDamper);
if exist('diskmargin','file') == 2
    nominalToolboxDiskMargins = diskmargin(nominalLoopTransfer,0);
    assert(abs(nominalToolboxDiskMargins.DiskMargin - ...
        nominalIndependentDiskMargins.diskMargin) < 1.0e-3, ...
        ['Independent balanced-disk calculation does not match ', ...
        'MathWorks DISKMARGIN.']);
end

slowFrequency = yawDamper.validation.slowTurnFrequency_radps;
dutchFrequency = nominalOpenModes.dutchNaturalFrequency;
slowWashoutMagnitude = washoutMagnitude(slowFrequency,Tw);
dutchWashoutMagnitude = washoutMagnitude(dutchFrequency,Tw);
fourSecondResidual = exp( ...
    -yawDamper.validation.washoutResidualTime_s/Tw);

assert(slowWashoutMagnitude <= ...
    req.maximumSlowFrequencyWashoutMagnitude, ...
    'The washout passes too much slow-turn content.');
assert(dutchWashoutMagnitude >= ...
    req.minimumDutchBandWashoutMagnitude, ...
    'The washout attenuates too much of the Dutch-roll band.');
assert(fourSecondResidual <= ...
    req.maximumFourSecondWashoutResidual, ...
    'The washout does not release a sustained yaw rate quickly enough.');

%% Nominal independent aileron and rudder pulse checks

aileronCapture = simulatePulse(nominalPlant,Kr,Tw,t,1, ...
    yawDamper.validation.aileronPulseStart_s, ...
    yawDamper.validation.aileronPulseEnd_s, ...
    deg2rad(yawDamper.validation.aileronPulse_deg));
rudderCapture = simulatePulse(nominalPlant,Kr,Tw,t,2, ...
    yawDamper.validation.rudderPulseStart_s, ...
    yawDamper.validation.rudderPulseEnd_s, ...
    deg2rad(yawDamper.validation.rudderPulse_deg));

aileronYawIntegralRatio = trapz(t,abs(aileronCapture.closed(:,3)))/ ...
    trapz(t,abs(aileronCapture.open(:,3)));
aileronBankPeakRatio = max(abs(aileronCapture.closed(:,4)))/ ...
    max(abs(aileronCapture.open(:,4)));
rudderYawIntegralRatio = trapz(t,abs(rudderCapture.closed(:,3)))/ ...
    trapz(t,abs(rudderCapture.open(:,3)));
rudderBetaPeakRatio = max(abs(rudderCapture.closed(:,1)))/ ...
    max(abs(rudderCapture.open(:,1)));
peakPulseAutomaticRudder_deg = max([ ...
    max(abs(rad2deg(aileronCapture.automaticRudder))), ...
    max(abs(rad2deg(rudderCapture.automaticRudder)))]);

assert(aileronYawIntegralRatio <= ...
    req.maximumAileronYawIntegralRatio, ...
    'Yaw Damper V1 did not reduce the aileron-induced yaw response.');
assert(aileronBankPeakRatio >= req.minimumAileronBankPeakRatio & ...
    aileronBankPeakRatio <= req.maximumAileronBankPeakRatio, ...
    'Yaw Damper V1 changed the aileron bank response excessively.');
assert(rudderYawIntegralRatio <= req.maximumRudderYawIntegralRatio, ...
    'Yaw Damper V1 did not sufficiently damp the rudder-pulse response.');
assert(rudderBetaPeakRatio <= req.maximumRudderBetaPeakRatio, ...
    'Yaw Damper V1 did not sufficiently reduce rudder-induced sideslip.');
assert(peakPulseAutomaticRudder_deg <= ...
    req.maximumPulseAutomaticRudder_deg, ...
    'A nominal pulse required excessive automatic rudder demand.');

%% Nominal initial-condition histories for plots and report

nominalDisturbance = initialYawDisturbanceMetrics( ...
    nominalPlant,Kr,Tw,t,initialYawRate_radps,true);

fprintf('============================================================\n');
fprintf('          C172S YAW DAMPER V1 - ROBUST DESIGN\n');
fprintf('============================================================\n\n');
fprintf('Plant baseline                       : Lateral-Directional Plant V0.1\n');
fprintf('Plant status                         : Provisional research baseline\n');
fprintf('Artifact revision                    : %s\n', ...
    yawDamper.artifactRevision);
fprintf('Control law                          : deltaR_YD = -Kr*(r - xw)\n');
fprintf('Yaw-rate gain Kr                     : %.3f s\n',Kr);
fprintf('Washout time constant                : %.2f s\n',Tw);
fprintf('Washout corner frequency             : %.3f rad/s\n',1/Tw);
fprintf('Minimum qualifying gain              : %.3f s\n', ...
    minimumQualifyingGain_s);
fprintf('Structured sensitivity cases         : %d\n\n',nCase);

fprintf('Nominal open Dutch-roll poles        : %.6f +/- %.6fi 1/s\n', ...
    real(nominalOpenModes.dutchPole), ...
    abs(imag(nominalOpenModes.dutchPole)));
fprintf('Nominal closed Dutch-roll poles      : %.6f +/- %.6fi 1/s\n', ...
    real(nominalClosedModes.dutchPole), ...
    abs(imag(nominalClosedModes.dutchPole)));
fprintf('Nominal Dutch damping, open/closed   : %.4f / %.4f\n', ...
    nominalOpenModes.dutchDampingRatio, ...
    nominalClosedModes.dutchDampingRatio);
fprintf('Nominal Dutch frequency, open/closed : %.4f / %.4f rad/s\n\n', ...
    nominalOpenModes.dutchNaturalFrequency, ...
    nominalClosedModes.dutchNaturalFrequency);

fprintf('Robust Dutch damping range           : %.4f to %.4f\n', ...
    min(results.dutchDampingRatio),max(results.dutchDampingRatio));
fprintf('Robust Dutch frequency range         : %.4f to %.4f rad/s\n', ...
    min(results.dutchNaturalFrequency_radps), ...
    max(results.dutchNaturalFrequency_radps));
fprintf('Robust Dutch decay-rate range        : %.4f to %.4f 1/s\n', ...
    min(results.dutchDecayRate_radps), ...
    max(results.dutchDecayRate_radps));
fprintf('Balanced disk phase-margin range     : %.2f to %.2f deg\n', ...
    min(results.balancedDiskPhaseMargin_deg), ...
    max(results.balancedDiskPhaseMargin_deg));
fprintf('Balanced disk minimum gain margin    : %.2f dB\n', ...
    min(results.balancedDiskGainMargin_dB));
fprintf('Unity-gain crossover count range     : %.0f to %.0f\n', ...
    min(results.unityGainCrossoverCount), ...
    max(results.unityGainCrossoverCount));
fprintf('Roll-pole magnitude ratio            : %.4f to %.4f\n', ...
    min(results.rollPoleMagnitudeRatio), ...
    max(results.rollPoleMagnitudeRatio));
fprintf('Spiral decay-rate ratio              : %.4f to %.4f\n\n', ...
    min(results.spiralDecayRateRatio), ...
    max(results.spiralDecayRateRatio));

fprintf('Initial-r integral ratio range       : %.4f to %.4f\n', ...
    min(results.initialYawRateIntegralRatio), ...
    max(results.initialYawRateIntegralRatio));
fprintf('Initial-beta integral ratio range    : %.4f to %.4f\n', ...
    min(results.initialBetaIntegralRatio), ...
    max(results.initialBetaIntegralRatio));
fprintf('Peak initial-disturbance rudder      : %.4f deg\n', ...
    max(results.peakInitialDisturbanceRudder_deg));
fprintf('Slow-frequency washout magnitude     : %.5f at %.3f rad/s\n', ...
    slowWashoutMagnitude,slowFrequency);
fprintf('Dutch-band washout magnitude         : %.5f at %.3f rad/s\n', ...
    dutchWashoutMagnitude,dutchFrequency);
fprintf('Four-second washout residual         : %.4f %%\n\n', ...
    100*fourSecondResidual);

fprintf('Nominal aileron yaw integral ratio   : %.4f\n', ...
    aileronYawIntegralRatio);
fprintf('Nominal aileron bank peak ratio      : %.4f\n', ...
    aileronBankPeakRatio);
fprintf('Nominal rudder yaw integral ratio    : %.4f\n', ...
    rudderYawIntegralRatio);
fprintf('Nominal rudder beta peak ratio       : %.4f\n', ...
    rudderBetaPeakRatio);
fprintf('Peak pulse automatic rudder          : %.4f deg\n\n', ...
    peakPulseAutomaticRudder_deg);

fprintf('PASS: all %d structured sensitivity cases retain stable dynamic modes.\n',nCase);
fprintf('PASS: robust Dutch-roll damping meets the project 0.35 target.\n');
fprintf('PASS: balanced disk margins exceed 60 deg and 12 dB.\n');
fprintf('PASS: roll-subsidence and stable spiral behavior are preserved.\n');
fprintf('PASS: washout retains the Dutch-roll band and rejects steady yaw rate.\n');
fprintf('PASS: initial-rate and independent pulse tests satisfy every V1 gate.\n');
fprintf(['NOTE: MIL-F-8785C values are comparative guidance, not a ', ...
    'C172S certification claim.\n']);
fprintf(['NOTE: actuator, authority, sensor/noise, delay, gust and ', ...
    'full-envelope validation remain open.\n']);

assignin('base','yawDamperDesign',yawDamper);
assignin('base','yawDamperRobustnessResults',results);

%% Figures

figure('Name','C172S Yaw Damper V1 - Modal and Disturbance Response', ...
    'Color','w');
tiledlayout(2,2);

nexttile;
openDynamicPoles = eig(nominalPlant.A);
openDynamicPoles = openDynamicPoles(abs(openDynamicPoles) > 1.0e-10);
closedDynamicPoles = eig(nominalAclosed);
closedDynamicPoles = closedDynamicPoles(abs(closedDynamicPoles) > 1.0e-10);
plot(real(openDynamicPoles),imag(openDynamicPoles),'x', ...
    'LineWidth',1.5,'MarkerSize',8);
hold on;
plot(real(closedDynamicPoles),imag(closedDynamicPoles),'o', ...
    'LineWidth',1.2,'MarkerSize',6);
grid on;
xlabel('Real (1/s)');
ylabel('Imaginary (rad/s)');
title('Nominal Dynamic Poles');
legend('Open loop','Yaw Damper V1','Location','best');

nexttile;
plot(t,rad2deg(nominalDisturbance.open(:,3)),'--','LineWidth',1.1);
hold on;
plot(t,rad2deg(nominalDisturbance.closed(:,3)),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
title('5-deg/s Initial Yaw-Rate Disturbance');
legend('Open loop','Yaw Damper V1','Location','best');

nexttile;
plot(t,rad2deg(nominalDisturbance.open(:,1)),'--','LineWidth',1.1);
hold on;
plot(t,rad2deg(nominalDisturbance.closed(:,1)),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Sideslip (deg)');
title('Initial-Disturbance Sideslip');
legend('Open loop','Yaw Damper V1','Location','best');

nexttile;
plot(t,rad2deg(nominalDisturbance.automaticRudder),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Automatic rudder (deg)');
title('Washout-Filtered Rudder Demand');

sgtitle('C172S Yaw Damper V1');

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

function Aclosed = yawDamperClosedLoopMatrix(plant,Kr,Tw)
% State order: [beta p r phi psi xw].
yawRateOutput = [0 0 1 0 0];
rudderColumn = plant.B(:,2);
Aclosed = zeros(6,6);
Aclosed(1:5,1:5) = plant.A - rudderColumn*Kr*yawRateOutput;
Aclosed(1:5,6) = rudderColumn*Kr;
Aclosed(6,1:5) = yawRateOutput/Tw;
Aclosed(6,6) = -1/Tw;
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

function modes = classifyYawDamperModes(Aclosed)
poles = eig(Aclosed);
[~,headingIndex] = min(abs(poles));
modes.headingPole = poles(headingIndex);
dynamicPoles = poles;
dynamicPoles(headingIndex) = [];
modes.dynamicStable = all(real(dynamicPoles) < -1.0e-10);

positiveComplex = dynamicPoles(imag(dynamicPoles) > 1.0e-8);
assert(numel(positiveComplex) == 1, ...
    'Expected exactly one closed-loop Dutch-roll pole pair.');
modes.dutchPole = positiveComplex(1);
modes.dutchNaturalFrequency = abs(modes.dutchPole);
modes.dutchDampingRatio = ...
    -real(modes.dutchPole)/modes.dutchNaturalFrequency;
modes.dutchDecayRate = -real(modes.dutchPole);

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

function metrics = initialYawDisturbanceMetrics( ...
    plant,Kr,Tw,t,initialYawRate_radps,retainHistories)
if nargin < 6
    retainHistories = false;
end

Aclosed = yawDamperClosedLoopMatrix(plant,Kr,Tw);
dt = t(2) - t(1);
AdOpen = expm(plant.A*dt);
AdClosed = expm(Aclosed*dt);

xOpen = zeros(5,1);
xOpen(3) = initialYawRate_radps;
xClosed = zeros(6,1);
xClosed(3) = initialYawRate_radps;

openHistory = zeros(numel(t),5);
closedHistory = zeros(numel(t),6);
automaticRudder = zeros(numel(t),1);
for k = 1:numel(t)
    openHistory(k,:) = xOpen.';
    closedHistory(k,:) = xClosed.';
    automaticRudder(k) = -Kr*(xClosed(3) - xClosed(6));
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
metrics.peakAutomaticRudder_deg = ...
    max(abs(rad2deg(automaticRudder)));

if retainHistories
    metrics.open = openHistory;
    metrics.closed = closedHistory(:,1:5);
    metrics.washoutState = closedHistory(:,6);
    metrics.automaticRudder = automaticRudder;
end
end

function capture = simulatePulse(plant,Kr,Tw,t,inputIndex,startTime, ...
    endTime,amplitude)
Aclosed = yawDamperClosedLoopMatrix(plant,Kr,Tw);
Bclosed = [plant.B;zeros(1,2)];
dt = t(2) - t(1);
[AdOpen,BdOpen] = exactZoh(plant.A,plant.B,dt);
[AdClosed,BdClosed] = exactZoh(Aclosed,Bclosed,dt);

xOpen = zeros(5,1);
xClosed = zeros(6,1);
openHistory = zeros(numel(t),5);
closedHistory = zeros(numel(t),6);
automaticRudder = zeros(numel(t),1);

for k = 1:numel(t)
    openHistory(k,:) = xOpen.';
    closedHistory(k,:) = xClosed.';
    automaticRudder(k) = -Kr*(xClosed(3) - xClosed(6));
    if k < numel(t)
        input = zeros(2,1);
        if t(k) >= startTime && t(k) < endTime
            input(inputIndex) = amplitude;
        end
        xOpen = AdOpen*xOpen + BdOpen*input;
        xClosed = AdClosed*xClosed + BdClosed*input;
    end
end

capture.open = openHistory;
capture.closed = closedHistory(:,1:5);
capture.washoutState = closedHistory(:,6);
capture.automaticRudder = automaticRudder;
end

function [Ad,Bd] = exactZoh(A,B,dt)
nState = size(A,1);
nInput = size(B,2);
augmented = [A,B;zeros(nInput,nState + nInput)];
transition = expm(augmented*dt);
Ad = transition(1:nState,1:nState);
Bd = transition(1:nState,nState + 1:end);
end

function magnitude = washoutMagnitude(frequency_radps,Tw)
s = 1i*frequency_radps;
magnitude = abs(Tw*s/(Tw*s + 1));
end

function margins = balancedDiskMargins(loopTransfer,yawDamper)
%BALANCEDDISKMARGINS All-frequency SISO disk margin for zero skew.
%
% For F = (1 + alpha*delta/2)/(1 - alpha*delta/2), |delta| < 1,
% the smallest destabilizing balanced disk at each frequency is
%
%   alpha(w) = |2*(1 + L(jw))/(1 - L(jw))|.
%
% Minimizing alpha over frequency gives the balanced disk margin. This is
% appropriate for the present multi-crossover loop, whereas one classical
% phase-margin value does not characterize the complete Nyquist response.

frequencyRange = ...
    yawDamper.validation.diskMarginFrequencyRange_radps;
nGrid = yawDamper.validation.diskMarginGridPointCount;
frequencyGrid = logspace(log10(frequencyRange(1)), ...
    log10(frequencyRange(2)),nGrid);
loopResponse = reshape(freqresp(loopTransfer,frequencyGrid),[],1);
alphaGrid = abs(2*(1 + loopResponse)./(1 - loopResponse));

[~,minimumIndex] = min(alphaGrid);
assert(minimumIndex > 1 && minimumIndex < nGrid, ...
    ['Disk-margin frequency range does not bracket the critical ', ...
    'frequency.']);

lowerLogFrequency = log10(frequencyGrid(minimumIndex - 1));
upperLogFrequency = log10(frequencyGrid(minimumIndex + 1));
objective = @(logFrequency) diskMarginAlpha( ...
    loopTransfer,10.^logFrequency);
options = optimset('TolX',1.0e-10,'Display','off');
[criticalLogFrequency,diskMargin] = fminbnd(objective, ...
    lowerLogFrequency,upperLogFrequency,options);

margins.diskMargin = diskMargin;
margins.criticalFrequency_radps = 10.^criticalLogFrequency;
margins.phaseMargin_deg = 2*atand(diskMargin/2);
if diskMargin < 2
    gainMarginUpper = (2 + diskMargin)/(2 - diskMargin);
    margins.gainMargin_dB = 20*log10(gainMarginUpper);
else
    margins.gainMargin_dB = inf;
end

gainDifference = abs(loopResponse) - 1;
margins.unityGainCrossoverCount = sum( ...
    gainDifference(1:end-1).*gainDifference(2:end) < 0);
end

function alpha = diskMarginAlpha(loopTransfer,frequency_radps)
loopValue = evalfr(loopTransfer,1i*frequency_radps);
alpha = abs(2*(1 + loopValue)/(1 - loopValue));
end
