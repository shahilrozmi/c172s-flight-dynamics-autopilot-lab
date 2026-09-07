%% C172S Pitch-Attitude Hold V2 - actuator and anti-windup design
%
% V1 remains the frozen ideal-actuator baseline. V2 adds:
%   * asymmetric aircraft hard-stop metadata;
%   * conservative automatic-control authority;
%   * first-order, rate-limited elevator dynamics;
%   * rate-aware back-calculation anti-windup; and
%   * the minimum q-damper compensation required by the actuator lag.

clc;
clear;
close all;

baselineData = c172sLongitudinalData();
attitude = c172sPitchAttitudeControllerData();
damperV1 = c172sPitchRateDamperData();
actuator = c172sElevatorActuatorData();

assert(exist('ss','file') == 2 && exist('tf','file') == 2, ...
    'Control System Toolbox is required for the actuator design.');

Kp = attitude.Kp_radPerRad;
Ki = attitude.Ki_radPerRadPerS;
tauCommand = attitude.commandFilterTimeConstant_s;

cgValues_in = [baselineData.cg.forwardLimit_in, ...
    baselineData.cg.nominal_in,baselineData.cg.aftLimit_in];
iyValues_kgm2 = unique([baselineData.uncertainty.Iy_range_kgm2(1), ...
    baselineData.inertia.Iy_kgm2,2300.0, ...
    baselineData.uncertainty.Iy_range_kgm2(2)],'stable');
clDeltaEValues = [baselineData.uncertainty.CL_deltaE_range(1), ...
    baselineData.aero.CL_deltaE, ...
    baselineData.uncertainty.CL_deltaE_range(2)];
tauCases_s = actuator.dynamics.timeConstantCases_s;

%% Robust Kq compensation search with actuator dynamics included

KqGrid_s = (actuator.integration.KqSearchRange_s(1): ...
    actuator.integration.KqSearchIncrement_s: ...
    actuator.integration.KqSearchRange_s(2)).';
minimumFastModeZeta = nan(size(KqGrid_s));
allCasesStableAtGain = false(size(KqGrid_s));

for iGain = 1:numel(KqGrid_s)
    caseZeta = [];
    stableAtGain = true;
    for iCG = 1:numel(cgValues_in)
        for iIy = 1:numel(iyValues_kgm2)
            for iCLde = 1:numel(clDeltaEValues)
                data = c172sLongitudinalData(cgValues_in(iCG));
                data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
                data.aero.CL_deltaE = clDeltaEValues(iCLde);
                [~,plant] = c172sLongitudinalPlant(data);
                for iTau = 1:numel(tauCases_s)
                    [Aclosed,~] = actuatorClosedLoopMatrices(plant,Kp,Ki, ...
                        KqGrid_s(iGain),tauCommand,tauCases_s(iTau));
                    poles = eig(Aclosed);
                    stableAtGain = stableAtGain && all(real(poles) < 0);
                    positivePoles = poles(imag(poles) > 1.0e-8);
                    if isempty(positivePoles)
                        stableAtGain = false;
                    else
                        fastPole = selectFastPole(positivePoles);
                        caseZeta(end+1,1) = ...
                            -real(fastPole)/abs(fastPole); %#ok<SAGROW>
                    end
                end
            end
        end
    end
    allCasesStableAtGain(iGain) = stableAtGain;
    if stableAtGain
        minimumFastModeZeta(iGain) = min(caseZeta);
    end
end

qualifyingGain = allCasesStableAtGain & minimumFastModeZeta >= ...
    actuator.integration.minimumFastModeZeta;
firstQualifyingIndex = find(qualifyingGain,1,'first');
assert(~isempty(firstQualifyingIndex), ...
    'No Kq value satisfies the actuator-integrated damping target.');

minimumQualifyingKq_s = KqGrid_s(firstQualifyingIndex);
Kq = actuator.integration.Kq_s;
assert(abs(Kq - minimumQualifyingKq_s) < 1.0e-12, ...
    'Stored actuator-integrated Kq is not the minimum qualifying grid value.');

v1Index = find(abs(KqGrid_s - damperV1.Kq_s) < 1.0e-12,1,'first');
assert(~isempty(v1Index),'The Kq search grid does not contain the V1 gain.');
assert(minimumFastModeZeta(v1Index) < ...
    actuator.integration.minimumFastModeZeta, ...
    'The stored V1 Kq unexpectedly satisfies the actuator damping target.');

%% Detailed 108-case linear validation

nPlantCase = numel(cgValues_in)*numel(iyValues_kgm2)* ...
    numel(clDeltaEValues);
nCase = nPlantCase*numel(tauCases_s);

caseID = (1:nCase).';
cgIn = zeros(nCase,1);
iyKgm2 = zeros(nCase,1);
clDeltaE = zeros(nCase,1);
actuatorTau_s = zeros(nCase,1);
isStable = false(nCase,1);
fastModeWn = zeros(nCase,1);
fastModeZeta = zeros(nCase,1);
phaseMargin_deg = zeros(nCase,1);
gainMargin_dB = zeros(nCase,1);
gainCrossover_radps = zeros(nCase,1);
innerNonPhugoidFrequency = zeros(nCase,1);
bandwidthSeparationRatio = zeros(nCase,1);
overshoot_pct = zeros(nCase,1);
riseTime_s = zeros(nCase,1);
settlingTime_s = zeros(nCase,1);
peakPitchRate_degps = zeros(nCase,1);
peakActuator_deg = zeros(nCase,1);
peakActuatorRate_degps = zeros(nCase,1);
peakUnsaturatedDemand_deg = zeros(nCase,1);
peakSpeedPerturbationFraction = zeros(nCase,1);
peakAlphaPerturbation_deg = zeros(nCase,1);
finalAttitudeError_deg = zeros(nCase,1);
maximumAttitudeDifferenceFromIdeal_deg = zeros(nCase,1);

t = (0:actuator.validation.timeStep_s: ...
    actuator.validation.stopTime_s).';
thetaCommand = deg2rad(actuator.validation.commandStep_deg);
commandInput = thetaCommand*ones(size(t));

thetaHistory_deg = zeros(numel(t),nCase);
actuatorHistory_deg = zeros(numel(t),nCase);
rateHistory_degps = zeros(numel(t),nCase);

row = 0;
for iCG = 1:numel(cgValues_in)
    for iIy = 1:numel(iyValues_kgm2)
        for iCLde = 1:numel(clDeltaEValues)
            data = c172sLongitudinalData(cgValues_in(iCG));
            data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
            data.aero.CL_deltaE = clDeltaEValues(iCLde);
            [~,plant] = c172sLongitudinalPlant(data);

            [Aideal,Bideal] = idealClosedLoopMatrices( ...
                plant,Kp,Ki,damperV1.Kq_s,tauCommand);
            idealSystem = ss(Aideal,Bideal,eye(6),zeros(6,1));
            zIdeal = lsim(idealSystem,commandInput,t);

            for iTau = 1:numel(tauCases_s)
                row = row + 1;
                tauActuator = tauCases_s(iTau);
                [Aclosed,Bclosed] = actuatorClosedLoopMatrices( ...
                    plant,Kp,Ki,Kq,tauCommand,tauActuator);
                commandSystem = ss(Aclosed,Bclosed,eye(7),zeros(7,1));
                z = lsim(commandSystem,commandInput,t);
                poles = eig(Aclosed);
                positivePoles = poles(imag(poles) > 1.0e-8);
                fastPole = selectFastPole(positivePoles);

                [Aouter,Bouter,Couter] = actuatorOuterPlantMatrices( ...
                    plant,Kq,tauActuator);
                Ptheta = ss(Aouter,Bouter,Couter,0);
                Cpi = tf([Kp,Ki],[1,0]);
                [gainMargin,phaseMargin,~,gainCrossover] = ...
                    margin(Ptheta*Cpi);
                outerPlantPoles = eig(Aouter);
                innerFrequency = ...
                    slowestNonPhugoidFrequency(outerPlantPoles);

                theta = z(:,4);
                eTheta = z(:,6) - theta;
                deltaEUnsat = -Kp*eTheta - Ki*z(:,5) + Kq*z(:,3);
                deltaE = z(:,7);
                deltaEDot = (deltaEUnsat - deltaE)/tauActuator;
                [caseRise,caseSettling,caseOvershoot] = ...
                    stepMetrics(t,theta,thetaCommand);

                cgIn(row) = data.cg.selected_in;
                iyKgm2(row) = data.inertia.Iy_kgm2;
                clDeltaE(row) = data.aero.CL_deltaE;
                actuatorTau_s(row) = tauActuator;
                isStable(row) = all(real(poles) < 0);
                fastModeWn(row) = abs(fastPole);
                fastModeZeta(row) = -real(fastPole)/abs(fastPole);
                phaseMargin_deg(row) = phaseMargin;
                gainMargin_dB(row) = 20*log10(gainMargin);
                gainCrossover_radps(row) = gainCrossover;
                innerNonPhugoidFrequency(row) = innerFrequency;
                bandwidthSeparationRatio(row) = ...
                    innerNonPhugoidFrequency(row)/gainCrossover;
                overshoot_pct(row) = caseOvershoot;
                riseTime_s(row) = caseRise;
                settlingTime_s(row) = caseSettling;
                peakPitchRate_degps(row) = max(abs(rad2deg(z(:,3))));
                peakActuator_deg(row) = max(abs(rad2deg(deltaE)));
                peakActuatorRate_degps(row) = ...
                    max(abs(rad2deg(deltaEDot)));
                peakUnsaturatedDemand_deg(row) = ...
                    max(abs(rad2deg(deltaEUnsat)));
                peakSpeedPerturbationFraction(row) = ...
                    max(abs(z(:,1)))/data.trim.V_mps;
                peakAlphaPerturbation_deg(row) = ...
                    max(abs(rad2deg(z(:,2))));
                finalAttitudeError_deg(row) = ...
                    abs(rad2deg(thetaCommand - theta(end)));
                maximumAttitudeDifferenceFromIdeal_deg(row) = ...
                    max(abs(rad2deg(theta - zIdeal(:,4))));

                thetaHistory_deg(:,row) = rad2deg(theta);
                actuatorHistory_deg(:,row) = rad2deg(deltaE);
                rateHistory_degps(:,row) = rad2deg(deltaEDot);
            end
        end
    end
end

results = table(caseID,cgIn,iyKgm2,clDeltaE,actuatorTau_s,isStable, ...
    fastModeWn,fastModeZeta,phaseMargin_deg,gainMargin_dB, ...
    gainCrossover_radps,innerNonPhugoidFrequency, ...
    bandwidthSeparationRatio,overshoot_pct, ...
    riseTime_s,settlingTime_s,peakPitchRate_degps,peakActuator_deg, ...
    peakActuatorRate_degps,peakUnsaturatedDemand_deg, ...
    peakSpeedPerturbationFraction,peakAlphaPerturbation_deg, ...
    finalAttitudeError_deg,maximumAttitudeDifferenceFromIdeal_deg);

%% Linear requirements and inactivity of nonlinear protection

req = actuator.requirements;
assert(all(results.isStable),'At least one actuator-integrated case is unstable.');
assert(all(results.fastModeZeta >= req.minimumFastModeZeta), ...
    'At least one case failed the fast-mode damping gate.');
assert(all(results.phaseMargin_deg >= req.minimumPhaseMargin_deg), ...
    'At least one case failed the phase-margin gate.');
assert(all(results.gainCrossover_radps <= req.maximumGainCrossover_radps), ...
    'At least one case exceeded the crossover gate.');
assert(all(results.bandwidthSeparationRatio >= ...
    req.minimumBandwidthSeparationRatio), ...
    'At least one case failed bandwidth separation.');
assert(all(results.overshoot_pct <= req.maximumStepOvershoot_pct), ...
    'At least one case exceeded the overshoot gate.');
assert(all(results.riseTime_s >= req.minimumRiseTime_s & ...
    results.riseTime_s <= req.maximumRiseTime_s), ...
    'At least one case failed the rise-time gate.');
assert(all(results.settlingTime_s <= req.maximumSettlingTime_s), ...
    'At least one case failed the settling-time gate.');
assert(all(results.peakPitchRate_degps <= req.maximumPitchRate_degps), ...
    'At least one case exceeded the pitch-rate gate.');
assert(all(results.peakActuator_deg <= ...
    req.maximumAutomaticElevator_deg), ...
    'At least one case exceeded the automatic-elevator gate.');
assert(all(results.peakActuatorRate_degps <= ...
    req.maximumNominalActuatorRate_degps), ...
    'At least one normal command used excessive actuator rate.');
assert(all(results.peakUnsaturatedDemand_deg < ...
    min(actuator.authority.uncertainty_deg)), ...
    'The normal command unexpectedly activated position limiting.');
assert(all(results.peakActuatorRate_degps < ...
    min(actuator.dynamics.rateLimitCases_degps)), ...
    'The normal command unexpectedly activated rate limiting.');
assert(all(results.peakSpeedPerturbationFraction <= ...
    req.maximumSpeedPerturbationFraction), ...
    'At least one case left the speed-perturbation envelope.');
assert(all(results.peakAlphaPerturbation_deg <= ...
    req.maximumAlphaPerturbation_deg), ...
    'At least one case left the alpha envelope.');
assert(all(results.finalAttitudeError_deg <= ...
    req.maximumFinalAttitudeError_deg), ...
    'At least one case failed final tracking.');
assert(all(results.maximumAttitudeDifferenceFromIdeal_deg <= ...
    req.maximumAttitudeDifferenceFromIdeal_deg), ...
    'Actuator lag changed the V1 response more than allowed.');

%% Nonlinear anti-windup logic stress test

[stressAw,stressMetricsAw] = simulateProtectionStress( ...
    baselineData,attitude,actuator,Kq,true);
[stressNoAw,stressMetricsNoAw] = simulateProtectionStress( ...
    baselineData,attitude,actuator,Kq,false);

assert(stressMetricsAw.positionLimitActive, ...
    'The anti-windup stress case did not activate position limiting.');
assert(stressMetricsAw.rateLimitActive, ...
    'The anti-windup stress case did not activate rate limiting.');
assert(stressMetricsAw.peakIntegrator_deg_s <= ...
    actuator.stress.maximumPeakIntegratorRatio* ...
    stressMetricsNoAw.peakIntegrator_deg_s, ...
    'Anti-windup did not sufficiently limit integrator growth.');
assert(stressMetricsAw.positionSaturationDuration_s <= ...
    actuator.stress.maximumSaturationDurationRatio* ...
    stressMetricsNoAw.positionSaturationDuration_s, ...
    'Anti-windup did not sufficiently shorten position saturation.');
assert(abs(stressMetricsAw.finalAttitude_deg) <= ...
    actuator.stress.maximumFinalAttitude_deg, ...
    'The protected stress response did not recover near zero attitude.');

%% Console report and ledger

fprintf('============================================================\n');
fprintf(' C172S PITCH-ATTITUDE HOLD V2 - ACTUATOR PROTECTION DESIGN\n');
fprintf('============================================================\n\n');
fprintf('Frozen attitude gains                : Kp = %.4f, Ki = %.4f 1/s\n',Kp,Ki);
fprintf('Ideal-actuator q gain                : %.4f s\n',damperV1.Kq_s);
fprintf('Actuator-integrated q gain           : %.4f s\n',Kq);
fprintf('Worst zeta with original q gain      : %.4f\n', ...
    minimumFastModeZeta(v1Index));
fprintf('Minimum qualifying compensated gain  : %.4f s\n\n', ...
    minimumQualifyingKq_s);
fprintf('Certified elevator travel            : %.1f deg up / %.1f deg down\n', ...
    actuator.physicalTravel.trailingEdgeUp_deg, ...
    actuator.physicalTravel.trailingEdgeDown_deg);
fprintf('Automatic-control authority          : +/- %.1f deg (assumption)\n', ...
    actuator.authority.trailingEdgeUp_deg);
fprintf('Actuator time constant               : %.3f s [%.3f, %.3f]\n', ...
    actuator.dynamics.timeConstant_s, ...
    actuator.dynamics.timeConstantRange_s(1), ...
    actuator.dynamics.timeConstantRange_s(2));
fprintf('Actuator rate limit                  : %.1f deg/s [%.1f, %.1f]\n', ...
    actuator.dynamics.rateLimit_degps, ...
    actuator.dynamics.rateLimitRange_degps(1), ...
    actuator.dynamics.rateLimitRange_degps(2));
fprintf('Anti-windup tracking time constant   : %.3f s\n\n', ...
    actuator.antiWindup.trackingTimeConstant_s);

fprintf('Stable linear cases                  : %d/%d\n', ...
    nnz(results.isStable),height(results));
fprintf('Fast-mode damping                    : %.4f to %.4f\n', ...
    min(results.fastModeZeta),max(results.fastModeZeta));
fprintf('Fast-mode natural frequency          : %.4f to %.4f rad/s\n', ...
    min(results.fastModeWn),max(results.fastModeWn));
fprintf('Phase margin                         : %.2f to %.2f deg\n', ...
    min(results.phaseMargin_deg),max(results.phaseMargin_deg));
fprintf('Gain crossover                       : %.4f to %.4f rad/s\n', ...
    min(results.gainCrossover_radps),max(results.gainCrossover_radps));
fprintf('Bandwidth separation                 : %.3f to %.3f\n\n', ...
    min(results.bandwidthSeparationRatio), ...
    max(results.bandwidthSeparationRatio));

fprintf('Command                              : %.2f deg\n', ...
    actuator.validation.commandStep_deg);
fprintf('Overshoot                            : %.3f to %.3f %%\n', ...
    min(results.overshoot_pct),max(results.overshoot_pct));
fprintf('10-90%% rise time                     : %.3f to %.3f s\n', ...
    min(results.riseTime_s),max(results.riseTime_s));
fprintf('2%% settling time                     : %.3f to %.3f s\n', ...
    min(results.settlingTime_s),max(results.settlingTime_s));
fprintf('Peak pitch rate                      : %.3f to %.3f deg/s\n', ...
    min(results.peakPitchRate_degps),max(results.peakPitchRate_degps));
fprintf('Peak actuator position               : %.3f to %.3f deg\n', ...
    min(results.peakActuator_deg),max(results.peakActuator_deg));
fprintf('Peak actuator rate                   : %.3f to %.3f deg/s\n', ...
    min(results.peakActuatorRate_degps), ...
    max(results.peakActuatorRate_degps));
fprintf('Peak |u|/Vtrim                       : %.2f to %.2f %%\n', ...
    100*min(results.peakSpeedPerturbationFraction), ...
    100*max(results.peakSpeedPerturbationFraction));
fprintf('Peak alpha perturbation              : %.3f to %.3f deg\n', ...
    min(results.peakAlphaPerturbation_deg), ...
    max(results.peakAlphaPerturbation_deg));
fprintf('Maximum difference from V1 attitude  : %.5f deg\n\n', ...
    max(results.maximumAttitudeDifferenceFromIdeal_deg));

fprintf('Stress initial attitude              : %.1f deg (logic test only)\n', ...
    actuator.stress.initialAttitude_deg);
fprintf('Peak integrator, anti-windup         : %.3f deg s\n', ...
    stressMetricsAw.peakIntegrator_deg_s);
fprintf('Peak integrator, no anti-windup      : %.3f deg s\n', ...
    stressMetricsNoAw.peakIntegrator_deg_s);
fprintf('Position saturation, anti-windup     : %.3f s\n', ...
    stressMetricsAw.positionSaturationDuration_s);
fprintf('Position saturation, no anti-windup  : %.3f s\n', ...
    stressMetricsNoAw.positionSaturationDuration_s);
fprintf('Protected final attitude             : %.5f deg\n\n', ...
    stressMetricsAw.finalAttitude_deg);

fprintf('PASS: all 108 linear plant/actuator cases satisfy the V2 gates.\n');
fprintf('PASS: nonlinear limits remain inactive for the validated 1 deg command.\n');
fprintf('PASS: anti-windup limits integrator growth and saturation duration.\n');
fprintf('PASS: aircraft data and provisional actuator assumptions remain separated.\n\n');

outputFile = fullfile(pwd,'pitch_attitude_actuator_robustness_results.csv');
writetable(results,outputFile);
fprintf('Saved actuator ledger: %s\n',outputFile);

%% Figures

figure('Name','C172S Actuator-Integrated Kq Selection','Color','w');
plot(KqGrid_s,minimumFastModeZeta,'LineWidth',1.5);
hold on;
yline(req.minimumFastModeZeta,'k--','Damping requirement');
xline(damperV1.Kq_s,':','V1 ideal-actuator Kq');
xline(Kq,'--','V2 selected Kq');
grid on;
xlabel('Pitch-rate feedback gain K_q (s)');
ylabel('Worst-case fast-mode damping ratio');
title('Actuator-Integrated Pitch-Rate Gain Selection');

figure('Name','C172S V2 Robust Attitude Responses','Color','w');
plot(t,thetaHistory_deg,'Color',[0.75 0.82 0.90], ...
    'HandleVisibility','off');
hold on;
yline(actuator.validation.commandStep_deg,'k--','Command');
grid on;
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Pitch-Attitude Hold V2 Across 108 Linear Cases');

nominalSelection = abs(results.cgIn - baselineData.cg.nominal_in) < 1.0e-12 ...
    & abs(results.iyKgm2 - baselineData.inertia.Iy_kgm2) < 1.0e-9 ...
    & abs(results.clDeltaE - baselineData.aero.CL_deltaE) < 1.0e-12 ...
    & abs(results.actuatorTau_s - actuator.dynamics.timeConstant_s) < 1.0e-12;
nominalIndex = find(nominalSelection,1,'first');
assert(~isempty(nominalIndex),'Unable to identify the nominal actuator case.');

figure('Name','C172S V2 Nominal Actuator Activity','Color','w');
tiledlayout(3,1);
nexttile;
plot(t,thetaHistory_deg(:,nominalIndex),'LineWidth',1.4);
hold on;
yline(actuator.validation.commandStep_deg,'k--','Command');
grid on;
ylabel('\theta (deg)');
title('Nominal Attitude Response');
nexttile;
plot(t,actuatorHistory_deg(:,nominalIndex),'LineWidth',1.4);
grid on;
ylabel('\delta_e (deg)');
title('Actual Elevator Position');
nexttile;
plot(t,rateHistory_degps(:,nominalIndex),'LineWidth',1.4);
hold on;
yline(actuator.dynamics.rateLimit_degps,'k--','Rate limit');
yline(-actuator.dynamics.rateLimit_degps,'k--');
grid on;
xlabel('Time (s)');
ylabel('d\delta_e/dt (deg/s)');
title('Elevator Rate');

figure('Name','C172S Anti-Windup Stress Comparison','Color','w');
tiledlayout(3,1);
nexttile;
plot(stressAw.t,rad2deg(stressAw.z(:,4)),'LineWidth',1.4, ...
    'DisplayName','Anti-windup');
hold on;
plot(stressNoAw.t,rad2deg(stressNoAw.z(:,4)),'--','LineWidth',1.2, ...
    'DisplayName','No anti-windup');
grid on;
ylabel('\theta (deg)');
title('Out-of-Envelope Controller-Logic Stress Test');
legend('Location','best');
nexttile;
plot(stressAw.t,rad2deg(stressAw.z(:,5)),'LineWidth',1.4, ...
    'DisplayName','Anti-windup');
hold on;
plot(stressNoAw.t,rad2deg(stressNoAw.z(:,5)),'--','LineWidth',1.2, ...
    'DisplayName','No anti-windup');
grid on;
ylabel('\xi (deg s)');
title('Integrator State');
legend('Location','best');
nexttile;
plot(stressAw.t,rad2deg(stressAw.deltaEUnsat),'LineWidth',1.1, ...
    'DisplayName','Unsaturated demand');
hold on;
plot(stressAw.t,rad2deg(stressAw.z(:,7)),'LineWidth',1.4, ...
    'DisplayName','Actual elevator');
yline(actuator.authority.upperLimit_rad*180/pi,'k--','Authority');
yline(actuator.authority.lowerLimit_rad*180/pi,'k--');
grid on;
xlabel('Time (s)');
ylabel('\delta_e (deg)');
title('Protected Elevator Demand');
legend('Location','best');

%% Local helpers

function [Aclosed,Bcommand] = actuatorClosedLoopMatrices( ...
    plant,Kp,Ki,Kq,tauCommand,tauActuator)
% State order: [u, alpha, q, theta, xi, thetaFiltered, deltaE].
Aclosed = zeros(7,7);
Aclosed(1:4,1:4) = plant.A;
Aclosed(1:4,7) = plant.B;
Aclosed(5,4) = -1.0;
Aclosed(5,6) = 1.0;
Aclosed(6,6) = -1.0/tauCommand;
Aclosed(7,3) = Kq/tauActuator;
Aclosed(7,4) = Kp/tauActuator;
Aclosed(7,5) = -Ki/tauActuator;
Aclosed(7,6) = -Kp/tauActuator;
Aclosed(7,7) = -1.0/tauActuator;
Bcommand = zeros(7,1);
Bcommand(6) = 1.0/tauCommand;
end

function [Aideal,Bcommand] = idealClosedLoopMatrices( ...
    plant,Kp,Ki,Kq,tauCommand)
% State order: [u, alpha, q, theta, xi, thetaFiltered].
Ctheta = [0,0,0,1];
Aq = plant.A + plant.B*[0,0,Kq,0];
Aideal = zeros(6,6);
Aideal(1:4,1:4) = Aq + plant.B*Kp*Ctheta;
Aideal(1:4,5) = -plant.B*Ki;
Aideal(1:4,6) = -plant.B*Kp;
Aideal(5,1:4) = -Ctheta;
Aideal(5,6) = 1.0;
Aideal(6,6) = -1.0/tauCommand;
Bcommand = zeros(6,1);
Bcommand(6) = 1.0/tauCommand;
end

function [Aouter,Bouter,Couter] = actuatorOuterPlantMatrices( ...
    plant,Kq,tauActuator)
% Input is PI elevator command; output is -theta for conventional 1+L form.
Aouter = zeros(5,5);
Aouter(1:4,1:4) = plant.A;
Aouter(1:4,5) = plant.B;
Aouter(5,3) = Kq/tauActuator;
Aouter(5,5) = -1.0/tauActuator;
Bouter = zeros(5,1);
Bouter(5) = 1.0/tauActuator;
Couter = [0,0,0,-1,0];
end

function fastPole = selectFastPole(positivePoles)
[~,index] = max(abs(imag(positivePoles)));
fastPole = positivePoles(index);
end

function frequency = slowestNonPhugoidFrequency(poles)
% The actuator-damped short-period dynamics can become three real poles.
% Therefore an oscillatory-pole selector is not valid for inner/outer
% bandwidth separation. Remove the two lowest-frequency poles (the
% phugoid pair), then use the slowest remaining pole as the conservative
% inner-loop characteristic frequency.
poleFrequencies = sort(abs(poles),'ascend');
assert(numel(poleFrequencies) >= 3, ...
    'The actuator outer plant has too few poles for mode separation.');
frequency = poleFrequencies(3);
end

function [riseTime,settlingTime,overshoot] = ...
    stepMetrics(t,response,command)
rise10Index = find(response >= 0.10*command,1,'first');
rise90Index = find(response >= 0.90*command,1,'first');
if isempty(rise10Index) || isempty(rise90Index)
    riseTime = inf;
else
    riseTime = t(rise90Index) - t(rise10Index);
end
outside = find(abs(response - command) > 0.02*abs(command));
if isempty(outside)
    settlingTime = 0.0;
elseif outside(end) == numel(t)
    settlingTime = inf;
else
    settlingTime = t(outside(end) + 1);
end
overshoot = max(0.0,100*(max(response) - command)/abs(command));
end

function [history,metrics] = simulateProtectionStress( ...
    data,attitude,actuator,Kq,antiWindupEnabled)
[~,plant] = c172sLongitudinalPlant(data);
dt = actuator.stress.timeStep_s;
t = (0:dt:actuator.stress.stopTime_s).';
z = zeros(7,1);
z(4) = deg2rad(actuator.stress.initialAttitude_deg);
zHistory = zeros(numel(t),7);
deltaEUnsat = zeros(size(t));
positionLimited = false(size(t));
rateLimited = false(size(t));

for k = 1:numel(t)
    zHistory(k,:) = z.';
    [~,diagnostic] = stressDerivative(z,plant,attitude,actuator, ...
        Kq,antiWindupEnabled);
    deltaEUnsat(k) = diagnostic.deltaEUnsat;
    positionLimited(k) = diagnostic.positionLimited;
    rateLimited(k) = diagnostic.rateLimited;
    if k < numel(t)
        k1 = stressDerivative(z,plant,attitude,actuator,Kq,antiWindupEnabled);
        k2 = stressDerivative(z + 0.5*dt*k1,plant,attitude,actuator, ...
            Kq,antiWindupEnabled);
        k3 = stressDerivative(z + 0.5*dt*k2,plant,attitude,actuator, ...
            Kq,antiWindupEnabled);
        k4 = stressDerivative(z + dt*k3,plant,attitude,actuator, ...
            Kq,antiWindupEnabled);
        z = z + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end

history.t = t;
history.z = zHistory;
history.deltaEUnsat = deltaEUnsat;
history.positionLimited = positionLimited;
history.rateLimited = rateLimited;

metrics.positionLimitActive = any(positionLimited);
metrics.rateLimitActive = any(rateLimited);
metrics.positionSaturationDuration_s = dt*nnz(positionLimited);
metrics.rateSaturationDuration_s = dt*nnz(rateLimited);
metrics.peakIntegrator_deg_s = max(abs(rad2deg(zHistory(:,5))));
metrics.finalAttitude_deg = rad2deg(zHistory(end,4));
recoveryIndex = find(abs(rad2deg(zHistory(:,4))) <= ...
    actuator.stress.recoveryThreshold_deg,1,'first');
if isempty(recoveryIndex)
    metrics.recoveryTime_s = inf;
else
    metrics.recoveryTime_s = t(recoveryIndex);
end
end

function [zDot,diagnostic] = stressDerivative( ...
    z,plant,attitude,actuator,Kq,antiWindupEnabled)
Kp = attitude.Kp_radPerRad;
Ki = attitude.Ki_radPerRadPerS;
tauCommand = attitude.commandFilterTimeConstant_s;
tauActuator = actuator.dynamics.timeConstant_s;
Tt = actuator.antiWindup.trackingTimeConstant_s;
rateLimit = deg2rad(actuator.dynamics.rateLimit_degps);

eTheta = z(6) - z(4);
deltaEUnsat = -Kp*eTheta - Ki*z(5) + Kq*z(3);
deltaEPositionCommand = min(max(deltaEUnsat, ...
    actuator.authority.lowerLimit_rad),actuator.authority.upperLimit_rad);
rawRate = (deltaEPositionCommand - z(7))/tauActuator;
limitedRate = min(max(rawRate,-rateLimit),rateLimit);
deltaETrack = z(7) + tauActuator*limitedRate;
antiWindupCorrection = antiWindupEnabled* ...
    (deltaEUnsat - deltaETrack)/(Ki*Tt);

zDot = zeros(7,1);
zDot(1:4) = plant.A*z(1:4) + plant.B*z(7);
zDot(5) = eTheta + antiWindupCorrection;
zDot(6) = -z(6)/tauCommand;
zDot(7) = limitedRate;

diagnostic.deltaEUnsat = deltaEUnsat;
diagnostic.positionLimited = ...
    abs(deltaEPositionCommand - deltaEUnsat) > 1.0e-12;
diagnostic.rateLimited = abs(limitedRate - rawRate) > 1.0e-12;
end
