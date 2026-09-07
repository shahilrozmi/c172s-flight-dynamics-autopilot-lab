%% C172S Altitude Hold V1 - robust outer-loop design
%
% This script augments the validated longitudinal plant with
%
%       hDot = Vtrim*(theta - alpha)
%
% and wraps a bounded altitude/vertical-speed controller around the frozen
% actuator-protected Pitch-Attitude Hold V2. The frozen attitude gains,
% actuator dynamics, authority limits and anti-windup law are not changed.

clc;
clear;
close all;

baselineData = c172sLongitudinalData();
attitude = c172sPitchAttitudeControllerData();
actuator = c172sElevatorActuatorData();
altitude = c172sAltitudeHoldControllerData();

assert(exist('ss','file') == 2 && exist('tf','file') == 2, ...
    'Control System Toolbox is required for the altitude-hold design.');

Kp = attitude.Kp_radPerRad;
Ki = attitude.Ki_radPerRadPerS;
Kq = actuator.integration.Kq_s;
tauTheta = attitude.commandFilterTimeConstant_s;
tauAltitude = altitude.commandFilterTimeConstant_s;
Vtrim = baselineData.trim.V_mps;

cgValues_in = [baselineData.cg.forwardLimit_in, ...
    baselineData.cg.nominal_in,baselineData.cg.aftLimit_in];
iyValues_kgm2 = unique([baselineData.uncertainty.Iy_range_kgm2(1), ...
    baselineData.inertia.Iy_kgm2,2300.0, ...
    baselineData.uncertainty.Iy_range_kgm2(2)],'stable');
clDeltaEValues = [baselineData.uncertainty.CL_deltaE_range(1), ...
    baselineData.aero.CL_deltaE, ...
    baselineData.uncertainty.CL_deltaE_range(2)];
tauCases_s = actuator.dynamics.timeConstantCases_s;

%% Construct the 108-case frozen-inner-loop family

nCase = numel(cgValues_in)*numel(iyValues_kgm2)* ...
    numel(clDeltaEValues)*numel(tauCases_s);
plants = cell(nCase,1);
caseCG_in = zeros(nCase,1);
caseIy_kgm2 = zeros(nCase,1);
caseCLdeltaE = zeros(nCase,1);
caseTau_s = zeros(nCase,1);
attitudeCrossover_radps = zeros(nCase,1);

row = 0;
for iCG = 1:numel(cgValues_in)
    for iIy = 1:numel(iyValues_kgm2)
        for iCLde = 1:numel(clDeltaEValues)
            data = c172sLongitudinalData(cgValues_in(iCG));
            data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
            data.aero.CL_deltaE = clDeltaEValues(iCLde);
            [~,plant] = c172sLongitudinalPlant(data);
            for iTau = 1:numel(tauCases_s)
                row = row + 1;
                plants{row} = plant;
                caseCG_in(row) = data.cg.selected_in;
                caseIy_kgm2(row) = data.inertia.Iy_kgm2;
                caseCLdeltaE(row) = data.aero.CL_deltaE;
                caseTau_s(row) = tauCases_s(iTau);

                [Aouter,Bouter,Couter] = actuatorOuterPlantMatrices( ...
                    plant,Kq,tauCases_s(iTau));
                attitudePlant = ss(Aouter,Bouter,Couter,0);
                attitudePI = tf([Kp,Ki],[1,0]);
                [~,~,~,attitudeCrossover_radps(row)] = ...
                    margin(attitudePlant*attitudePI);
            end
        end
    end
end

assert(row == nCase,'The uncertainty-case ledger is incomplete.');

%% Select vertical-speed damping gain

KvGrid = (altitude.design.verticalSpeedGainSearchRange(1): ...
    altitude.design.verticalSpeedGainIncrement: ...
    altitude.design.verticalSpeedGainSearchRange(2)).';
minimumVerticalSpeedPhaseMargin_deg = nan(size(KvGrid));
verticalSpeedFamilyStable = false(size(KvGrid));

for iGain = 1:numel(KvGrid)
    phaseMargins = zeros(nCase,1);
    stable = true;
    for iCase = 1:nCase
        [Aattitude,BthetaCommand] = actuatorClosedLoopMatrices( ...
            plants{iCase},Kp,Ki,Kq,tauTheta,caseTau_s(iCase));
        CverticalSpeed = verticalSpeedOutput(Vtrim);
        verticalSpeedPlant = ss(Aattitude,BthetaCommand, ...
            CverticalSpeed,0);
        [~,phaseMargins(iCase)] = margin( ...
            KvGrid(iGain)*verticalSpeedPlant);
        AverticalSpeed = Aattitude - ...
            BthetaCommand*(KvGrid(iGain)*CverticalSpeed);
        stable = stable && all(real(eig(AverticalSpeed)) < 0) ...
            && isfinite(phaseMargins(iCase));
    end
    verticalSpeedFamilyStable(iGain) = stable;
    if stable
        minimumVerticalSpeedPhaseMargin_deg(iGain) = min(phaseMargins);
    end
end

qualifyingKv = verticalSpeedFamilyStable & ...
    minimumVerticalSpeedPhaseMargin_deg >= ...
    altitude.design.minimumVerticalSpeedFeedbackPhaseMargin_deg;
selectedKvIndex = find(qualifyingKv,1,'last');
assert(~isempty(selectedKvIndex), ...
    'No vertical-speed feedback gain satisfies its robust margin gate.');
selectedKv = KvGrid(selectedKvIndex);
assert(abs(selectedKv - altitude.verticalSpeedGain_radPerMps) < 1.0e-12, ...
    'Stored vertical-speed gain is not the largest qualifying grid value.');

%% Select altitude-error gain with the vertical-speed damping frozen

KhGrid = (altitude.design.altitudeGainSearchRange(1): ...
    altitude.design.altitudeGainIncrement: ...
    altitude.design.altitudeGainSearchRange(2)).';
minimumAltitudePhaseMargin_deg = nan(size(KhGrid));
minimumBandwidthSeparationRatio = nan(size(KhGrid));
altitudeFamilyStable = false(size(KhGrid));

for iGain = 1:numel(KhGrid)
    phaseMargins = zeros(nCase,1);
    separations = zeros(nCase,1);
    stable = true;
    for iCase = 1:nCase
        [Aattitude,BthetaCommand] = actuatorClosedLoopMatrices( ...
            plants{iCase},Kp,Ki,Kq,tauTheta,caseTau_s(iCase));
        CverticalSpeed = verticalSpeedOutput(Vtrim);
        [Aheight,BverticalSpeedCommand,Cheight] = ...
            altitudeOuterPlantMatrices(Aattitude,BthetaCommand, ...
            CverticalSpeed,selectedKv);
        altitudePlant = ss(Aheight,BverticalSpeedCommand,Cheight,0);
        [~,phaseMargins(iCase),~,altitudeCrossover] = ...
            margin(KhGrid(iGain)*altitudePlant);
        separations(iCase) = ...
            attitudeCrossover_radps(iCase)/altitudeCrossover;
        Aaltitude = Aheight - ...
            BverticalSpeedCommand*(KhGrid(iGain)*Cheight);
        stable = stable && all(real(eig(Aaltitude)) < 0) ...
            && isfinite(phaseMargins(iCase)) ...
            && isfinite(altitudeCrossover);
    end
    altitudeFamilyStable(iGain) = stable;
    if stable
        minimumAltitudePhaseMargin_deg(iGain) = min(phaseMargins);
        minimumBandwidthSeparationRatio(iGain) = min(separations);
    end
end

qualifyingKh = altitudeFamilyStable & ...
    minimumAltitudePhaseMargin_deg >= ...
    altitude.design.minimumAltitudePhaseMargin_deg & ...
    minimumBandwidthSeparationRatio >= ...
    altitude.design.minimumAttitudeToAltitudeBandwidthRatio;
selectedKhIndex = find(qualifyingKh,1,'last');
assert(~isempty(selectedKhIndex), ...
    'No altitude gain satisfies the robust margin and separation gates.');
selectedKh = KhGrid(selectedKhIndex);
assert(abs(selectedKh - altitude.altitudeGain_per_s) < 1.0e-12, ...
    'Stored altitude gain is not the largest qualifying grid value.');

Kh = altitude.altitudeGain_per_s;
Kv = altitude.verticalSpeedGain_radPerMps;

%% Detailed linear and protected operational validation

t = (0:altitude.validation.timeStep_s: ...
    altitude.validation.stopTime_s).';
smallCommand_m = altitude.validation.smallSignalCommand_m;
operationalCommand_m = altitude.validation.operationalCommand_m;
smallInput = smallCommand_m*ones(size(t));

isStable = false(nCase,1);
verticalSpeedPhaseMargin_deg = zeros(nCase,1);
altitudePhaseMargin_deg = zeros(nCase,1);
verticalSpeedCrossover_radps = zeros(nCase,1);
altitudeCrossover_radps = zeros(nCase,1);
bandwidthSeparationRatio = zeros(nCase,1);

smallOvershoot_pct = zeros(nCase,1);
smallRiseTime_s = zeros(nCase,1);
smallSettlingTime_s = zeros(nCase,1);
smallFinalError_ft = zeros(nCase,1);
smallPeakPitchCommand_deg = zeros(nCase,1);
smallPeakVerticalSpeed_fpm = zeros(nCase,1);
smallPeakElevator_deg = zeros(nCase,1);
smallPeakActuatorRate_degps = zeros(nCase,1);
smallPeakSpeedFraction = zeros(nCase,1);
smallPeakAlpha_deg = zeros(nCase,1);

operationalOvershoot_pct = zeros(nCase,1);
operationalRiseTime_s = zeros(nCase,1);
operationalSettlingTime_s = zeros(nCase,1);
operationalFinalError_ft = zeros(nCase,1);
operationalPeakPitchCommand_deg = zeros(nCase,1);
operationalPeakPitch_deg = zeros(nCase,1);
operationalPeakVerticalSpeed_fpm = zeros(nCase,1);
operationalPeakElevator_deg = zeros(nCase,1);
operationalPeakActuatorRate_degps = zeros(nCase,1);
operationalPeakSpeedFraction = zeros(nCase,1);
operationalPeakAlpha_deg = zeros(nCase,1);
verticalSpeedLimitActive = false(nCase,1);
pitchCommandLimitActive = false(nCase,1);
elevatorPositionLimitActive = false(nCase,1);
elevatorRateLimitActive = false(nCase,1);

smallHeightHistory_ft = zeros(numel(t),nCase);
operationalHeightHistory_ft = zeros(numel(t),nCase);

nominalSignals = struct();
for iCase = 1:nCase
    plant = plants{iCase};
    tauActuator = caseTau_s(iCase);
    [Aattitude,BthetaCommand] = actuatorClosedLoopMatrices( ...
        plant,Kp,Ki,Kq,tauTheta,tauActuator);
    CverticalSpeed = verticalSpeedOutput(Vtrim);
    verticalSpeedPlant = ss(Aattitude,BthetaCommand,CverticalSpeed,0);
    [~,verticalSpeedPhaseMargin_deg(iCase),~, ...
        verticalSpeedCrossover_radps(iCase)] = ...
        margin(Kv*verticalSpeedPlant);

    [Aheight,BverticalSpeedCommand,Cheight] = ...
        altitudeOuterPlantMatrices(Aattitude,BthetaCommand, ...
        CverticalSpeed,Kv);
    altitudePlant = ss(Aheight,BverticalSpeedCommand,Cheight,0);
    [~,altitudePhaseMargin_deg(iCase),~, ...
        altitudeCrossover_radps(iCase)] = margin(Kh*altitudePlant);
    bandwidthSeparationRatio(iCase) = attitudeCrossover_radps(iCase)/ ...
        altitudeCrossover_radps(iCase);

    [Aclosed,BheightCommand] = altitudeClosedLoopMatrices( ...
        Aattitude,BthetaCommand,CverticalSpeed,Kh,Kv,tauAltitude);
    poles = eig(Aclosed);
    isStable(iCase) = all(real(poles) < 0);
    smallSystem = ss(Aclosed,BheightCommand,eye(9),zeros(9,1));
    smallState = lsim(smallSystem,smallInput,t);
    smallSignals = reconstructLinearSignals(smallState,Kh,Kv,Vtrim, ...
        Kp,Ki,Kq,tauActuator);
    [smallRiseTime_s(iCase),smallSettlingTime_s(iCase), ...
        smallOvershoot_pct(iCase)] = stepMetrics( ...
        t,smallState(:,8),smallCommand_m);
    smallFinalError_ft(iCase) = ...
        abs(smallCommand_m-smallState(end,8))/0.3048;
    smallPeakPitchCommand_deg(iCase) = ...
        max(abs(rad2deg(smallSignals.thetaCommand)));
    smallPeakVerticalSpeed_fpm(iCase) = ...
        max(abs(smallSignals.verticalSpeed))*196.850393700787;
    smallPeakElevator_deg(iCase) = max(abs(rad2deg(smallState(:,7))));
    smallPeakActuatorRate_degps(iCase) = ...
        max(abs(rad2deg(smallSignals.actuatorRate)));
    smallPeakSpeedFraction(iCase) = max(abs(smallState(:,1)))/Vtrim;
    smallPeakAlpha_deg(iCase) = max(abs(rad2deg(smallState(:,2))));
    smallHeightHistory_ft(:,iCase) = smallState(:,8)/0.3048;

    [operationalState,operationalSignals] = simulateAltitudeCapture( ...
        t,plant,operationalCommand_m,Kh,Kv,tauAltitude, ...
        Kp,Ki,Kq,tauTheta,tauActuator,actuator,altitude,Vtrim);
    [operationalRiseTime_s(iCase),operationalSettlingTime_s(iCase), ...
        operationalOvershoot_pct(iCase)] = stepMetrics( ...
        t,operationalState(:,8),operationalCommand_m);
    operationalFinalError_ft(iCase) = ...
        abs(operationalCommand_m-operationalState(end,8))/0.3048;
    operationalPeakPitchCommand_deg(iCase) = ...
        max(abs(rad2deg(operationalSignals.thetaCommand)));
    operationalPeakPitch_deg(iCase) = ...
        max(abs(rad2deg(operationalState(:,4))));
    operationalPeakVerticalSpeed_fpm(iCase) = ...
        max(abs(operationalSignals.verticalSpeed))*196.850393700787;
    operationalPeakElevator_deg(iCase) = ...
        max(abs(rad2deg(operationalState(:,7))));
    operationalPeakActuatorRate_degps(iCase) = ...
        max(abs(rad2deg(operationalSignals.actuatorRate)));
    operationalPeakSpeedFraction(iCase) = ...
        max(abs(operationalState(:,1)))/Vtrim;
    operationalPeakAlpha_deg(iCase) = ...
        max(abs(rad2deg(operationalState(:,2))));
    verticalSpeedLimitActive(iCase) = operationalSignals.vzLimitActive;
    pitchCommandLimitActive(iCase) = operationalSignals.thetaLimitActive;
    elevatorPositionLimitActive(iCase) = ...
        operationalSignals.elevatorPositionLimitActive;
    elevatorRateLimitActive(iCase) = ...
        operationalSignals.elevatorRateLimitActive;
    operationalHeightHistory_ft(:,iCase) = operationalState(:,8)/0.3048;

    isNominal = abs(caseCG_in(iCase)-baselineData.cg.nominal_in) < 1.0e-12 ...
        && abs(caseIy_kgm2(iCase)-baselineData.inertia.Iy_kgm2) < 1.0e-9 ...
        && abs(caseCLdeltaE(iCase)-baselineData.aero.CL_deltaE) < 1.0e-12 ...
        && abs(caseTau_s(iCase)-actuator.dynamics.timeConstant_s) < 1.0e-12;
    if isNominal
        nominalSignals.t = t;
        nominalSignals.state = operationalState;
        nominalSignals.signals = operationalSignals;
    end
end

caseID = (1:nCase).';
results = table(caseID,caseCG_in,caseIy_kgm2,caseCLdeltaE,caseTau_s, ...
    isStable,verticalSpeedPhaseMargin_deg,altitudePhaseMargin_deg, ...
    verticalSpeedCrossover_radps,altitudeCrossover_radps, ...
    bandwidthSeparationRatio,smallOvershoot_pct,smallRiseTime_s, ...
    smallSettlingTime_s,smallFinalError_ft,smallPeakPitchCommand_deg, ...
    smallPeakVerticalSpeed_fpm,smallPeakElevator_deg, ...
    smallPeakActuatorRate_degps,smallPeakSpeedFraction,smallPeakAlpha_deg, ...
    operationalOvershoot_pct,operationalRiseTime_s, ...
    operationalSettlingTime_s,operationalFinalError_ft, ...
    operationalPeakPitchCommand_deg,operationalPeakPitch_deg, ...
    operationalPeakVerticalSpeed_fpm,operationalPeakElevator_deg, ...
    operationalPeakActuatorRate_degps,operationalPeakSpeedFraction, ...
    operationalPeakAlpha_deg,verticalSpeedLimitActive, ...
    pitchCommandLimitActive,elevatorPositionLimitActive, ...
    elevatorRateLimitActive);

%% Requirements

req = altitude.requirements;
assert(all(results.isStable),'At least one altitude-loop case is unstable.');
assert(all(results.verticalSpeedPhaseMargin_deg >= ...
    altitude.design.minimumVerticalSpeedFeedbackPhaseMargin_deg), ...
    'At least one case failed the vertical-speed feedback margin gate.');
assert(all(results.altitudePhaseMargin_deg >= ...
    altitude.design.minimumAltitudePhaseMargin_deg), ...
    'At least one case failed the altitude-loop phase-margin gate.');
assert(all(results.bandwidthSeparationRatio >= ...
    altitude.design.minimumAttitudeToAltitudeBandwidthRatio), ...
    'At least one case failed attitude-to-altitude bandwidth separation.');

assert(all(results.smallOvershoot_pct <= req.maximumSmallSignalOvershoot_pct), ...
    'A small-signal altitude response exceeded the overshoot gate.');
assert(all(results.smallRiseTime_s >= req.minimumSmallSignalRiseTime_s & ...
    results.smallRiseTime_s <= req.maximumSmallSignalRiseTime_s), ...
    'A small-signal altitude response failed the rise-time gate.');
assert(all(results.smallSettlingTime_s <= ...
    req.maximumSmallSignalSettlingTime_s), ...
    'A small-signal altitude response failed the settling-time gate.');
assert(all(results.smallFinalError_ft <= ...
    req.maximumSmallSignalFinalError_ft), ...
    'A small-signal altitude response failed final tracking.');
assert(all(results.smallPeakPitchCommand_deg <= ...
    req.maximumSmallSignalPitchCommand_deg), ...
    'A small-signal response used excessive pitch command.');
assert(all(results.smallPeakVerticalSpeed_fpm <= ...
    req.maximumSmallSignalVerticalSpeed_fpm), ...
    'A small-signal response used excessive vertical speed.');
assert(all(results.smallPeakElevator_deg < ...
    min(actuator.authority.uncertainty_deg)), ...
    'A small-signal response unexpectedly reached elevator authority.');
assert(all(results.smallPeakActuatorRate_degps < ...
    min(actuator.dynamics.rateLimitCases_degps)), ...
    'A small-signal response unexpectedly reached actuator rate limiting.');

assert(all(results.operationalOvershoot_pct <= ...
    req.maximumOperationalOvershoot_pct), ...
    'A protected 100-ft capture exceeded the overshoot gate.');
assert(all(results.operationalRiseTime_s >= ...
    req.minimumOperationalRiseTime_s & ...
    results.operationalRiseTime_s <= req.maximumOperationalRiseTime_s), ...
    'A protected 100-ft capture failed the rise-time gate.');
assert(all(results.operationalSettlingTime_s <= ...
    req.maximumOperationalSettlingTime_s), ...
    'A protected 100-ft capture failed the settling-time gate.');
assert(all(results.operationalFinalError_ft <= ...
    req.maximumOperationalFinalError_ft), ...
    'A protected 100-ft capture failed final tracking.');
assert(all(results.operationalPeakPitchCommand_deg <= ...
    req.maximumOperationalPitchCommand_deg + 1.0e-10), ...
    'A protected capture exceeded pitch-command authority.');
assert(all(results.operationalPeakPitch_deg <= ...
    req.maximumOperationalPitch_deg), ...
    'A protected capture exceeded the pitch-attitude gate.');
assert(all(results.operationalPeakVerticalSpeed_fpm <= ...
    req.maximumOperationalVerticalSpeed_fpm), ...
    'A protected capture exceeded the vertical-speed gate.');
assert(all(results.operationalPeakElevator_deg <= ...
    req.maximumOperationalElevator_deg), ...
    'A protected capture exceeded the elevator gate.');
assert(all(results.operationalPeakActuatorRate_degps <= ...
    req.maximumOperationalActuatorRate_degps), ...
    'A protected capture exceeded the actuator-rate gate.');
assert(all(results.operationalPeakSpeedFraction <= ...
    req.maximumSpeedPerturbationFraction), ...
    'A protected capture left the speed-perturbation envelope.');
assert(all(results.operationalPeakAlpha_deg <= ...
    req.maximumAlphaPerturbation_deg), ...
    'A protected capture left the alpha envelope.');
assert(all(results.verticalSpeedLimitActive), ...
    'The 100-ft capture did not exercise the vertical-speed command cap.');
assert(~any(results.pitchCommandLimitActive), ...
    'The normal 100-ft capture unexpectedly reached pitch-command limiting.');
assert(~any(results.elevatorPositionLimitActive), ...
    'The normal 100-ft capture unexpectedly reached elevator authority.');
assert(~any(results.elevatorRateLimitActive), ...
    'The normal 100-ft capture unexpectedly reached actuator rate limiting.');

%% Console report and result ledger

fprintf('============================================================\n');
fprintf('          C172S ALTITUDE HOLD V1 - ROBUST DESIGN\n');
fprintf('============================================================\n\n');
fprintf('Architecture                         : bounded altitude/vertical-speed outer loop\n');
fprintf('Frozen inner loop                    : Pitch-Attitude Hold V2\n');
fprintf('Altitude kinematics                  : hDot = Vtrim*(theta - alpha)\n');
fprintf('Altitude gain Kh                     : %.4f 1/s\n',Kh);
fprintf('Vertical-speed gain Kv               : %.4f rad/(m/s)\n',Kv);
fprintf('Altitude command-filter time constant: %.2f s\n',tauAltitude);
fprintf('Vertical-speed command authority     : +/- %.0f ft/min\n', ...
    altitude.verticalSpeedCommandLimit_fpm);
fprintf('Pitch-command authority              : +/- %.1f deg\n\n', ...
    altitude.pitchCommandLimit_deg);

fprintf('Stable uncertainty cases             : %d/%d\n', ...
    nnz(results.isStable),height(results));
fprintf('Vertical-speed feedback phase margin : %.2f to %.2f deg\n', ...
    min(results.verticalSpeedPhaseMargin_deg), ...
    max(results.verticalSpeedPhaseMargin_deg));
fprintf('Altitude-loop phase margin           : %.2f to %.2f deg\n', ...
    min(results.altitudePhaseMargin_deg), ...
    max(results.altitudePhaseMargin_deg));
fprintf('Altitude gain crossover              : %.4f to %.4f rad/s\n', ...
    min(results.altitudeCrossover_radps), ...
    max(results.altitudeCrossover_radps));
fprintf('Attitude/altitude bandwidth ratio    : %.3f to %.3f\n\n', ...
    min(results.bandwidthSeparationRatio), ...
    max(results.bandwidthSeparationRatio));

fprintf('Linear command                       : %.1f ft\n', ...
    altitude.validation.smallSignalCommand_ft);
fprintf('Overshoot                            : %.3f to %.3f %%\n', ...
    min(results.smallOvershoot_pct),max(results.smallOvershoot_pct));
fprintf('10-90%% rise time                     : %.3f to %.3f s\n', ...
    min(results.smallRiseTime_s),max(results.smallRiseTime_s));
fprintf('2%% settling time                     : %.3f to %.3f s\n', ...
    min(results.smallSettlingTime_s),max(results.smallSettlingTime_s));
fprintf('Peak pitch command                   : %.3f to %.3f deg\n', ...
    min(results.smallPeakPitchCommand_deg), ...
    max(results.smallPeakPitchCommand_deg));
fprintf('Peak vertical speed                  : %.2f to %.2f ft/min\n\n', ...
    min(results.smallPeakVerticalSpeed_fpm), ...
    max(results.smallPeakVerticalSpeed_fpm));

fprintf('Protected operational command        : %.1f ft\n', ...
    altitude.validation.operationalCommand_ft);
fprintf('Overshoot                            : %.3f to %.3f %%\n', ...
    min(results.operationalOvershoot_pct), ...
    max(results.operationalOvershoot_pct));
fprintf('10-90%% rise time                     : %.3f to %.3f s\n', ...
    min(results.operationalRiseTime_s), ...
    max(results.operationalRiseTime_s));
fprintf('2%% settling time                     : %.3f to %.3f s\n', ...
    min(results.operationalSettlingTime_s), ...
    max(results.operationalSettlingTime_s));
fprintf('Final altitude error                 : %.3f to %.3f ft\n', ...
    min(results.operationalFinalError_ft), ...
    max(results.operationalFinalError_ft));
fprintf('Peak pitch command                   : %.3f to %.3f deg\n', ...
    min(results.operationalPeakPitchCommand_deg), ...
    max(results.operationalPeakPitchCommand_deg));
fprintf('Peak aircraft pitch                  : %.3f to %.3f deg\n', ...
    min(results.operationalPeakPitch_deg), ...
    max(results.operationalPeakPitch_deg));
fprintf('Peak vertical speed                  : %.2f to %.2f ft/min\n', ...
    min(results.operationalPeakVerticalSpeed_fpm), ...
    max(results.operationalPeakVerticalSpeed_fpm));
fprintf('Peak actual elevator                 : %.3f to %.3f deg\n', ...
    min(results.operationalPeakElevator_deg), ...
    max(results.operationalPeakElevator_deg));
fprintf('Peak actuator rate                   : %.3f to %.3f deg/s\n', ...
    min(results.operationalPeakActuatorRate_degps), ...
    max(results.operationalPeakActuatorRate_degps));
fprintf('Peak |u|/Vtrim                       : %.2f to %.2f %%\n', ...
    100*min(results.operationalPeakSpeedFraction), ...
    100*max(results.operationalPeakSpeedFraction));
fprintf('Peak alpha perturbation              : %.3f to %.3f deg\n\n', ...
    min(results.operationalPeakAlpha_deg), ...
    max(results.operationalPeakAlpha_deg));

fprintf('PASS: all 108 linear altitude-loop cases satisfy the design gates.\n');
fprintf('PASS: all 108 protected 100-ft captures remain inside the linear envelope.\n');
fprintf('PASS: only the intended vertical-speed command cap activates normally.\n');
fprintf('PASS: the frozen attitude, elevator-position and actuator-rate protections are preserved.\n\n');

outputFile = fullfile(pwd,'altitude_hold_robustness_results.csv');
writetable(results,outputFile);
fprintf('Saved altitude ledger: %s\n',outputFile);

%% Figures

figure('Name','C172S Altitude-Hold Gain Selection','Color','w');
tiledlayout(2,1);
nexttile;
plot(KvGrid,minimumVerticalSpeedPhaseMargin_deg,'LineWidth',1.5);
hold on;
yline(altitude.design.minimumVerticalSpeedFeedbackPhaseMargin_deg, ...
    'k--','Margin requirement');
xline(Kv,'--','Selected K_v');
grid on;
xlabel('Vertical-speed feedback gain K_v (rad/(m/s))');
ylabel('Worst-case phase margin (deg)');
title('Vertical-Speed Damping Gain Selection');
nexttile;
yyaxis left;
plot(KhGrid,minimumAltitudePhaseMargin_deg,'LineWidth',1.5);
yline(altitude.design.minimumAltitudePhaseMargin_deg,'--', ...
    'Phase-margin requirement');
ylabel('Worst-case altitude phase margin (deg)');
yyaxis right;
plot(KhGrid,minimumBandwidthSeparationRatio,'LineWidth',1.5);
yline(altitude.design.minimumAttitudeToAltitudeBandwidthRatio,'--', ...
    'Separation requirement');
ylabel('Worst-case attitude/altitude bandwidth ratio');
xline(Kh,'k--','Selected K_h');
grid on;
xlabel('Altitude gain K_h (1/s)');
title('Altitude Gain Selection');

figure('Name','C172S Robust Altitude Responses','Color','w');
tiledlayout(2,1);
nexttile;
plot(t,smallHeightHistory_ft,'Color',[0.75 0.82 0.90], ...
    'HandleVisibility','off');
hold on;
yline(altitude.validation.smallSignalCommand_ft,'k--','Command');
grid on;
ylabel('Altitude perturbation (ft)');
title('Linear 10-ft Altitude Command Across 108 Cases');
nexttile;
plot(t,operationalHeightHistory_ft,'Color',[0.75 0.82 0.90], ...
    'HandleVisibility','off');
hold on;
yline(altitude.validation.operationalCommand_ft,'k--','Command');
grid on;
xlabel('Time (s)');
ylabel('Altitude perturbation (ft)');
title('Protected 100-ft Altitude Capture Across 108 Cases');

assert(isfield(nominalSignals,'state'), ...
    'Unable to identify the nominal altitude-hold case.');
figure('Name','C172S Nominal Altitude Capture','Color','w');
tiledlayout(4,1);
nexttile;
plot(nominalSignals.t,nominalSignals.state(:,8)/0.3048,'LineWidth',1.4);
hold on;
yline(altitude.validation.operationalCommand_ft,'k--','Command');
grid on;
ylabel('h (ft)');
title('Nominal Protected Altitude Capture');
nexttile;
plot(nominalSignals.t, ...
    nominalSignals.signals.verticalSpeed*196.850393700787,'LineWidth',1.4);
hold on;
yline(altitude.verticalSpeedCommandLimit_fpm,'k--', ...
    'Command authority','HandleVisibility','off');
yline(-altitude.verticalSpeedCommandLimit_fpm,'k--', ...
    'HandleVisibility','off');
grid on;
ylabel('hDot (ft/min)');
title('Vertical Speed');
nexttile;
plot(nominalSignals.t,rad2deg(nominalSignals.signals.thetaCommand), ...
    'LineWidth',1.4,'DisplayName','Pitch command');
hold on;
plot(nominalSignals.t,rad2deg(nominalSignals.state(:,4)), ...
    'LineWidth',1.2,'DisplayName','Aircraft pitch');
grid on;
ylabel('Angle (deg)');
title('Pitch Command and Response');
legend('Location','best');
nexttile;
plot(nominalSignals.t,rad2deg(nominalSignals.state(:,7)), ...
    'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('delta_e (deg)');
title('Actual Elevator');

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

function CverticalSpeed = verticalSpeedOutput(Vtrim)
CverticalSpeed = [0,-Vtrim,0,Vtrim,0,0,0];
end

function [Aheight,BverticalSpeedCommand,Cheight] = ...
    altitudeOuterPlantMatrices(Aattitude,BthetaCommand, ...
    CverticalSpeed,Kv)
% Vertical-speed feedback is closed before opening the altitude loop.
AverticalSpeed = Aattitude - BthetaCommand*(Kv*CverticalSpeed);
BverticalSpeed = BthetaCommand*Kv;
Aheight = zeros(8,8);
Aheight(1:7,1:7) = AverticalSpeed;
Aheight(8,1:7) = CverticalSpeed;
BverticalSpeedCommand = zeros(8,1);
BverticalSpeedCommand(1:7) = BverticalSpeed;
Cheight = [zeros(1,7),1];
end

function [Aclosed,BheightCommand] = altitudeClosedLoopMatrices( ...
    Aattitude,BthetaCommand,CverticalSpeed,Kh,Kv,tauAltitude)
% State order: [attitude-loop seven states, h, hCommandFiltered].
Aclosed = zeros(9,9);
Aclosed(1:7,1:7) = Aattitude - ...
    BthetaCommand*(Kv*CverticalSpeed);
Aclosed(1:7,8) = -BthetaCommand*Kv*Kh;
Aclosed(1:7,9) = BthetaCommand*Kv*Kh;
Aclosed(8,1:7) = CverticalSpeed;
Aclosed(9,9) = -1.0/tauAltitude;
BheightCommand = zeros(9,1);
BheightCommand(9) = 1.0/tauAltitude;
end

function signals = reconstructLinearSignals( ...
    state,Kh,Kv,Vtrim,Kp,Ki,Kq,tauActuator)
signals.verticalSpeed = Vtrim*(state(:,4)-state(:,2));
signals.verticalSpeedCommand = Kh*(state(:,9)-state(:,8));
signals.thetaCommand = Kv*( ...
    signals.verticalSpeedCommand-signals.verticalSpeed);
eTheta = state(:,6)-state(:,4);
signals.unsaturatedElevator = -Kp*eTheta-Ki*state(:,5)+Kq*state(:,3);
signals.actuatorRate = ...
    (signals.unsaturatedElevator-state(:,7))/tauActuator;
end

function [state,signals] = simulateAltitudeCapture( ...
    t,plant,heightCommand,Kh,Kv,tauAltitude,Kp,Ki,Kq,tauTheta, ...
    tauActuator,actuator,altitude,Vtrim)
state = zeros(numel(t),9);
dt = t(2)-t(1);
for i = 1:numel(t)-1
    k1 = altitudeDerivative(state(i,:).',heightCommand,plant,Kh,Kv, ...
        tauAltitude,Kp,Ki,Kq,tauTheta,tauActuator,actuator,altitude,Vtrim);
    k2 = altitudeDerivative(state(i,:).' + 0.5*dt*k1,heightCommand, ...
        plant,Kh,Kv,tauAltitude,Kp,Ki,Kq,tauTheta,tauActuator, ...
        actuator,altitude,Vtrim);
    k3 = altitudeDerivative(state(i,:).' + 0.5*dt*k2,heightCommand, ...
        plant,Kh,Kv,tauAltitude,Kp,Ki,Kq,tauTheta,tauActuator, ...
        actuator,altitude,Vtrim);
    k4 = altitudeDerivative(state(i,:).' + dt*k3,heightCommand,plant, ...
        Kh,Kv,tauAltitude,Kp,Ki,Kq,tauTheta,tauActuator, ...
        actuator,altitude,Vtrim);
    state(i+1,:) = state(i,:) + ...
        (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4).';
end
signals = reconstructLimitedSignals( ...
    state,Kh,Kv,Vtrim,Kp,Ki,Kq,tauActuator,actuator,altitude);
end

function dz = altitudeDerivative(z,heightCommand,plant,Kh,Kv, ...
    tauAltitude,Kp,Ki,Kq,tauTheta,tauActuator,actuator,altitude,Vtrim)
signals = reconstructLimitedSignals( ...
    z.',Kh,Kv,Vtrim,Kp,Ki,Kq,tauActuator,actuator,altitude);
dz = zeros(9,1);
dz(1:4) = plant.A*z(1:4) + plant.B*z(7);
eTheta = z(6)-z(4);
dz(5) = eTheta + signals.antiWindupCorrection;
dz(6) = (signals.thetaCommand-z(6))/tauTheta;
dz(7) = signals.actuatorRate;
dz(8) = signals.verticalSpeed;
dz(9) = (heightCommand-z(9))/tauAltitude;
end

function signals = reconstructLimitedSignals( ...
    state,Kh,Kv,Vtrim,Kp,Ki,Kq,tauActuator,actuator,altitude)
signals.verticalSpeed = Vtrim*(state(:,4)-state(:,2));
signals.verticalSpeedCommandRaw = Kh*(state(:,9)-state(:,8));
signals.verticalSpeedCommand = clamp( ...
    signals.verticalSpeedCommandRaw, ...
    -altitude.verticalSpeedCommandLimit_mps, ...
    altitude.verticalSpeedCommandLimit_mps);
signals.thetaCommandRaw = Kv*( ...
    signals.verticalSpeedCommand-signals.verticalSpeed);
signals.thetaCommand = clamp(signals.thetaCommandRaw, ...
    -altitude.pitchCommandLimit_rad,altitude.pitchCommandLimit_rad);
eTheta = state(:,6)-state(:,4);
signals.unsaturatedElevator = -Kp*eTheta-Ki*state(:,5)+Kq*state(:,3);
signals.positionCommand = clamp(signals.unsaturatedElevator, ...
    actuator.authority.lowerLimit_rad,actuator.authority.upperLimit_rad);
signals.rawActuatorRate = ...
    (signals.positionCommand-state(:,7))/tauActuator;
rateLimit = deg2rad(actuator.dynamics.rateLimit_degps);
signals.actuatorRate = clamp(signals.rawActuatorRate,-rateLimit,rateLimit);
trackedElevator = state(:,7)+tauActuator*signals.actuatorRate;
signals.antiWindupCorrection = ...
    (signals.unsaturatedElevator-trackedElevator)/ ...
    (Ki*actuator.antiWindup.trackingTimeConstant_s);
tol = 1.0e-10;
signals.vzLimitActive = any(abs(signals.verticalSpeedCommandRaw) > ...
    altitude.verticalSpeedCommandLimit_mps + tol);
signals.thetaLimitActive = any(abs(signals.thetaCommandRaw) > ...
    altitude.pitchCommandLimit_rad + tol);
signals.elevatorPositionLimitActive = any( ...
    signals.unsaturatedElevator < actuator.authority.lowerLimit_rad-tol | ...
    signals.unsaturatedElevator > actuator.authority.upperLimit_rad+tol);
signals.elevatorRateLimitActive = any( ...
    abs(signals.rawActuatorRate) > rateLimit+tol);
end

function [Aouter,Bouter,Couter] = actuatorOuterPlantMatrices( ...
    plant,Kq,tauActuator)
Aouter = zeros(5,5);
Aouter(1:4,1:4) = plant.A;
Aouter(1:4,5) = plant.B;
Aouter(5,3) = Kq/tauActuator;
Aouter(5,5) = -1.0/tauActuator;
Bouter = zeros(5,1);
Bouter(5) = 1.0/tauActuator;
Couter = [0,0,0,-1,0];
end

function y = clamp(x,lowerLimit,upperLimit)
y = min(max(x,lowerLimit),upperLimit);
end

function [riseTime,settlingTime,overshoot] = ...
    stepMetrics(t,response,command)
rise10Index = find(response >= 0.10*command,1,'first');
rise90Index = find(response >= 0.90*command,1,'first');
if isempty(rise10Index) || isempty(rise90Index)
    riseTime = inf;
else
    riseTime = t(rise90Index)-t(rise10Index);
end
outside = find(abs(response-command) > 0.02*abs(command));
if isempty(outside)
    settlingTime = 0.0;
elseif outside(end) == numel(t)
    settlingTime = inf;
else
    settlingTime = t(outside(end)+1);
end
overshoot = max(0.0,100.0*(max(response)-command)/abs(command));
end
