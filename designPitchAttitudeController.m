%% C172S Pitch-Attitude Hold V1 - robust PI design
%
% Architecture:
%
%   thetaCommand -> first-order command filter -> PI attitude controller
%                -> deltaECommand -> frozen q damper -> aircraft
%
% Sign convention:
%
%   eTheta        = thetaFilteredCommand - theta
%   deltaECommand = -Kp*eTheta - Ki*integral(eTheta)
%   deltaE        = deltaECommand + Kq*q
%
% Positive elevator is trailing-edge down. The leading minus signs in the
% attitude controller are therefore required for negative feedback.

clc;
clear;
close all;

baselineData = c172sLongitudinalData();
damper = c172sPitchRateDamperData();
attitude = c172sPitchAttitudeControllerData();

assert(exist('ss','file') == 2 && exist('tf','file') == 2, ...
    'Control System Toolbox is required for the attitude-controller design.');

cgValues_in = [ ...
    baselineData.cg.forwardLimit_in, ...
    baselineData.cg.nominal_in, ...
    baselineData.cg.aftLimit_in];

iyValues_kgm2 = unique([ ...
    baselineData.uncertainty.Iy_range_kgm2(1), ...
    baselineData.inertia.Iy_kgm2, ...
    2300.0, ...
    baselineData.uncertainty.Iy_range_kgm2(2)], 'stable');

clDeltaEValues = [ ...
    baselineData.uncertainty.CL_deltaE_range(1), ...
    baselineData.aero.CL_deltaE, ...
    baselineData.uncertainty.CL_deltaE_range(2)];

%% Robust search in the constrained PI family Kp = Ki

gainGrid = ( ...
    attitude.design.gainSearchRange(1): ...
    attitude.design.gainSearchIncrement: ...
    attitude.design.gainSearchRange(2)).';

minimumFastModeZeta = nan(size(gainGrid));
allCasesStableAtGain = false(size(gainGrid));
allCasesOscillatoryAtGain = false(size(gainGrid));

for iGain = 1:numel(gainGrid)
    gain = gainGrid(iGain);
    caseZeta = [];
    stableAtGain = true;
    oscillatoryAtGain = true;

    for iCG = 1:numel(cgValues_in)
        for iIy = 1:numel(iyValues_kgm2)
            for iCLde = 1:numel(clDeltaEValues)
                data = c172sLongitudinalData(cgValues_in(iCG));
                data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
                data.aero.CL_deltaE = clDeltaEValues(iCLde);
                [~,plant] = c172sLongitudinalPlant(data);

                Acontroller = controllerStateMatrix( ...
                    plant,damper,gain,gain);
                poles = eig(Acontroller);

                stableAtGain = stableAtGain && all(real(poles) < 0);
                positivePoles = poles(imag(poles) > 1.0e-8);
                if isempty(positivePoles)
                    oscillatoryAtGain = false;
                else
                    fastPole = selectFastPole(positivePoles);
                    caseZeta(end+1,1) = ...
                        -real(fastPole)/abs(fastPole); %#ok<SAGROW>
                end
            end
        end
    end

    allCasesStableAtGain(iGain) = stableAtGain;
    allCasesOscillatoryAtGain(iGain) = oscillatoryAtGain;
    if stableAtGain && oscillatoryAtGain
        minimumFastModeZeta(iGain) = min(caseZeta);
    end
end

qualifyingGain = allCasesStableAtGain & allCasesOscillatoryAtGain ...
    & minimumFastModeZeta >= ...
    attitude.design.minimumSearchFastModeZeta;

lastQualifyingIndex = find(qualifyingGain,1,'last');
if isempty(lastQualifyingIndex)
    error('No gain in the search interval satisfies the design target.');
end

maximumQualifyingGain = gainGrid(lastQualifyingIndex);
Kp = attitude.Kp_radPerRad;
Ki = attitude.Ki_radPerRadPerS;

assert(abs(Kp - Ki) < 1.0e-12, ...
    'This V1 search assumes the constrained controller family Kp = Ki.');
assert(Kp <= maximumQualifyingGain + 1.0e-12, ...
    'The selected PI gain exceeds the robust search limit.');
assert(maximumQualifyingGain - Kp < ...
    attitude.design.gainSearchIncrement + 1.0e-12, ...
    'The stored PI gain is not the largest qualifying search-grid value.');

%% Detailed 36-case modal, margin and command-response validation

nCase = numel(cgValues_in)*numel(iyValues_kgm2)* ...
    numel(clDeltaEValues);

caseID = (1:nCase).';
cgIn = zeros(nCase,1);
iyKgm2 = zeros(nCase,1);
clDeltaE = zeros(nCase,1);
isStable = false(nCase,1);
fastModeWn = zeros(nCase,1);
fastModeZeta = zeros(nCase,1);
phaseMargin_deg = zeros(nCase,1);
gainMargin_dB = zeros(nCase,1);
gainCrossover_radps = zeros(nCase,1);
qdShortPeriodWn = zeros(nCase,1);
bandwidthSeparationRatio = zeros(nCase,1);
overshoot_pct = zeros(nCase,1);
riseTime_s = zeros(nCase,1);
settlingTime_s = zeros(nCase,1);
peakPitchRate_degps = zeros(nCase,1);
peakElevatorCommand_deg = zeros(nCase,1);
peakTotalElevator_deg = zeros(nCase,1);
peakSpeedPerturbation_mps = zeros(nCase,1);
peakSpeedPerturbationFraction = zeros(nCase,1);
peakAlphaPerturbation_deg = zeros(nCase,1);
finalSpeedPerturbation_mps = zeros(nCase,1);
finalAlphaPerturbation_deg = zeros(nCase,1);
finalAttitudeError_deg = zeros(nCase,1);
finalInitialDisturbance_deg = zeros(nCase,1);
finalBiasResponse_deg = zeros(nCase,1);

t = (0:attitude.validation.timeStep_s: ...
    attitude.validation.stopTime_s).';
thetaCommand = deg2rad(attitude.validation.commandStep_deg);
commandInput = thetaCommand*ones(size(t));

thetaHistory_deg = zeros(numel(t),nCase);
qHistory_degps = zeros(numel(t),nCase);
elevatorHistory_deg = zeros(numel(t),nCase);
speedHistory_mps = zeros(numel(t),nCase);
alphaHistory_deg = zeros(numel(t),nCase);

row = 0;
for iCG = 1:numel(cgValues_in)
    for iIy = 1:numel(iyValues_kgm2)
        for iCLde = 1:numel(clDeltaEValues)
            row = row + 1;

            data = c172sLongitudinalData(cgValues_in(iCG));
            data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
            data.aero.CL_deltaE = clDeltaEValues(iCLde);
            [~,plant] = c172sLongitudinalPlant(data);

            Aq = plant.A + plant.B*damper.qFeedbackRow;
            Acontroller = controllerStateMatrix(plant,damper,Kp,Ki);
            controllerPoles = eig(Acontroller);
            positiveControllerPoles = ...
                controllerPoles(imag(controllerPoles) > 1.0e-8);
            if isempty(positiveControllerPoles)
                error('Case %d lost the fast oscillatory mode.',row);
            end
            fastPole = selectFastPole(positiveControllerPoles);

            qdPositivePoles = eig(Aq);
            qdPositivePoles = qdPositivePoles( ...
                imag(qdPositivePoles) > 1.0e-8);
            qdFastPole = selectFastPole(qdPositivePoles);

            % The sign-reversed plant Ptheta = -theta/deltaECommand has
            % positive DC gain and gives the conventional 1 + L loop form.
            Ctheta = [0,0,0,1];
            Gtheta = ss(Aq,plant.B,Ctheta,0);
            Ptheta = -Gtheta;
            Cpi = tf([Kp,Ki],[1,0]);
            [gainMargin,phaseMargin,~,gainCrossover] = margin(Ptheta*Cpi);

            [Acommand,Bcommand] = commandFilteredMatrices( ...
                plant,damper,attitude);
            commandSystem = ss(Acommand,Bcommand,eye(6),zeros(6,1));
            z = lsim(commandSystem,commandInput,t);

            theta = z(:,4);
            thetaFiltered = z(:,6);
            thetaError = thetaFiltered - theta;
            deltaECommand = -Kp*thetaError - Ki*z(:,5);
            deltaETotal = deltaECommand + damper.Kq_s*z(:,3);

            [caseRiseTime,caseSettlingTime,caseOvershoot] = ...
                stepMetrics(t,theta,thetaCommand);

            % Regulation checks: an initial pitch-angle error and a
            % constant elevator bias must both be rejected by the PI loop.
            zeroInputSystem = ss(Acommand,zeros(6,1), ...
                eye(6),zeros(6,1));
            initialState = zeros(6,1);
            initialState(4) = deg2rad( ...
                attitude.validation.initialAttitudeDisturbance_deg);
            initialResponse = initial(zeroInputSystem,initialState,t);

            elevatorBiasInput = [plant.B;0;0];
            biasSystem = ss(Acommand,elevatorBiasInput, ...
                eye(6),zeros(6,1));
            biasResponse = lsim(biasSystem, ...
                deg2rad(attitude.validation.constantElevatorBias_deg) ...
                *ones(size(t)),t);

            cgIn(row) = data.cg.selected_in;
            iyKgm2(row) = data.inertia.Iy_kgm2;
            clDeltaE(row) = data.aero.CL_deltaE;
            isStable(row) = all(real(controllerPoles) < 0) ...
                && all(real(eig(Acommand)) < 0);
            fastModeWn(row) = abs(fastPole);
            fastModeZeta(row) = -real(fastPole)/abs(fastPole);
            phaseMargin_deg(row) = phaseMargin;
            gainMargin_dB(row) = 20*log10(gainMargin);
            gainCrossover_radps(row) = gainCrossover;
            qdShortPeriodWn(row) = abs(qdFastPole);
            bandwidthSeparationRatio(row) = ...
                qdShortPeriodWn(row)/gainCrossover_radps(row);
            overshoot_pct(row) = caseOvershoot;
            riseTime_s(row) = caseRiseTime;
            settlingTime_s(row) = caseSettlingTime;
            peakPitchRate_degps(row) = max(abs(rad2deg(z(:,3))));
            peakElevatorCommand_deg(row) = ...
                max(abs(rad2deg(deltaECommand)));
            peakTotalElevator_deg(row) = ...
                max(abs(rad2deg(deltaETotal)));
            peakSpeedPerturbation_mps(row) = max(abs(z(:,1)));
            peakSpeedPerturbationFraction(row) = ...
                peakSpeedPerturbation_mps(row)/data.trim.V_mps;
            peakAlphaPerturbation_deg(row) = ...
                max(abs(rad2deg(z(:,2))));
            finalSpeedPerturbation_mps(row) = z(end,1);
            finalAlphaPerturbation_deg(row) = rad2deg(z(end,2));
            finalAttitudeError_deg(row) = ...
                abs(rad2deg(thetaCommand - theta(end)));
            finalInitialDisturbance_deg(row) = ...
                abs(rad2deg(initialResponse(end,4)));
            finalBiasResponse_deg(row) = ...
                abs(rad2deg(biasResponse(end,4)));

            thetaHistory_deg(:,row) = rad2deg(theta);
            qHistory_degps(:,row) = rad2deg(z(:,3));
            elevatorHistory_deg(:,row) = rad2deg(deltaETotal);
            speedHistory_mps(:,row) = z(:,1);
            alphaHistory_deg(:,row) = rad2deg(z(:,2));
        end
    end
end

results = table( ...
    caseID,cgIn,iyKgm2,clDeltaE,isStable, ...
    fastModeWn,fastModeZeta,phaseMargin_deg,gainMargin_dB, ...
    gainCrossover_radps,qdShortPeriodWn,bandwidthSeparationRatio, ...
    overshoot_pct,riseTime_s,settlingTime_s,peakPitchRate_degps, ...
    peakElevatorCommand_deg,peakTotalElevator_deg, ...
    peakSpeedPerturbation_mps,peakSpeedPerturbationFraction, ...
    peakAlphaPerturbation_deg,finalSpeedPerturbation_mps, ...
    finalAlphaPerturbation_deg, ...
    finalAttitudeError_deg,finalInitialDisturbance_deg, ...
    finalBiasResponse_deg);

%% Requirements and regression gates

req = attitude.requirements;
assert(all(results.isStable), ...
    'At least one attitude-loop uncertainty case is unstable.');
assert(all(results.fastModeZeta >= req.minimumFastModeZeta), ...
    'At least one case failed the minimum fast-mode damping requirement.');
assert(all(results.phaseMargin_deg >= req.minimumPhaseMargin_deg), ...
    'At least one case failed the minimum phase-margin requirement.');
assert(all(results.gainCrossover_radps <= ...
    req.maximumGainCrossover_radps), ...
    'At least one case exceeded the maximum outer-loop crossover.');
assert(all(results.bandwidthSeparationRatio >= ...
    req.minimumBandwidthSeparationRatio), ...
    'At least one case failed the minimum bandwidth-separation ratio.');
assert(all(results.overshoot_pct <= req.maximumStepOvershoot_pct), ...
    'At least one case exceeded the step-overshoot requirement.');
assert(all(results.riseTime_s >= req.minimumRiseTime_s ...
    & results.riseTime_s <= req.maximumRiseTime_s), ...
    'At least one case failed the command rise-time window.');
assert(all(results.settlingTime_s <= req.maximumSettlingTime_s), ...
    'At least one case exceeded the settling-time requirement.');
assert(all(results.peakPitchRate_degps <= ...
    req.maximumPitchRate_degps), ...
    'At least one case exceeded the pitch-rate requirement.');
assert(all(results.peakTotalElevator_deg <= ...
    req.maximumElevator_deg), ...
    'At least one case exceeded the elevator-demand requirement.');
assert(all(results.peakSpeedPerturbationFraction <= ...
    req.maximumSpeedPerturbationFraction), ...
    'At least one case left the allowed speed-perturbation envelope.');
assert(all(results.peakAlphaPerturbation_deg <= ...
    req.maximumAlphaPerturbation_deg), ...
    'At least one case left the allowed angle-of-attack envelope.');
assert(all(results.finalAttitudeError_deg <= ...
    req.maximumFinalAttitudeError_deg), ...
    'At least one case failed the final command-tracking requirement.');
assert(all(results.finalInitialDisturbance_deg <= ...
    req.maximumFinalAttitudeError_deg), ...
    'At least one case failed to reject the initial attitude disturbance.');
assert(all(results.finalBiasResponse_deg <= ...
    req.maximumFinalAttitudeError_deg), ...
    'At least one case failed to reject the constant elevator bias.');
assert(-Kp*deg2rad(1.0) < 0, ...
    'A positive pitch-attitude error must command trailing-edge-up elevator.');

%% Console report and machine-readable ledger

fprintf('============================================================\n');
fprintf('      C172S PITCH-ATTITUDE HOLD V1 - ROBUST DESIGN\n');
fprintf('============================================================\n\n');
fprintf('Architecture                       : Filtered-command PI + frozen q damper\n');
fprintf('Controller law                     : deltaECmd = -Kp*eTheta - Ki*integral(eTheta)\n');
fprintf('Pitch-rate damper                  : Kq = %.4f s\n',damper.Kq_s);
fprintf('Selected Kp                        : %.4f rad/rad\n',Kp);
fprintf('Selected Ki                        : %.4f 1/s\n',Ki);
fprintf('PI zero                            : %.4f rad/s\n',Ki/Kp);
fprintf('Command-filter time constant       : %.3f s\n', ...
    attitude.commandFilterTimeConstant_s);
fprintf('Maximum robust search gain         : %.4f\n\n', ...
    maximumQualifyingGain);

fprintf('Stable uncertainty cases           : %d/%d\n', ...
    nnz(results.isStable),height(results));
fprintf('Fast-mode damping                  : %.4f to %.4f\n', ...
    min(results.fastModeZeta),max(results.fastModeZeta));
fprintf('Fast-mode natural frequency        : %.4f to %.4f rad/s\n', ...
    min(results.fastModeWn),max(results.fastModeWn));
fprintf('Phase margin                       : %.2f to %.2f deg\n', ...
    min(results.phaseMargin_deg),max(results.phaseMargin_deg));
fprintf('Gain crossover                     : %.4f to %.4f rad/s\n', ...
    min(results.gainCrossover_radps),max(results.gainCrossover_radps));
fprintf('Bandwidth separation ratio         : %.3f to %.3f\n\n', ...
    min(results.bandwidthSeparationRatio), ...
    max(results.bandwidthSeparationRatio));

fprintf('Command step                       : %.2f deg\n', ...
    attitude.validation.commandStep_deg);
fprintf('Overshoot                          : %.3f to %.3f %%\n', ...
    min(results.overshoot_pct),max(results.overshoot_pct));
fprintf('10-90%% rise time                   : %.3f to %.3f s\n', ...
    min(results.riseTime_s),max(results.riseTime_s));
fprintf('2%% settling time                   : %.3f to %.3f s\n', ...
    min(results.settlingTime_s),max(results.settlingTime_s));
fprintf('Peak pitch rate                    : %.3f to %.3f deg/s\n', ...
    min(results.peakPitchRate_degps),max(results.peakPitchRate_degps));
fprintf('Peak total elevator                : %.3f to %.3f deg\n', ...
    min(results.peakTotalElevator_deg),max(results.peakTotalElevator_deg));
fprintf('Peak |u|/Vtrim                    : %.2f to %.2f %%\n', ...
    100*min(results.peakSpeedPerturbationFraction), ...
    100*max(results.peakSpeedPerturbationFraction));
fprintf('Peak angle-of-attack perturbation  : %.3f to %.3f deg\n', ...
    min(results.peakAlphaPerturbation_deg), ...
    max(results.peakAlphaPerturbation_deg));
fprintf('Final attitude error               : %.5f to %.5f deg\n\n', ...
    min(results.finalAttitudeError_deg), ...
    max(results.finalAttitudeError_deg));

fprintf('PASS: all 36 uncertainty cases satisfy the attitude-loop gates.\n');
fprintf('PASS: the outer loop preserves robust fast-mode damping and separation.\n');
fprintf('PASS: the PI loop rejects initial-attitude and elevator-bias disturbances.\n');
fprintf('PASS: all command responses remain inside the linear-model envelope.\n');
fprintf('PASS: positive attitude error commands trailing-edge-up elevator.\n\n');

outputFile = fullfile(pwd,'pitch_attitude_controller_robustness_results.csv');
writetable(results,outputFile);
fprintf('Saved controller ledger: %s\n',outputFile);

%% Figure 1: constrained robust gain search

figure('Name','C172S Pitch-Attitude PI Gain Selection','Color','w');
plot(gainGrid,minimumFastModeZeta,'LineWidth',1.4);
hold on;
yline(attitude.design.minimumSearchFastModeZeta,'k--', ...
    'Design target','HandleVisibility','off');
yline(req.minimumFastModeZeta,':','Hard requirement', ...
    'HandleVisibility','off');
xline(Kp,'--','Selected gain','HandleVisibility','off');
grid on;
xlabel('Constrained gain, K_p = K_i');
ylabel('Worst-case fast-mode damping ratio');
title('Robust PI Gain Selection');

%% Figure 2: all robust pitch-attitude command responses

figure('Name','C172S Robust Pitch-Attitude Responses','Color','w');
hold on;
plot(t,thetaHistory_deg,'Color',[0.75 0.82 0.90], ...
    'HandleVisibility','off');

nominalSelection = abs(results.cgIn - baselineData.cg.nominal_in) < 1.0e-12 ...
    & abs(results.iyKgm2 - baselineData.inertia.Iy_kgm2) < 1.0e-9 ...
    & abs(results.clDeltaE - baselineData.aero.CL_deltaE) < 1.0e-12;
nominalIndex = find(nominalSelection,1,'first');
assert(~isempty(nominalIndex),'Unable to identify the nominal case.');

plot(t,thetaHistory_deg(:,nominalIndex),'LineWidth',1.7, ...
    'Color',[0 0.447 0.741],'DisplayName','Nominal response');
yline(attitude.validation.commandStep_deg,'k--', ...
    'DisplayName','Command');
grid on;
xlabel('Time (s)');
ylabel('\theta (deg)');
title(sprintf('%.1f-Degree Pitch-Attitude Command Across 36 Cases', ...
    attitude.validation.commandStep_deg));
legend('Location','best');

%% Figure 3: nominal command response and control activity

figure('Name','C172S Nominal Pitch-Attitude Hold','Color','w');
tiledlayout(3,1);

nexttile;
plot(t,attitude.validation.commandStep_deg*ones(size(t)),'k--', ...
    'LineWidth',1.0,'DisplayName','Command');
hold on;
plot(t,thetaHistory_deg(:,nominalIndex),'LineWidth',1.5, ...
    'DisplayName','Aircraft attitude');
grid on;
ylabel('\theta (deg)');
title('Nominal Pitch-Attitude Tracking');
legend('Location','best');

nexttile;
plot(t,qHistory_degps(:,nominalIndex),'LineWidth',1.4);
grid on;
ylabel('q (deg/s)');
title('Pitch Rate');

nexttile;
plot(t,elevatorHistory_deg(:,nominalIndex),'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('\delta_e (deg)');
title('Total Elevator, Trailing-Edge Down Positive');

%% Figure 4: robust small-disturbance envelope

figure('Name','C172S Pitch-Attitude Linear Envelope','Color','w');
tiledlayout(2,1);

nexttile;
plot(t,speedHistory_mps,'Color',[0.75 0.82 0.90], ...
    'HandleVisibility','off');
hold on;
plot(t,speedHistory_mps(:,nominalIndex),'LineWidth',1.6, ...
    'DisplayName','Nominal response');
speedLimit_mps = req.maximumSpeedPerturbationFraction*baselineData.trim.V_mps;
yline(speedLimit_mps,'k--','Envelope limit','HandleVisibility','off');
yline(-speedLimit_mps,'k--','HandleVisibility','off');
grid on;
ylabel('u (m/s)');
title('Forward-Speed Perturbation Across 36 Cases');
legend('Location','best');

nexttile;
plot(t,alphaHistory_deg,'Color',[0.75 0.82 0.90], ...
    'HandleVisibility','off');
hold on;
plot(t,alphaHistory_deg(:,nominalIndex),'LineWidth',1.6, ...
    'DisplayName','Nominal response');
yline(req.maximumAlphaPerturbation_deg,'k--','Envelope limit', ...
    'HandleVisibility','off');
yline(-req.maximumAlphaPerturbation_deg,'k--','HandleVisibility','off');
grid on;
xlabel('Time (s)');
ylabel('\alpha (deg)');
title('Angle-of-Attack Perturbation Across 36 Cases');
legend('Location','best');

%% Figure 5: robust controller pole map

figure('Name','C172S Pitch-Attitude Robust Pole Map','Color','w');
hold on;
grid on;
for iCase = 1:height(results)
    data = c172sLongitudinalData(results.cgIn(iCase));
    data.inertia.Iy_kgm2 = results.iyKgm2(iCase);
    data.aero.CL_deltaE = results.clDeltaE(iCase);
    [~,plant] = c172sLongitudinalPlant(data);
    poles = eig(controllerStateMatrix(plant,damper,Kp,Ki));
    plot(real(poles),imag(poles),'o','Color',[0 0.447 0.741], ...
        'MarkerSize',4,'HandleVisibility','off');
end
xline(0,'k--','HandleVisibility','off');
xlabel('Real axis (1/s)');
ylabel('Imaginary axis (rad/s)');
title('Pitch-Attitude Closed-Loop Poles Across 36 Cases');

%% Local helpers

function Acontroller = controllerStateMatrix(plant,damper,Kp,Ki)
% State order: [u, alpha, q, theta, xi].
Ctheta = [0,0,0,1];
Aq = plant.A + plant.B*damper.qFeedbackRow;
Acontroller = [ ...
    Aq + plant.B*Kp*Ctheta, -plant.B*Ki; ...
    -Ctheta,                 0];
end

function [Acommand,Bcommand] = commandFilteredMatrices( ...
    plant,damper,attitude)
% State order: [u, alpha, q, theta, xi, thetaFilteredCommand].
Kp = attitude.Kp_radPerRad;
Ki = attitude.Ki_radPerRadPerS;
tau = attitude.commandFilterTimeConstant_s;
Ctheta = [0,0,0,1];
Aq = plant.A + plant.B*damper.qFeedbackRow;

Acommand = zeros(6,6);
Acommand(1:4,1:4) = Aq + plant.B*Kp*Ctheta;
Acommand(1:4,5) = -plant.B*Ki;
Acommand(1:4,6) = -plant.B*Kp;
Acommand(5,1:4) = -Ctheta;
Acommand(5,6) = 1.0;
Acommand(6,6) = -1.0/tau;

Bcommand = zeros(6,1);
Bcommand(6) = 1.0/tau;
end

function fastPole = selectFastPole(positivePoles)
[~,index] = max(abs(imag(positivePoles)));
fastPole = positivePoles(index);
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
