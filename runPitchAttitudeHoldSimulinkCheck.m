%% C172S Pitch-Attitude Hold V1 - Simulink equivalence check

clc;
clear;
close all;

fprintf('\n*** RUNNING PITCH-ATTITUDE HOLD V1 CHECKER REVISION R2 ***\n\n');

try
    load_system('simulink');
catch ME
    error('runPitchAttitudeHoldSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

data = c172sLongitudinalData();
[~,plant] = c172sLongitudinalPlant(data);
damper = c172sPitchRateDamperData();
attitude = c172sPitchAttitudeControllerData();

Kp = attitude.Kp_radPerRad;
Ki = attitude.Ki_radPerRadPerS;
Kq = damper.Kq_s;
tauCommand = attitude.commandFilterTimeConstant_s;
thetaCommand = deg2rad(attitude.validation.commandStep_deg);

%% Initialize block variables and rebuild the generated diagram

assignin('base','paData',data);
assignin('base','paPlant',plant);
assignin('base','paDamper',damper);
assignin('base','paController',attitude);
assignin('base','A_pa',plant.A);
assignin('base','B_pa',plant.B);
assignin('base','C_pa',plant.C);
assignin('base','D_pa',plant.D);
assignin('base','Kq_pa_s',Kq);
assignin('base','Kp_pa',Kp);
assignin('base','Ki_pa_per_s',Ki);
assignin('base','tauThetaCommand_pa_s',tauCommand);
assignin('base','thetaCommandStep_pa_rad',thetaCommand);
assignin('base','deltaEBias_pa_rad',0.0);
assignin('base','x0_pa',zeros(4,1));
assignin('base','xi0_pa',0.0);

modelName = buildC172sPitchAttitudeHoldSimulink(true);

%% Run Simulink

simOut = sim(modelName, ...
    'StopTime',num2str(attitude.validation.stopTime_s), ...
    'ReturnWorkspaceOutputs','on');

tSim = simOut.tout;
ySim = simOut.yout;

if isa(ySim,'timeseries')
    ySim = ySim.Data;
elseif isa(ySim,'Simulink.SimulationData.Dataset')
    ySim = ySim.getElement(1).Values.Data;
end

ySim = squeeze(ySim);
if size(ySim,1) ~= numel(tSim) && size(ySim,2) == numel(tSim)
    ySim = ySim.';
end

if size(ySim,2) ~= 8
    error(['Expected eight columns ordered as [u, alpha, q, theta, ', ...
        'thetaFilteredCommand, xi, deltaECommand, deltaETotal].']);
end

%% Independent exact six-state reference

% Reference state order:
%   z = [u; alpha; q; theta; xi; thetaFilteredCommand]
Ctheta = [0,0,0,1];
Aq = plant.A + plant.B*damper.qFeedbackRow;

Aclosed = zeros(6,6);
Aclosed(1:4,1:4) = Aq + plant.B*Kp*Ctheta;
Aclosed(1:4,5) = -plant.B*Ki;
Aclosed(1:4,6) = -plant.B*Kp;
Aclosed(5,1:4) = -Ctheta;
Aclosed(5,6) = 1.0;
Aclosed(6,6) = -1.0/tauCommand;

Bcommand = zeros(6,1);
Bcommand(6) = 1.0/tauCommand;

closedPoles = eig(Aclosed);
assert(all(real(closedPoles) < 0), ...
    'The nominal pitch-attitude closed-loop reference is not stable.');

dt = diff(tSim);
nominalDt = median(dt);
assert(max(abs(dt - nominalDt)) < 1.0e-10, ...
    'The exact reference requires a uniformly sampled time vector.');

nState = size(Aclosed,1);
augmentedMatrix = [Aclosed,Bcommand;zeros(1,nState + 1)];
transition = expm(augmentedMatrix*nominalDt);
Ad = transition(1:nState,1:nState);
Bd = transition(1:nState,end);

zReference = zeros(numel(tSim),nState);
z = zeros(nState,1);
for k = 1:numel(tSim)-1
    z = Ad*z + Bd*thetaCommand;
    zReference(k+1,:) = z.';
end

thetaFilteredReference = zReference(:,6);
thetaErrorReference = thetaFilteredReference - zReference(:,4);
deltaECommandReference = -Kp*thetaErrorReference ...
    - Ki*zReference(:,5);
deltaETotalReference = deltaECommandReference ...
    + Kq*zReference(:,3);

yReference = zeros(size(ySim));
yReference(:,1:4) = zReference(:,1:4);
yReference(:,5) = thetaFilteredReference;
yReference(:,6) = zReference(:,5);
yReference(:,7) = deltaECommandReference;
yReference(:,8) = deltaETotalReference;

%% Numerical equivalence gates

signalError = ySim - yReference;
maximumErrorBySignal = max(abs(signalError),[],1);
maxAbsError = max(maximumErrorBySignal);
rmsError = sqrt(mean(signalError(:).^2));

% The command filter removes the discontinuity before it reaches the
% feedback dynamics. Fixed-step RK4 at 0.01 s should satisfy this gate by
% a comfortable margin.
tolerance = 1.0e-4;
assert(maxAbsError < tolerance, ...
    ['Simulink/MATLAB pitch-attitude mismatch: maximum absolute ', ...
     'signal error %.6g exceeds %.6g.'],maxAbsError,tolerance);

%% Controller direction and nominal performance gates

theta = ySim(:,4);
thetaFiltered = ySim(:,5);
xi = ySim(:,6);
deltaECommand = ySim(:,7);
deltaETotal = ySim(:,8);
q = ySim(:,3);
qDamperElevator = Kq*q;

thetaError = thetaFiltered - theta;
firstPositiveError = find(thetaError > deg2rad(0.01),1,'first');
assert(~isempty(firstPositiveError), ...
    'The test did not generate a positive pitch-attitude error.');
assert(deltaECommand(firstPositiveError) < 0, ...
    ['Controller sign failed: positive pitch-attitude error must command ', ...
     'trailing-edge-up elevator.']);

firstPositiveQ = find(q > deg2rad(0.01),1,'first');
assert(~isempty(firstPositiveQ), ...
    'The command test did not produce a positive pitch rate.');
assert(qDamperElevator(firstPositiveQ) > 0, ...
    'q-damper sign failed: positive q must command trailing-edge-down elevator.');

[riseTime_s,settlingTime_s,overshoot_pct] = ...
    stepMetrics(tSim,theta,thetaCommand);
peakPitchRate_degps = max(abs(rad2deg(q)));
peakElevatorCommand_deg = max(abs(rad2deg(deltaECommand)));
peakTotalElevator_deg = max(abs(rad2deg(deltaETotal)));
peakSpeedPerturbation_mps = max(abs(ySim(:,1)));
peakSpeedPerturbationFraction = ...
    peakSpeedPerturbation_mps/data.trim.V_mps;
peakAlphaPerturbation_deg = max(abs(rad2deg(ySim(:,2))));
finalAttitudeError_deg = abs(rad2deg(thetaCommand - theta(end)));

req = attitude.requirements;
assert(overshoot_pct <= req.maximumStepOvershoot_pct, ...
    'Nominal Simulink overshoot exceeded the V1 requirement.');
assert(riseTime_s >= req.minimumRiseTime_s ...
    && riseTime_s <= req.maximumRiseTime_s, ...
    'Nominal Simulink rise time failed the V1 requirement.');
assert(settlingTime_s <= req.maximumSettlingTime_s, ...
    'Nominal Simulink settling time exceeded the V1 requirement.');
assert(peakPitchRate_degps <= req.maximumPitchRate_degps, ...
    'Nominal Simulink pitch rate exceeded the V1 requirement.');
assert(peakTotalElevator_deg <= req.maximumElevator_deg, ...
    'Nominal Simulink elevator demand exceeded the V1 requirement.');
assert(peakSpeedPerturbationFraction <= ...
    req.maximumSpeedPerturbationFraction, ...
    'Nominal Simulink response left the allowed speed-perturbation envelope.');
assert(peakAlphaPerturbation_deg <= req.maximumAlphaPerturbation_deg, ...
    'Nominal Simulink response left the allowed angle-of-attack envelope.');
assert(finalAttitudeError_deg <= req.maximumFinalAttitudeError_deg, ...
    'Nominal Simulink final attitude error exceeded the V1 requirement.');

% Independent algebraic reconstruction of the controller output.
deltaECommandReconstructed = -Kp*(thetaFiltered - theta) - Ki*xi;
controllerReconstructionError = max(abs( ...
    deltaECommand - deltaECommandReconstructed));
assert(controllerReconstructionError < 1.0e-12, ...
    'Logged PI command does not satisfy the implemented controller law.');

%% Nominal modal and loop-margin diagnostics

controllerPoles = eig(Aclosed(1:5,1:5));
positivePoles = controllerPoles(imag(controllerPoles) > 1.0e-8);
assert(~isempty(positivePoles), ...
    'Expected a stable oscillatory fast mode in the nominal controller.');
[~,fastIndex] = max(abs(imag(positivePoles)));
fastPole = positivePoles(fastIndex);
fastModeWn = abs(fastPole);
fastModeZeta = -real(fastPole)/fastModeWn;

Gtheta = ss(Aq,plant.B,Ctheta,0);
Cpi = tf([Kp,Ki],[1,0]);
[gainMargin,phaseMargin_deg,~,gainCrossover_radps] = ...
    margin((-Gtheta)*Cpi);
gainMargin_dB = 20*log10(gainMargin);

qdPositivePoles = eig(Aq);
qdPositivePoles = qdPositivePoles(imag(qdPositivePoles) > 1.0e-8);
[~,qdFastIndex] = max(abs(imag(qdPositivePoles)));
qdFastPole = qdPositivePoles(qdFastIndex);
bandwidthSeparationRatio = abs(qdFastPole)/gainCrossover_radps;

assert(fastModeZeta >= req.minimumFastModeZeta, ...
    'Nominal Simulink fast-mode damping failed the V1 requirement.');
assert(phaseMargin_deg >= req.minimumPhaseMargin_deg, ...
    'Nominal Simulink phase margin failed the V1 requirement.');
assert(gainCrossover_radps <= req.maximumGainCrossover_radps, ...
    'Nominal Simulink gain crossover exceeded the V1 requirement.');
assert(bandwidthSeparationRatio >= ...
    req.minimumBandwidthSeparationRatio, ...
    'Nominal Simulink bandwidth separation failed the V1 requirement.');

%% Console report

fprintf('============================================================\n');
fprintf(' C172S PITCH-ATTITUDE HOLD V1 - SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                            : %s\n',modelName);
fprintf('Architecture                     : Filtered-command PI + frozen q damper\n');
fprintf('Kp                               : %.4f rad/rad\n',Kp);
fprintf('Ki                               : %.4f 1/s\n',Ki);
fprintf('Kq                               : %.4f s\n',Kq);
fprintf('Command-filter time constant     : %.3f s\n',tauCommand);
fprintf('Command                          : %.2f deg\n',rad2deg(thetaCommand));
fprintf('Solver                           : Fixed-step ode4\n');
fprintf('Fixed step                       : %.3f s\n\n',nominalDt);

fprintf('Maximum absolute signal error    : %.9g\n',maxAbsError);
fprintf('RMS signal error                 : %.9g\n',rmsError);
fprintf('Allowed error                    : %.3g\n',tolerance);
fprintf('PI reconstruction error          : %.9g\n\n', ...
    controllerReconstructionError);

fprintf('Fast-mode wn                     : %.4f rad/s\n',fastModeWn);
fprintf('Fast-mode damping                : %.4f\n',fastModeZeta);
fprintf('Phase margin                     : %.2f deg\n',phaseMargin_deg);
fprintf('Gain margin                      : %.2f dB\n',gainMargin_dB);
fprintf('Gain crossover                   : %.4f rad/s\n',gainCrossover_radps);
fprintf('Bandwidth separation ratio       : %.3f\n\n', ...
    bandwidthSeparationRatio);

fprintf('Overshoot                        : %.4f %%\n',overshoot_pct);
fprintf('10-90%% rise time                 : %.3f s\n',riseTime_s);
fprintf('2%% settling time                 : %.3f s\n',settlingTime_s);
fprintf('Peak pitch rate                  : %.4f deg/s\n',peakPitchRate_degps);
fprintf('Peak PI elevator command         : %.4f deg\n', ...
    peakElevatorCommand_deg);
fprintf('Peak total elevator              : %.4f deg\n',peakTotalElevator_deg);
fprintf('Peak speed perturbation          : %.4f m/s (%.2f %% Vtrim)\n', ...
    peakSpeedPerturbation_mps,100*peakSpeedPerturbationFraction);
fprintf('Peak angle-of-attack perturbation: %.4f deg\n', ...
    peakAlphaPerturbation_deg);
fprintf('Final attitude error             : %.5f deg\n', ...
    finalAttitudeError_deg);
fprintf('Final speed perturbation         : %.4f m/s\n',ySim(end,1));
fprintf('Final angle-of-attack perturbation: %.4f deg\n\n', ...
    rad2deg(ySim(end,2)));

fprintf('PASS: Simulink matches the independent exact six-state reference.\n');
fprintf('PASS: the PI and q-damper feedback directions are correct.\n');
fprintf('PASS: the nominal command response satisfies all V1 requirements.\n');
fprintf('PASS: the command response remains inside the linear-model envelope.\n');
fprintf('PASS: all eight logged signals match their MATLAB definitions.\n');

%% Figure 1: complete plant-state equivalence

figure('Name','C172S Attitude-Hold Simulink Equivalence','Color','w');
tiledlayout(2,2);

stateLabels = {'u (m/s)','alpha (deg)','q (deg/s)','theta (deg)'};
plotDataSim = ySim(:,1:4);
plotDataReference = yReference(:,1:4);
plotDataSim(:,2:4) = rad2deg(plotDataSim(:,2:4));
plotDataReference(:,2:4) = rad2deg(plotDataReference(:,2:4));

for k = 1:4
    nexttile;
    plot(tSim,plotDataSim(:,k),'LineWidth',1.3);
    hold on;
    plot(tSim,plotDataReference(:,k),'--','LineWidth',1.0);
    grid on;
    xlabel('Time (s)');
    ylabel(stateLabels{k});
    if k == 1
        speedLimit_mps = ...
            req.maximumSpeedPerturbationFraction*data.trim.V_mps;
        yline(speedLimit_mps,'k:','Envelope limit', ...
            'HandleVisibility','off');
        yline(-speedLimit_mps,'k:','HandleVisibility','off');
    elseif k == 2
        yline(req.maximumAlphaPerturbation_deg,'k:','Envelope limit', ...
            'HandleVisibility','off');
        yline(-req.maximumAlphaPerturbation_deg,'k:', ...
            'HandleVisibility','off');
    end
    if k == 1
        legend('Simulink','MATLAB exact','Location','best');
    end
end

sgtitle('Pitch-Attitude Hold V1: Plant-State Equivalence');

%% Figure 2: attitude tracking and controller activity

figure('Name','C172S Attitude-Hold Controller Signals','Color','w');
tiledlayout(4,1);

nexttile;
plot(tSim,rad2deg(thetaCommand)*ones(size(tSim)),'k--', ...
    'LineWidth',1.0,'DisplayName','Command');
hold on;
plot(tSim,rad2deg(thetaFiltered),'LineWidth',1.2, ...
    'DisplayName','Filtered command');
plot(tSim,rad2deg(theta),'LineWidth',1.4, ...
    'DisplayName','Simulink attitude');
plot(tSim,rad2deg(yReference(:,4)),':','LineWidth',1.1, ...
    'DisplayName','MATLAB exact');
grid on;
ylabel('\theta (deg)');
title('Pitch-Attitude Tracking');
legend('Location','best');

nexttile;
plot(tSim,rad2deg(q),'LineWidth',1.4);
grid on;
ylabel('q (deg/s)');
title('Pitch Rate');

nexttile;
plot(tSim,rad2deg(deltaECommand),'LineWidth',1.3, ...
    'DisplayName','PI command');
hold on;
plot(tSim,rad2deg(qDamperElevator),'LineWidth',1.1, ...
    'DisplayName','q-damper contribution');
plot(tSim,rad2deg(deltaETotal),'LineWidth',1.4, ...
    'DisplayName','Total elevator');
grid on;
ylabel('\delta_e (deg)');
title('Elevator Contributions, Trailing-Edge Down Positive');
legend('Location','best');

nexttile;
plot(tSim,rad2deg(xi),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('\xi (deg s)');
title('Attitude-Error Integrator State');

%% Refresh the saved model after all simulations and plotting

modelFile = get_param(modelName,'FileName');
close_system(modelName,0);
load_system(modelFile);
open_system(modelName);
fprintf('Reloaded generated model to clear transient editor diagnostics.\n');

%% Local helper

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
