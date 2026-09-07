%% C172S Pitch-Attitude Hold V2 - Simulink actuator/protection check

clc;
clear;
close all;

fprintf('\n*** RUNNING PITCH-ATTITUDE ACTUATOR CHECKER REVISION R2 ***\n\n');

try
    load_system('simulink');
catch ME
    error('runPitchAttitudeActuatorSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

data = c172sLongitudinalData();
[~,plant] = c172sLongitudinalPlant(data);
controller = c172sPitchAttitudeControllerData();
actuator = c172sElevatorActuatorData();

Kp = controller.Kp_radPerRad;
Ki = controller.Ki_radPerRadPerS;
Kq = actuator.integration.Kq_s;
tauCommand = controller.commandFilterTimeConstant_s;
tauActuator = actuator.dynamics.timeConstant_s;
Tt = actuator.antiWindup.trackingTimeConstant_s;
thetaCommand = deg2rad(actuator.validation.commandStep_deg);

modelName = buildC172sPitchAttitudeActuatorSimulink(true);

%% Normal one-degree command

normalInput = Simulink.SimulationInput(modelName);
normalInput = normalInput.setVariable('thetaCommand_pa2_rad',thetaCommand);
normalInput = normalInput.setVariable('antiWindupEnable_pa2',1.0);
normalInput = normalInput.setVariable('x0_pa2',zeros(4,1));
normalInput = normalInput.setVariable('xi0_pa2',0.0);
normalInput = normalInput.setVariable('deltaE0_pa2',0.0);
normalInput = normalInput.setModelParameter( ...
    'StopTime',num2str(actuator.validation.stopTime_s));
normalOut = sim(normalInput);
[tNormal,yNormal] = extractArrayOutput(normalOut,13);

z0Normal = zeros(7,1);
yNormalReference = rk4Reference(tNormal,z0Normal,thetaCommand,true, ...
    plant,controller,actuator);

normalError = yNormal - yNormalReference;
normalMaxError = max(abs(normalError),[],'all');
normalRmsError = sqrt(mean(normalError(:).^2));
tolerance = 1.0e-5;
assert(normalMaxError < tolerance, ...
    'Normal Simulink/reference mismatch %.6g exceeds %.6g.', ...
    normalMaxError,tolerance);

%% Protection stress case, with and without anti-windup

x0Stress = zeros(4,1);
x0Stress(4) = deg2rad(actuator.stress.initialAttitude_deg);

stressAwInput = Simulink.SimulationInput(modelName);
stressAwInput = stressAwInput.setVariable('thetaCommand_pa2_rad',0.0);
stressAwInput = stressAwInput.setVariable('antiWindupEnable_pa2',1.0);
stressAwInput = stressAwInput.setVariable('x0_pa2',x0Stress);
stressAwInput = stressAwInput.setVariable('xi0_pa2',0.0);
stressAwInput = stressAwInput.setVariable('deltaE0_pa2',0.0);
stressAwInput = stressAwInput.setModelParameter( ...
    'StopTime',num2str(actuator.stress.stopTime_s));
stressAwOut = sim(stressAwInput);
[tStressAw,yStressAw] = extractArrayOutput(stressAwOut,13);

z0Stress = zeros(7,1);
z0Stress(4) = x0Stress(4);
yStressAwReference = rk4Reference(tStressAw,z0Stress,0.0,true, ...
    plant,controller,actuator);
stressAwMaxError = max(abs(yStressAw - yStressAwReference),[],'all');
assert(stressAwMaxError < tolerance, ...
    'Protected stress Simulink/reference mismatch exceeds the tolerance.');

stressNoAwInput = Simulink.SimulationInput(modelName);
stressNoAwInput = stressNoAwInput.setVariable('thetaCommand_pa2_rad',0.0);
stressNoAwInput = stressNoAwInput.setVariable('antiWindupEnable_pa2',0.0);
stressNoAwInput = stressNoAwInput.setVariable('x0_pa2',x0Stress);
stressNoAwInput = stressNoAwInput.setVariable('xi0_pa2',0.0);
stressNoAwInput = stressNoAwInput.setVariable('deltaE0_pa2',0.0);
stressNoAwInput = stressNoAwInput.setModelParameter( ...
    'StopTime',num2str(actuator.stress.stopTime_s));
stressNoAwOut = sim(stressNoAwInput);
[tStressNoAw,yStressNoAw] = extractArrayOutput(stressNoAwOut,13);

yStressNoAwReference = rk4Reference(tStressNoAw,z0Stress,0.0,false, ...
    plant,controller,actuator);
stressNoAwMaxError = max(abs(yStressNoAw - yStressNoAwReference),[],'all');
assert(stressNoAwMaxError < tolerance, ...
    'Unprotected stress Simulink/reference mismatch exceeds the tolerance.');

%% Normal response requirements

% Columns:
% [u alpha q theta filtered xi PI qContribution unsat position actual rate AW]
theta = yNormal(:,4);
thetaFiltered = yNormal(:,5);
xi = yNormal(:,6);
deltaEPI = yNormal(:,7);
qContribution = yNormal(:,8);
deltaEUnsat = yNormal(:,9);
deltaEPositionCommand = yNormal(:,10);
deltaEActual = yNormal(:,11);
deltaERate = yNormal(:,12);
antiWindupCorrection = yNormal(:,13);
q = yNormal(:,3);

controllerReconstructionError = max(abs(deltaEPI - ...
    (-Kp*(thetaFiltered - theta) - Ki*xi)));
totalCommandReconstructionError = max(abs(deltaEUnsat - ...
    (deltaEPI + qContribution)));
qReconstructionError = max(abs(qContribution - Kq*q));
assert(controllerReconstructionError < 1.0e-12, ...
    'PI command reconstruction failed.');
assert(totalCommandReconstructionError < 1.0e-12, ...
    'Total command reconstruction failed.');
assert(qReconstructionError < 1.0e-12, ...
    'q-damper command reconstruction failed.');

firstPositiveError = find(thetaFiltered - theta > deg2rad(0.01),1,'first');
assert(~isempty(firstPositiveError) && deltaEPI(firstPositiveError) < 0, ...
    'Positive attitude error did not command trailing-edge-up elevator.');
firstPositiveQ = find(q > deg2rad(0.01),1,'first');
assert(~isempty(firstPositiveQ) && qContribution(firstPositiveQ) > 0, ...
    'Positive q did not command the required damping elevator.');

[riseTime_s,settlingTime_s,overshoot_pct] = ...
    stepMetrics(tNormal,theta,thetaCommand);
peakPitchRate_degps = max(abs(rad2deg(q)));
peakActualElevator_deg = max(abs(rad2deg(deltaEActual)));
peakActuatorRate_degps = max(abs(rad2deg(deltaERate)));
peakSpeedFraction = max(abs(yNormal(:,1)))/data.trim.V_mps;
peakAlpha_deg = max(abs(rad2deg(yNormal(:,2))));
finalAttitudeError_deg = abs(rad2deg(thetaCommand - theta(end)));
positionLimitActiveNormal = any(abs( ...
    deltaEPositionCommand - deltaEUnsat) > 1.0e-10);
rawRateNormal = (deltaEPositionCommand - deltaEActual)/tauActuator;
rateLimitActiveNormal = any(abs(rawRateNormal - deltaERate) > 1.0e-8);

req = actuator.requirements;
assert(overshoot_pct <= req.maximumStepOvershoot_pct, ...
    'Normal overshoot exceeded the V2 requirement.');
assert(riseTime_s >= req.minimumRiseTime_s && ...
    riseTime_s <= req.maximumRiseTime_s, ...
    'Normal rise time failed the V2 requirement.');
assert(settlingTime_s <= req.maximumSettlingTime_s, ...
    'Normal settling time exceeded the V2 requirement.');
assert(peakPitchRate_degps <= req.maximumPitchRate_degps, ...
    'Normal pitch rate exceeded the V2 requirement.');
assert(peakActualElevator_deg <= req.maximumAutomaticElevator_deg, ...
    'Normal elevator demand exceeded the V2 requirement.');
assert(peakActuatorRate_degps <= req.maximumNominalActuatorRate_degps, ...
    'Normal actuator rate exceeded the V2 requirement.');
assert(peakSpeedFraction <= req.maximumSpeedPerturbationFraction, ...
    'Normal response left the speed perturbation envelope.');
assert(peakAlpha_deg <= req.maximumAlphaPerturbation_deg, ...
    'Normal response left the alpha envelope.');
assert(finalAttitudeError_deg <= req.maximumFinalAttitudeError_deg, ...
    'Normal final attitude error exceeded the V2 requirement.');
assert(~positionLimitActiveNormal && ~rateLimitActiveNormal, ...
    'Protection unexpectedly activated during the one-degree command.');
assert(max(abs(antiWindupCorrection)) < 1.0e-10, ...
    'Anti-windup correction should be zero while limits are inactive.');

%% Stress-test anti-windup metrics

stressAw = stressMetrics(tStressAw,yStressAw,actuator,tauActuator);
stressNoAw = stressMetrics(tStressNoAw,yStressNoAw,actuator,tauActuator);

assert(stressAw.positionLimitActive && stressAw.rateLimitActive, ...
    'The stress test did not activate both actuator limits.');
assert(stressAw.peakIntegrator_deg_s <= ...
    actuator.stress.maximumPeakIntegratorRatio* ...
    stressNoAw.peakIntegrator_deg_s, ...
    'Simulink anti-windup did not sufficiently limit integrator growth.');
assert(stressAw.positionSaturationDuration_s <= ...
    actuator.stress.maximumSaturationDurationRatio* ...
    stressNoAw.positionSaturationDuration_s, ...
    'Simulink anti-windup did not sufficiently shorten saturation.');
assert(abs(stressAw.finalAttitude_deg) <= ...
    actuator.stress.maximumFinalAttitude_deg, ...
    'Protected stress response did not recover near zero attitude.');

%% Nominal modal and margin diagnostics

[Aclosed,~] = actuatorClosedLoopMatrices(plant,Kp,Ki,Kq, ...
    tauCommand,tauActuator);
poles = eig(Aclosed);
positivePoles = poles(imag(poles) > 1.0e-8);
fastPole = selectFastPole(positivePoles);
fastModeWn = abs(fastPole);
fastModeZeta = -real(fastPole)/fastModeWn;

[Aouter,Bouter,Couter] = actuatorOuterPlantMatrices(plant,Kq,tauActuator);
Ptheta = ss(Aouter,Bouter,Couter,0);
Cpi = tf([Kp,Ki],[1,0]);
[gainMargin,phaseMargin_deg,~,gainCrossover_radps] = margin(Ptheta*Cpi);
gainMargin_dB = 20*log10(gainMargin);
outerPlantPoles = eig(Aouter);
innerFrequency = slowestNonPhugoidFrequency(outerPlantPoles);
bandwidthSeparationRatio = innerFrequency/gainCrossover_radps;

assert(all(real(poles) < 0),'Nominal actuator-integrated loop is unstable.');
assert(fastModeZeta >= req.minimumFastModeZeta, ...
    'Nominal fast-mode damping failed.');
assert(phaseMargin_deg >= req.minimumPhaseMargin_deg, ...
    'Nominal phase margin failed.');
assert(gainCrossover_radps <= req.maximumGainCrossover_radps, ...
    'Nominal crossover failed.');
assert(bandwidthSeparationRatio >= req.minimumBandwidthSeparationRatio, ...
    'Nominal bandwidth separation failed.');

%% Console report

fprintf('============================================================\n');
fprintf(' C172S PITCH-ATTITUDE HOLD V2 - SIMULINK ACTUATOR CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                              : %s\n',modelName);
fprintf('Kp / Ki                            : %.4f / %.4f 1/s\n',Kp,Ki);
fprintf('Actuator-integrated Kq             : %.4f s\n',Kq);
fprintf('Actuator time constant             : %.3f s\n',tauActuator);
fprintf('Automatic authority               : %.1f deg up / %.1f deg down\n', ...
    actuator.authority.trailingEdgeUp_deg, ...
    actuator.authority.trailingEdgeDown_deg);
fprintf('Rate limit                         : %.1f deg/s\n', ...
    actuator.dynamics.rateLimit_degps);
fprintf('Anti-windup tracking time constant : %.3f s\n',Tt);
fprintf('Solver / fixed step                : ode4 / %.4f s\n\n', ...
    median(diff(tNormal)));

fprintf('Normal maximum signal error        : %.9g\n',normalMaxError);
fprintf('Normal RMS signal error            : %.9g\n',normalRmsError);
fprintf('Protected stress maximum error     : %.9g\n',stressAwMaxError);
fprintf('Unprotected stress maximum error   : %.9g\n',stressNoAwMaxError);
fprintf('Allowed error                      : %.3g\n\n',tolerance);

fprintf('Fast-mode wn                       : %.4f rad/s\n',fastModeWn);
fprintf('Fast-mode damping                  : %.4f\n',fastModeZeta);
fprintf('Phase margin                       : %.2f deg\n',phaseMargin_deg);
fprintf('Gain margin                        : %.2f dB\n',gainMargin_dB);
fprintf('Gain crossover                     : %.4f rad/s\n',gainCrossover_radps);
fprintf('Bandwidth separation ratio         : %.3f\n\n', ...
    bandwidthSeparationRatio);

fprintf('Command                            : %.2f deg\n', ...
    actuator.validation.commandStep_deg);
fprintf('Overshoot                          : %.4f %%\n',overshoot_pct);
fprintf('10-90%% rise time                   : %.3f s\n',riseTime_s);
fprintf('2%% settling time                   : %.3f s\n',settlingTime_s);
fprintf('Peak pitch rate                    : %.4f deg/s\n',peakPitchRate_degps);
fprintf('Peak actual elevator               : %.4f deg\n',peakActualElevator_deg);
fprintf('Peak actuator rate                 : %.4f deg/s\n',peakActuatorRate_degps);
fprintf('Peak speed perturbation            : %.2f %% Vtrim\n', ...
    100*peakSpeedFraction);
fprintf('Peak alpha perturbation            : %.4f deg\n',peakAlpha_deg);
fprintf('Final attitude error               : %.5f deg\n', ...
    finalAttitudeError_deg);
fprintf('Position/rate limiting active      : %d / %d\n\n', ...
    positionLimitActiveNormal,rateLimitActiveNormal);

fprintf('Stress initial attitude            : %.1f deg (logic test only)\n', ...
    actuator.stress.initialAttitude_deg);
fprintf('Peak integrator with anti-windup   : %.3f deg s\n', ...
    stressAw.peakIntegrator_deg_s);
fprintf('Peak integrator without protection : %.3f deg s\n', ...
    stressNoAw.peakIntegrator_deg_s);
fprintf('Saturation with anti-windup        : %.3f s\n', ...
    stressAw.positionSaturationDuration_s);
fprintf('Saturation without protection      : %.3f s\n', ...
    stressNoAw.positionSaturationDuration_s);
fprintf('Protected final attitude           : %.5f deg\n\n', ...
    stressAw.finalAttitude_deg);

fprintf('PASS: Simulink matches independent RK4 references in all three runs.\n');
fprintf('PASS: normal V2 response satisfies every nominal and envelope gate.\n');
fprintf('PASS: position and rate limits remain inactive for the 1 deg command.\n');
fprintf('PASS: anti-windup materially limits integrator growth and saturation.\n');
fprintf('PASS: actuator and controller feedback directions are correct.\n');

%% Figures

figure('Name','C172S V2 Simulink Equivalence','Color','w');
tiledlayout(2,2);
labels = {'u (m/s)','alpha (deg)','q (deg/s)','theta (deg)'};
yPlot = yNormal(:,1:4);
yReferencePlot = yNormalReference(:,1:4);
yPlot(:,2:4) = rad2deg(yPlot(:,2:4));
yReferencePlot(:,2:4) = rad2deg(yReferencePlot(:,2:4));
for k = 1:4
    nexttile;
    plot(tNormal,yPlot(:,k),'LineWidth',1.3,'DisplayName','Simulink');
    hold on;
    plot(tNormal,yReferencePlot(:,k),'--','LineWidth',1.0, ...
        'DisplayName','Independent RK4');
    grid on;
    xlabel('Time (s)');
    ylabel(labels{k});
    if k == 1
        legend('Location','best');
    end
end
sgtitle('Pitch-Attitude Hold V2: Plant-State Equivalence');

figure('Name','C172S V2 Controller and Actuator Signals','Color','w');
tiledlayout(4,1);
nexttile;
plot(tNormal,rad2deg(thetaCommand)*ones(size(tNormal)),'k--', ...
    'DisplayName','Command');
hold on;
plot(tNormal,rad2deg(thetaFiltered),'DisplayName','Filtered command');
plot(tNormal,rad2deg(theta),'LineWidth',1.4,'DisplayName','Attitude');
grid on;
ylabel('\theta (deg)');
legend('Location','best');
title('Pitch-Attitude Tracking');
nexttile;
plot(tNormal,rad2deg(deltaEUnsat),'DisplayName','Unsaturated demand');
hold on;
plot(tNormal,rad2deg(deltaEPositionCommand),'--', ...
    'DisplayName','Position command');
plot(tNormal,rad2deg(deltaEActual),'LineWidth',1.4, ...
    'DisplayName','Actual elevator');
grid on;
ylabel('\delta_e (deg)');
legend('Location','best');
title('Elevator Position');
nexttile;
plot(tNormal,rad2deg(deltaERate),'LineWidth',1.4);
hold on;
yline(actuator.dynamics.rateLimit_degps,'k--','Rate limit');
yline(-actuator.dynamics.rateLimit_degps,'k--');
grid on;
ylabel('deg/s');
title('Actuator Rate');
nexttile;
plot(tNormal,rad2deg(antiWindupCorrection),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('deg');
title('Anti-Windup Correction');

figure('Name','C172S V2 Anti-Windup Stress','Color','w');
tiledlayout(3,1);
nexttile;
plot(tStressAw,rad2deg(yStressAw(:,4)),'LineWidth',1.4, ...
    'DisplayName','Anti-windup');
hold on;
plot(tStressNoAw,rad2deg(yStressNoAw(:,4)),'--','LineWidth',1.2, ...
    'DisplayName','No anti-windup');
grid on;
ylabel('\theta (deg)');
title('Controller-Logic Stress Response');
legend('Location','best');
nexttile;
plot(tStressAw,rad2deg(yStressAw(:,6)),'LineWidth',1.4, ...
    'DisplayName','Anti-windup');
hold on;
plot(tStressNoAw,rad2deg(yStressNoAw(:,6)),'--','LineWidth',1.2, ...
    'DisplayName','No anti-windup');
grid on;
ylabel('\xi (deg s)');
title('Integrator Growth');
legend('Location','best');
nexttile;
plot(tStressAw,rad2deg(yStressAw(:,9)),'DisplayName','Unsaturated demand');
hold on;
plot(tStressAw,rad2deg(yStressAw(:,11)),'LineWidth',1.4, ...
    'DisplayName','Actual elevator');
yline(actuator.authority.trailingEdgeDown_deg,'k--','Authority');
yline(-actuator.authority.trailingEdgeUp_deg,'k--');
grid on;
xlabel('Time (s)');
ylabel('\delta_e (deg)');
title('Protected Elevator');
legend('Location','best');

%% Refresh the saved model after all simulations and plotting

modelFile = get_param(modelName,'FileName');
close_system(modelName,0);
load_system(modelFile);
open_system(modelName);
fprintf('Reloaded generated model to clear transient editor diagnostics.\n');

%% Local helpers

function [t,y] = extractArrayOutput(simOut,nSignal)
t = simOut.tout;
y = simOut.yout;
if isa(y,'timeseries')
    y = y.Data;
elseif isa(y,'Simulink.SimulationData.Dataset')
    y = y.getElement(1).Values.Data;
end
y = squeeze(y);
if size(y,1) ~= numel(t) && size(y,2) == numel(t)
    y = y.';
end
assert(size(y,2) == nSignal, ...
    'Expected %d logged signals but received %d.',nSignal,size(y,2));
end

function y = rk4Reference(t,z0,command,antiWindupEnabled, ...
    plant,controller,actuator)
dt = median(diff(t));
assert(max(abs(diff(t) - dt)) < 1.0e-10, ...
    'Reference integration requires uniform sample time.');
z = z0;
y = zeros(numel(t),13);
for k = 1:numel(t)
    [~,diagnostic] = protectedDerivative(z,command,antiWindupEnabled, ...
        plant,controller,actuator);
    y(k,:) = outputVector(z,diagnostic).';
    if k < numel(t)
        k1 = protectedDerivative(z,command,antiWindupEnabled, ...
            plant,controller,actuator);
        k2 = protectedDerivative(z + 0.5*dt*k1,command, ...
            antiWindupEnabled,plant,controller,actuator);
        k3 = protectedDerivative(z + 0.5*dt*k2,command, ...
            antiWindupEnabled,plant,controller,actuator);
        k4 = protectedDerivative(z + dt*k3,command, ...
            antiWindupEnabled,plant,controller,actuator);
        z = z + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end
end

function [zDot,d] = protectedDerivative(z,command,antiWindupEnabled, ...
    plant,controller,actuator)
Kp = controller.Kp_radPerRad;
Ki = controller.Ki_radPerRadPerS;
Kq = actuator.integration.Kq_s;
tauCommand = controller.commandFilterTimeConstant_s;
tauActuator = actuator.dynamics.timeConstant_s;
Tt = actuator.antiWindup.trackingTimeConstant_s;
rateLimit = deg2rad(actuator.dynamics.rateLimit_degps);

d.eTheta = z(6) - z(4);
d.deltaEPI = -Kp*d.eTheta - Ki*z(5);
d.qContribution = Kq*z(3);
d.deltaEUnsat = d.deltaEPI + d.qContribution;
d.deltaEPosition = min(max(d.deltaEUnsat, ...
    actuator.authority.lowerLimit_rad),actuator.authority.upperLimit_rad);
d.rawRate = (d.deltaEPosition - z(7))/tauActuator;
d.limitedRate = min(max(d.rawRate,-rateLimit),rateLimit);
d.deltaETrack = z(7) + tauActuator*d.limitedRate;
d.antiWindupCorrection = antiWindupEnabled* ...
    (d.deltaEUnsat - d.deltaETrack)/(Ki*Tt);

zDot = zeros(7,1);
zDot(1:4) = plant.A*z(1:4) + plant.B*z(7);
zDot(5) = d.eTheta + d.antiWindupCorrection;
zDot(6) = (command - z(6))/tauCommand;
zDot(7) = d.limitedRate;
end

function y = outputVector(z,d)
y = [z(1:4);z(6);z(5);d.deltaEPI;d.qContribution; ...
    d.deltaEUnsat;d.deltaEPosition;z(7);d.limitedRate; ...
    d.antiWindupCorrection];
end

function metrics = stressMetrics(t,y,actuator,tauActuator)
dt = median(diff(t));
positionActive = abs(y(:,9) - y(:,10)) > 1.0e-10;
rawRate = (y(:,10) - y(:,11))/tauActuator;
rateActive = abs(rawRate - y(:,12)) > 1.0e-8;
metrics.positionLimitActive = any(positionActive);
metrics.rateLimitActive = any(rateActive);
metrics.positionSaturationDuration_s = dt*nnz(positionActive);
metrics.rateSaturationDuration_s = dt*nnz(rateActive);
metrics.peakIntegrator_deg_s = max(abs(rad2deg(y(:,6))));
metrics.finalAttitude_deg = rad2deg(y(end,4));
recoveryIndex = find(abs(rad2deg(y(:,4))) <= ...
    actuator.stress.recoveryThreshold_deg,1,'first');
if isempty(recoveryIndex)
    metrics.recoveryTime_s = inf;
else
    metrics.recoveryTime_s = t(recoveryIndex);
end
end

function [Aclosed,Bcommand] = actuatorClosedLoopMatrices( ...
    plant,Kp,Ki,Kq,tauCommand,tauActuator)
Aclosed = zeros(7,7);
Aclosed(1:4,1:4) = plant.A;
Aclosed(1:4,7) = plant.B;
Aclosed(5,4) = -1;
Aclosed(5,6) = 1;
Aclosed(6,6) = -1/tauCommand;
Aclosed(7,3) = Kq/tauActuator;
Aclosed(7,4) = Kp/tauActuator;
Aclosed(7,5) = -Ki/tauActuator;
Aclosed(7,6) = -Kp/tauActuator;
Aclosed(7,7) = -1/tauActuator;
Bcommand = zeros(7,1);
Bcommand(6) = 1/tauCommand;
end

function [Aouter,Bouter,Couter] = actuatorOuterPlantMatrices( ...
    plant,Kq,tauActuator)
Aouter = zeros(5,5);
Aouter(1:4,1:4) = plant.A;
Aouter(1:4,5) = plant.B;
Aouter(5,3) = Kq/tauActuator;
Aouter(5,5) = -1/tauActuator;
Bouter = zeros(5,1);
Bouter(5) = 1/tauActuator;
Couter = [0,0,0,-1,0];
end

function fastPole = selectFastPole(positivePoles)
[~,index] = max(abs(imag(positivePoles)));
fastPole = positivePoles(index);
end

function frequency = slowestNonPhugoidFrequency(poles)
% Actuator/short-period dynamics may be real rather than oscillatory.
% Remove the two lowest-frequency poles (the phugoid pair) and use the
% slowest remaining pole for a conservative separation calculation.
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
    settlingTime = 0;
elseif outside(end) == numel(t)
    settlingTime = inf;
else
    settlingTime = t(outside(end) + 1);
end
overshoot = max(0,100*(max(response) - command)/abs(command));
end
