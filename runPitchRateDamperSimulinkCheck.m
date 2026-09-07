%% C172S Pitch-Rate Damper V1 - Simulink equivalence check

clc;
clear;
close all;

try
    load_system('simulink');
catch ME
    error('runPitchRateDamperSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

data = c172sLongitudinalData();
[~,plant] = c172sLongitudinalPlant(data);
damper = c172sPitchRateDamperData();

x0 = [0;0;deg2rad(5.0);0];
Aclosed = plant.A + plant.B*damper.qFeedbackRow;
closedPoles = eig(Aclosed);

assert(all(real(closedPoles) < 0), ...
    'The nominal q-damper closed-loop model is not stable.');

%% Initialize variables and rebuild the generated diagram

assignin('base','qdData',data);
assignin('base','qdPlant',plant);
assignin('base','qdDamper',damper);
assignin('base','A_qd',plant.A);
assignin('base','B_qd',plant.B);
assignin('base','C_qd',plant.C);
assignin('base','D_qd',plant.D);
assignin('base','Kq_pitchRate_s',damper.Kq_s);
assignin('base','qd_deltaECommand_rad',0.0);
assignin('base','x0_qd',x0);

modelName = buildC172sPitchRateDamperSimulink(true);

%% Run the Simulink feedback loop

simOut = sim(modelName, ...
    'StopTime','12', ...
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

if size(ySim,2) ~= 4
    error('Expected the Simulink State Vector output to have four columns.');
end

%% Independent exact closed-loop and open-loop references

dt = diff(tSim);
nominalDt = median(dt);
assert(max(abs(dt - nominalDt)) < 1.0e-10, ...
    'The exact reference requires a uniformly sampled time vector.');

AdClosed = expm(Aclosed*nominalDt);
AdOpen = expm(plant.A*nominalDt);

nSample = numel(tSim);
nState = size(plant.A,1);
yReference = zeros(nSample,nState);
yOpen = zeros(nSample,nState);
yReference(1,:) = x0.';
yOpen(1,:) = x0.';

xReference = x0;
xOpen = x0;
for k = 1:nSample-1
    xReference = AdClosed*xReference;
    xOpen = AdOpen*xOpen;
    yReference(k+1,:) = xReference.';
    yOpen(k+1,:) = xOpen.';
end

%% Numerical equivalence and controller-effectiveness gates

stateError = ySim - yReference;
maxAbsError = max(abs(stateError(:)));
rmsError = sqrt(mean(stateError(:).^2));

% With a smooth initial-condition response, fixed-step RK4 should be much
% closer than the earlier discontinuous-input plant check.
tolerance = 1.0e-4;
assert(maxAbsError < tolerance, ...
    ['Simulink/MATLAB q-damper mismatch: maximum absolute state ', ...
     'error %.6g exceeds %.6g.'],maxAbsError,tolerance);

deltaEFeedback = damper.Kq_s*ySim(:,3);
assert(deltaEFeedback(1) > 0, ...
    ['Feedback sign failed: positive q must command positive ', ...
     'trailing-edge-down elevator.']);

qIAEClosed_deg = rad2deg(trapz(tSim,abs(ySim(:,3))));
qIAEOpen_deg = rad2deg(trapz(tSim,abs(yOpen(:,3))));
assert(qIAEClosed_deg < qIAEOpen_deg, ...
    'The q damper did not reduce the integrated pitch-rate disturbance.');

peakFeedbackElevator_deg = max(abs(rad2deg(deltaEFeedback)));
assert(peakFeedbackElevator_deg < 1.0, ...
    'The nominal disturbance requires unexpectedly large elevator feedback.');

%% Modal identification

positivePoles = closedPoles(imag(closedPoles) > 1.0e-8);
assert(numel(positivePoles) == 2, ...
    'Expected two stable oscillatory mode pairs in the nominal model.');
[~,poleOrder] = sort(abs(imag(positivePoles)),'descend');
shortPeriodPole = positivePoles(poleOrder(1));
phugoidPole = positivePoles(poleOrder(2));

shortPeriodWn = abs(shortPeriodPole);
shortPeriodZeta = -real(shortPeriodPole)/shortPeriodWn;
phugoidPeriod_s = 2*pi/abs(imag(phugoidPole));
phugoidZeta = -real(phugoidPole)/abs(phugoidPole);

%% Console report

fprintf('============================================================\n');
fprintf('   C172S PITCH-RATE DAMPER V1 - SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                          : %s\n',modelName);
fprintf('Feedback law                   : deltaE = deltaECmd + Kq*q\n');
fprintf('Kq                             : %.4f s\n',damper.Kq_s);
fprintf('Initial pitch rate             : %.2f deg/s\n',rad2deg(x0(3)));
fprintf('Solver                         : Fixed-step ode4\n');
fprintf('Fixed step                     : %.3f s\n\n',nominalDt);

fprintf('Maximum absolute state error   : %.9g\n',maxAbsError);
fprintf('RMS state error                : %.9g\n',rmsError);
fprintf('Allowed error                  : %.3g\n\n',tolerance);

fprintf('Short-period wn                : %.4f rad/s\n',shortPeriodWn);
fprintf('Short-period damping           : %.4f\n',shortPeriodZeta);
fprintf('Phugoid period                 : %.3f s\n',phugoidPeriod_s);
fprintf('Phugoid damping                : %.4f\n',phugoidZeta);
fprintf('Open-loop integrated |q|       : %.4f deg\n',qIAEOpen_deg);
fprintf('Damped integrated |q|          : %.4f deg\n',qIAEClosed_deg);
fprintf('Peak damper elevator           : %.4f deg\n\n', ...
    peakFeedbackElevator_deg);

fprintf('PASS: Simulink matches the exact MATLAB q-damper model.\n');
fprintf('PASS: the feedback sign opposes a positive pitch rate.\n');
fprintf('PASS: the damper reduces the integrated pitch-rate response.\n');

%% Figure 1: complete state equivalence

figure('Name','C172S q-Damper Simulink Equivalence','Color','w');
tiledlayout(2,2);

stateLabels = {'u (m/s)','alpha (deg)','q (deg/s)','theta (deg)'};
plotDataSim = ySim;
plotDataReference = yReference;
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
        legend('Simulink','MATLAB exact','Location','best');
    end
end

sgtitle('Pitch-Rate Damper V1: Simulink vs MATLAB');

%% Figure 2: damping benefit and elevator activity

figure('Name','C172S q-Damper Effectiveness','Color','w');
tiledlayout(2,1);

nexttile;
plot(tSim,rad2deg(yOpen(:,3)),'LineWidth',1.2, ...
    'DisplayName','Open loop');
hold on;
plot(tSim,rad2deg(ySim(:,3)),'LineWidth',1.3, ...
    'DisplayName','Simulink q damper');
grid on;
ylabel('q (deg/s)');
title('Response to Initial q = 5 deg/s');
legend('Location','best');

nexttile;
plot(tSim,rad2deg(deltaEFeedback),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Elevator (deg)');
title('Damper Elevator Command, TE Down Positive');
