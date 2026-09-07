%% C172S Lateral-Directional Plant V0.1 - Simulink equivalence check

clc;
clear;
close all;

try
    load_system('simulink');
catch ME
    error('runLateralDirectionalSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

data = c172sLateralDirectionalData();
[~,plant] = c172sLateralDirectionalPlant(data);

assignin('base','ldpData',data);
assignin('base','ldpPlant',plant);
assignin('base','A_ldp',plant.A);
assignin('base','B_ldp',plant.B);
assignin('base','C_ldp',plant.C);
assignin('base','D_ldp',plant.D);
assignin('base','x0_ldp',zeros(5,1));

% Rebuild once from a clean diagram so no interrupted builder run or stale
% model file can affect the equivalence test.
modelName = buildC172sLateralDirectionalSimulink(true);

aileronCapture = runCapture(modelName,plant,deg2rad(1.0),0.0);
rudderCapture = runCapture(modelName,plant,0.0,deg2rad(1.0));

tolerance = 1.0e-5;
assert(aileronCapture.maxError < tolerance, ...
    'Aileron capture error %.6g exceeds %.6g.', ...
    aileronCapture.maxError,tolerance);
assert(rudderCapture.maxError < tolerance, ...
    'Rudder capture error %.6g exceeds %.6g.', ...
    rudderCapture.maxError,tolerance);

% Primary command-direction gates.
assert(max(aileronCapture.ySim(:,4)) > 0, ...
    'Aileron sign failed: positive command must produce phi > 0.');
assert(max(rudderCapture.ySim(:,3)) > 0, ...
    'Rudder sign failed: positive command must produce r > 0.');

assert(strcmp(get_param(modelName,'SolverType'),'Fixed-step'), ...
    'The generated model must use a fixed-step solver.');
assert(strcmp(get_param(modelName,'Solver'),'ode4'), ...
    'The generated model must use ode4.');
assert(abs(str2double(get_param(modelName,'FixedStep')) - 0.01) < eps, ...
    'The generated model fixed step must be 0.01 s.');

lambda = eig(plant.A);
dynamicLambda = lambda(abs(lambda) >= 1.0e-10);
complexModes = dynamicLambda(abs(imag(dynamicLambda)) >= 1.0e-8);
realModes = real(dynamicLambda(abs(imag(dynamicLambda)) < 1.0e-8));
[~,rollIndex] = min(realModes);
rollPole = realModes(rollIndex);
realModes(rollIndex) = [];
spiralPole = realModes(1);
dutchPole = complexModes(find(imag(complexModes) > 0,1,'first'));

fprintf('============================================================\n');
fprintf(' C172S LATERAL-DIRECTIONAL V0.1 - SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                               : %s\n',modelName);
fprintf('State order                         : [beta p r phi psi]\n');
fprintf('Input order                         : [deltaA deltaR]\n');
fprintf('Solver / fixed step                 : ode4 / 0.0100 s\n');
fprintf('Aileron pulse                       : +1.0 deg, 0-1 s\n');
fprintf('Rudder pulse                        : +1.0 deg, 5-6 s\n\n');
fprintf('Aileron maximum signal error        : %.9g\n', ...
    aileronCapture.maxError);
fprintf('Aileron RMS signal error            : %.9g\n', ...
    aileronCapture.rmsError);
fprintf('Rudder maximum signal error         : %.9g\n', ...
    rudderCapture.maxError);
fprintf('Rudder RMS signal error             : %.9g\n', ...
    rudderCapture.rmsError);
fprintf('Allowed error                       : %.3g\n\n',tolerance);
fprintf('Roll-subsidence pole                : %.6f 1/s\n',rollPole);
fprintf('Spiral pole                         : %.8f 1/s\n',spiralPole);
fprintf('Dutch-roll poles                    : %.6f +/- %.6fi 1/s\n\n', ...
    real(dutchPole),abs(imag(dutchPole)));
fprintf('PASS: Simulink matches independent fixed-step RK4 references.\n');
fprintf('PASS: aileron and rudder command directions are correct.\n');
fprintf('PASS: state/input ordering and fixed-step solver are correct.\n');
fprintf(['NOTE: Plant V0.1 remains a provisional research baseline; ', ...
    'this check validates implementation equivalence, not source fidelity.\n']);

%% Equivalence plots

figure('Name','C172S Lateral-Directional Simulink Equivalence','Color','w');
tiledlayout(2,2);

nexttile;
plotComparison(aileronCapture.t,aileronCapture.ySim(:,4), ...
    aileronCapture.yReference(:,4),'Bank angle (deg)');
title('Aileron Capture: Bank Angle');

nexttile;
plotComparison(aileronCapture.t,aileronCapture.ySim(:,3), ...
    aileronCapture.yReference(:,3),'Yaw rate (deg/s)');
title('Aileron Capture: Yaw Rate');

nexttile;
plotComparison(rudderCapture.t,rudderCapture.ySim(:,1), ...
    rudderCapture.yReference(:,1),'Sideslip (deg)');
title('Rudder Capture: Sideslip');

nexttile;
plotComparison(rudderCapture.t,rudderCapture.ySim(:,3), ...
    rudderCapture.yReference(:,3),'Yaw rate (deg/s)');
title('Rudder Capture: Yaw Rate');

sgtitle('Lateral-Directional Plant V0.1: Simulink vs Independent RK4');

function result = runCapture(modelName,plant,deltaA_rad,deltaR_rad)
assignin('base','deltaACommand_ldp_rad',deltaA_rad);
assignin('base','deltaRCommand_ldp_rad',deltaR_rad);
assignin('base','x0_ldp',zeros(5,1));

simOut = sim(modelName,'StopTime','20','ReturnWorkspaceOutputs','on');
tSim = simOut.tout;
ySim = extractArrayOutput(simOut.yout,tSim);

if size(ySim,2) ~= 7
    error('runLateralDirectionalSimulinkCheck:UnexpectedOutputWidth', ...
        'Expected seven output columns; received %d.',size(ySim,2));
end

dt = diff(tSim);
nominalDt = median(dt);
assert(max(abs(dt - nominalDt)) < 1.0e-10, ...
    'The RK4 reference requires a uniformly sampled time vector.');

nState = size(plant.A,1);
x = zeros(nState,1);
xReference = zeros(numel(tSim),nState);
uReference = zeros(numel(tSim),2);
for k = 1:numel(tSim)
    uReference(k,:) = pulseInput(tSim(k),deltaA_rad,deltaR_rad).';
end

% Simulink's fixed-step ode4 evaluates a discontinuous Step input at all
% four RK4 stages. In the interval ending exactly at a pulse edge, k4 sees
% the post-step value. An exact-ZOH reference would instead hold the old
% input through that complete interval and create a false boundary error.
stateDerivative = @(state,input) plant.A*state + plant.B*input;
for k = 1:numel(tSim)-1
    t0 = tSim(k);
    h = tSim(k + 1) - t0;
    u1 = pulseInput(t0,deltaA_rad,deltaR_rad);
    u2 = pulseInput(t0 + 0.5*h,deltaA_rad,deltaR_rad);
    u4 = pulseInput(t0 + h,deltaA_rad,deltaR_rad);

    k1 = stateDerivative(x,u1);
    k2 = stateDerivative(x + 0.5*h*k1,u2);
    k3 = stateDerivative(x + 0.5*h*k2,u2);
    k4 = stateDerivative(x + h*k3,u4);
    x = x + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
    xReference(k + 1,:) = x.';
end

yReference = [xReference,uReference];
errorSignal = ySim - yReference;

result.t = tSim;
result.ySim = ySim;
result.yReference = yReference;
result.maxError = max(abs(errorSignal(:)));
result.rmsError = sqrt(mean(errorSignal(:).^2));
end

function input = pulseInput(t,deltaA_rad,deltaR_rad)
% Snap roundoff-level values to the commanded Step times so the independent
% reference uses the same before/after convention at each pulse boundary.
edges = [0.0 1.0 5.0 6.0];
for k = 1:numel(edges)
    if abs(t - edges(k)) <= 1.0e-12
        t = edges(k);
        break;
    end
end

input = [0.0;0.0];
if t >= 0.0 && t < 1.0
    input(1) = deltaA_rad;
end
if t >= 5.0 && t < 6.0
    input(2) = deltaR_rad;
end
end

function y = extractArrayOutput(yOut,t)
if isa(yOut,'timeseries')
    y = yOut.Data;
elseif isa(yOut,'Simulink.SimulationData.Dataset')
    y = yOut.getElement(1).Values.Data;
else
    y = yOut;
end

y = squeeze(y);
if size(y,1) ~= numel(t) && size(y,2) == numel(t)
    y = y.';
end
end

function plotComparison(t,ySim,yReference,yLabel)
plot(t,rad2deg(ySim),'LineWidth',1.3);
hold on;
plot(t,rad2deg(yReference),'--','LineWidth',1.0);
grid on;
xlabel('Time (s)');
ylabel(yLabel);
legend('Simulink','Independent RK4','Location','best');
end