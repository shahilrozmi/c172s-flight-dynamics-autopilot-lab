%% C172S Simulink Plant V1 - numerical equivalence check

clc;
clear;
close all;

% Do not use exist('sim','file') as an installation check. Depending on the
% MATLAB release, SIM may be exposed through Simulink without appearing as
% an ordinary M-file. Loading the Simulink library is the reliable test.
try
    load_system('simulink');
catch ME
    error('runSimulinkPlantCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s', ME.message);
end

data = c172sLongitudinalData();
[sys, plant] = c172sLongitudinalPlant(data);

if isempty(sys)
    error('Control System Toolbox is required for the LSIM comparison.');
end

%% Initialize block variables and create/open the diagram

assignin('base','fcData',data);
assignin('base','fcPlant',plant);
assignin('base','A_long',plant.A);
assignin('base','B_long',plant.B);
assignin('base','C_long',plant.C);
assignin('base','D_long',plant.D);
assignin('base','x0_long',zeros(4,1));
assignin('base','deltaE_step_rad',-deg2rad(1.0));

% Rebuild once from a clean diagram so an earlier interrupted builder run
% cannot leave a partial or unsaved model in memory.
modelName = buildC172sLongitudinalSimulink(true);

%% Run Simulink

simOut = sim(modelName, ...
    'StopTime','40', ...
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

%% Generate an independent exact zero-order-hold MATLAB reference

deltaE = zeros(size(tSim));
deltaE(:) = -deg2rad(1.0);

% LSIM commonly uses linear interpolation between input samples. Across a
% discontinuous Step input that creates a half-sample ramp that Simulink's
% Step block does not contain. Build the exact ZOH state transition instead
% so both models see the same input over every fixed-step interval.
dt = diff(tSim);
nominalDt = median(dt);
assert(max(abs(dt - nominalDt)) < 1.0e-10, ...
    'The exact ZOH reference requires a uniformly sampled time vector.');

nState = size(plant.A,1);
augmentedMatrix = [plant.A, plant.B; zeros(1,nState + 1)];
transition = expm(augmentedMatrix*nominalDt);
Ad = transition(1:nState,1:nState);
Bd = transition(1:nState,end);

yReference = zeros(numel(tSim),nState);
xReference = zeros(nState,1);

for k = 1:numel(tSim)-1
    xReference = Ad*xReference + Bd*deltaE(k);
    yReference(k+1,:) = (plant.C*xReference ...
        + plant.D*deltaE(k+1)).';
end

maxAbsError = max(abs(ySim(:) - yReference(:)));
rmsError = sqrt(mean((ySim(:) - yReference(:)).^2));

% Fixed-step RK4 at 0.01 s should comfortably satisfy this tolerance.
tolerance = 1.0e-3;
assert(maxAbsError < tolerance, ...
    ['Simulink/MATLAB mismatch: maximum absolute state error %.6g ', ...
     'exceeds %.6g.'], maxAbsError, tolerance);

firstPostStep = find(tSim > 0.0,1,'first');
assert(ySim(firstPostStep,3) > 0, ...
    'Elevator sign failed: trailing-edge up must initially produce q > 0.');

fprintf('============================================================\n');
fprintf('       C172S SIMULINK PLANT V1 - EQUIVALENCE CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                  : %s\n', modelName);
fprintf('Solver                 : Fixed-step ode4\n');
fprintf('Fixed step             : 0.01 s\n');
fprintf('MATLAB reference       : Exact ZOH state transition\n');
fprintf('Simulation duration    : %.1f s\n', tSim(end));
fprintf('Maximum absolute error : %.9g\n', maxAbsError);
fprintf('RMS state error        : %.9g\n', rmsError);
fprintf('Allowed error          : %.3g\n\n', tolerance);
fprintf('PASS: Simulink matches the MATLAB state-space plant.\n');
fprintf('PASS: elevator trailing-edge-up produces the correct pitch sign.\n');

%% Overlay the Simulink and MATLAB reference responses

figure('Name','C172S Simulink Equivalence Check','Color','w');
tiledlayout(2,2);

stateLabels = {'u (m/s)','alpha (deg)','q (deg/s)','theta (deg)'};
plotDataSim = ySim;
plotDataRef = yReference;
plotDataSim(:,2:4) = rad2deg(plotDataSim(:,2:4));
plotDataRef(:,2:4) = rad2deg(plotDataRef(:,2:4));

for k = 1:4
    nexttile;
    plot(tSim,plotDataSim(:,k),'LineWidth',1.3);
    hold on;
    plot(tSim,plotDataRef(:,k),'--','LineWidth',1.0);
    grid on;
    xlabel('Time (s)');
    ylabel(stateLabels{k});
    if k == 1
        legend('Simulink','MATLAB exact ZOH','Location','best');
    end
end

sgtitle('C172S Plant V1: Simulink vs MATLAB');
