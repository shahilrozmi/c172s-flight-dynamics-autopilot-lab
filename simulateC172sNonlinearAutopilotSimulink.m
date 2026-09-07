function result = simulateC172sNonlinearAutopilotSimulink(scenario,model)
%SIMULATEC172SNONLINEARAUTOPILOTSIMULINK Run the executable lab backend.
%
%   RESULT = SIMULATEC172SNONLINEARAUTOPILOTSIMULINK(SCENARIO,MODEL) runs
%   C172S_NonlinearAutopilot_V1 through SimulationInput and reconstructs the
%   same public result structure returned by the MATLAB reference backend.

if nargin < 2 || isempty(model)
    model = c172sNonlinearAutopilotClosedLoopData();
end
lab = c172sFlightControlLabData(model);
assert(~isempty(which('sim')), ...
    'simulateC172sNonlinearAutopilotSimulink:SimulinkUnavailable', ...
    'Simulink is required for the SIMULINK and COMPARE backends.');
assert(isstruct(scenario) && isfield(scenario,'labArtifactRevision') && ...
    strcmp(scenario.labArtifactRevision,lab.artifactRevision), ...
    'simulateC172sNonlinearAutopilotSimulink:InvalidScenario', ...
    'Use createC172sFlightControlScenario to create the scenario.');

t = scenario.time_s(:);
n = numel(t);
assert(n >= 2 && size(scenario.request,1) == n && ...
    size(scenario.request,2) == model.input.requestCount && ...
    isequal(size(scenario.steadyWindNED_mps),[n,3]) && ...
    isequal(size(scenario.gustNED_mps),[n,3]), ...
    'simulateC172sNonlinearAutopilotSimulink:InvalidScenario', ...
    'Scenario arrays do not satisfy the frozen input interface.');
assert(max(abs(diff(t)-model.execution.fixedStep_s)) <= 1.0e-12, ...
    'simulateC172sNonlinearAutopilotSimulink:InvalidTimeGrid', ...
    'Scenario time must use the frozen %.3f-s sample interval.', ...
    model.execution.fixedStep_s);

modelName = lab.execution.simulinkModel;
modelAvailable = bdIsLoaded(modelName) || isfile([modelName '.slx']) || ...
    ~isempty(which([modelName '.slx']));
if ~modelAvailable && exist(lab.execution.simulinkBuilder,'file') == 2
    modelName = feval(lab.execution.simulinkBuilder,false);
end
assert(isfile([modelName '.slx']) || bdIsLoaded(modelName) || ...
    ~isempty(which([modelName '.slx'])), ...
    'simulateC172sNonlinearAutopilotSimulink:MissingModel', ...
    'Executable model %s.slx is unavailable.',modelName);

wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelName);
end
cleanup = onCleanup(@() closeIfOwned(modelName,wasLoaded)); %#ok<NASGU>

inputSignal = [t scenario.request scenario.steadyWindNED_mps ...
    scenario.gustNED_mps];

% The released model's InitFcn validates this From Workspace input before
% SimulationInput variables are installed in some MATLAB/Simulink releases
% (including R2026a). Seed the base workspace for that callback only, then
% restore the caller's workspace exactly when this function returns. The
% SimulationInput copy below remains the authoritative simulation input.
baseVariable = 'na1InputSignal';
baseVariableWasDefined = evalin('base', ...
    sprintf('exist(''%s'',''var'') == 1',baseVariable));
if baseVariableWasDefined
    previousBaseValue = evalin('base',baseVariable);
else
    previousBaseValue = [];
end
assignin('base',baseVariable,inputSignal);
baseCleanup = onCleanup(@() restoreBaseVariable( ...
    baseVariable,baseVariableWasDefined,previousBaseValue)); %#ok<NASGU>

simInput = Simulink.SimulationInput(modelName);
simInput = simInput.setVariable('na1InputSignal',inputSignal);
simInput = simInput.setModelParameter( ...
    'StopTime',num2str(t(end),17),'ReturnWorkspaceOutputs','on');
simOut = sim(simInput);
[simTime,matrix] = extractArrayOutput(simOut,58);
assert(numel(simTime) == n && max(abs(simTime-t)) <= 1.0e-10, ...
    'simulateC172sNonlinearAutopilotSimulink:SampleGridChanged', ...
    'Executable Simulink sample grid differs from the scenario grid.');

result = unpackResult(simTime,matrix,scenario,model);
end

function result = unpackResult(time,matrix,scenario,model)
result.time_s = time;
result.plantState = matrix(:,1:13);
result.controllerState = matrix(:,14:26);
result.control = matrix(:,27:30);
result.surfaceRate_radps = matrix(:,31:33);
result.euler_rad = matrix(:,34:36);
result.altitude_ft = matrix(:,37);
result.altitudePerturbation_m = matrix(:,38);
result.verticalSpeed_mps = matrix(:,39);
result.TAS_mps = matrix(:,40);
result.alpha_rad = matrix(:,41);
result.beta_rad = matrix(:,42);
result.quaternionNorm = matrix(:,43);
result.withinEnvelope = logical(round(matrix(:,44)));
result.engaged = logical(round(matrix(:,45)));
result.lateralMode = uint8(round(matrix(:,46)));
result.verticalMode = uint8(round(matrix(:,47)));
result.rollTrackRequest = logical(round(matrix(:,48)));
result.pitchTrackRequest = logical(round(matrix(:,49)));
result.eventCode = uint8(round(matrix(:,50)));
result.bankCommand_deg = matrix(:,51);
result.pitchCommand_deg = matrix(:,52);
result.headingCommand_deg = matrix(:,53);
result.altitudeCommand_ft = matrix(:,54);
result.aileronSaturated = logical(round(matrix(:,55)));
result.elevatorSaturated = logical(round(matrix(:,56)));
result.aileronSaturationTime_s = matrix(:,57);
result.elevatorSaturationTime_s = matrix(:,58);
result.request = scenario.request;
result.steadyWindNED_mps = scenario.steadyWindNED_mps;
result.gustNED_mps = scenario.gustNED_mps;
result.finalManagerState = struct( ...
    'engaged',result.engaged(end), ...
    'lateralMode',result.lateralMode(end), ...
    'verticalMode',result.verticalMode(end), ...
    'bankCommand_deg',result.bankCommand_deg(end), ...
    'pitchCommand_deg',result.pitchCommand_deg(end), ...
    'headingCommand_deg',result.headingCommand_deg(end), ...
    'altitudeCommand_ft',result.altitudeCommand_ft(end), ...
    'aileronSaturationTime_s',result.aileronSaturationTime_s(end), ...
    'elevatorSaturationTime_s',result.elevatorSaturationTime_s(end));
result.modelArtifactRevision = model.artifactRevision;
result.scenarioName = scenario.name;
result.executionBackend = 'SIMULINK';
end

function [time,output] = extractArrayOutput(simOut,nSignal)
time = simOut.tout(:);
output = simOut.yout;
if isa(output,'timeseries')
    time = output.Time(:);
    output = output.Data;
end
if isstruct(output) && isfield(output,'signals')
    output = output.signals.values;
end
output = squeeze(output);
if size(output,1) ~= numel(time) && size(output,2) == numel(time)
    output = output.';
end
assert(size(output,1) == numel(time) && size(output,2) == nSignal, ...
    'simulateC172sNonlinearAutopilotSimulink:OutputInterface', ...
    'Expected %d output signals; received %s.', ...
    nSignal,mat2str(size(output)));
end

function closeIfOwned(modelName,wasLoaded)
if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName,0);
end
end

function restoreBaseVariable(name,wasDefined,value)
if wasDefined
    assignin('base',name,value);
else
    evalin('base',sprintf('clear(''%s'')',name));
end
end
