function session = mainC172sFlightControlLab()
%MAINC172SFLIGHTCONTROLLAB Run the interactive C172S flight-control lab.
%
%   SESSION = MAINC172SFLIGHTCONTROLLAB opens the public command-line entry
%   point for nonlinear-autopilot demonstrations, custom missions,
%   autopilot comparisons, selectable execution and regression access.

clc;
model = c172sNonlinearAutopilotClosedLoopData();
lab = c172sFlightControlLabData(model);
session.startedUTC = datetime('now','TimeZone','UTC');
session.labArtifactRevision = lab.artifactRevision;
session.runs = cell(0,1);
session.comparisons = cell(0,1);

printWelcome(lab);
keepRunning = true;
while keepRunning
    printMenu();
    choice = promptInteger('Select an option',1,9,1);
    switch choice
        case 1
            scenario = createC172sFlightControlScenario( ...
                'quick-demo',struct(),model);
            flightRun = executeInteractive(scenario,model);
            session.runs{end+1,1} = flightRun; %#ok<AGROW>
        case 2
            options = customMissionInputs(lab);
            scenario = createC172sFlightControlScenario( ...
                'custom',options,model);
            flightRun = executeInteractive(scenario,model);
            session.runs{end+1,1} = flightRun; %#ok<AGROW>
        case 3
            backend = chooseExecutionBackend();
            [onRun,offRun,comparison] = runComparison(model,backend);
            session.runs{end+1,1} = onRun; %#ok<AGROW>
            session.runs{end+1,1} = offRun; %#ok<AGROW>
            session.comparisons{end+1,1} = comparison; %#ok<AGROW>
        case 4
            key = choosePreset(lab);
            if ~isempty(key)
                scenario = createC172sFlightControlScenario( ...
                    key,struct(),model);
                flightRun = executeInteractive(scenario,model);
                session.runs{end+1,1} = flightRun; %#ok<AGROW>
            end
        case 5
            openExecutableModel(lab);
        case 6
            showSystemInformation(lab,model);
        case 7
            if promptYesNo(['Run the complete nonlinear-autopilot ', ...
                    'regression now? This can take several minutes'],false)
                regression = feval(lab.execution.regressionRunner);
                session.regression = regression;
            end
        case 8
            if promptYesNo(['Run the V1.1 MATLAB and public-backend ', ...
                    'qualification now? This can take several minutes'],false)
                session.labAudit = auditC172sFlightControlLab();
                session.backendAudit = auditC172sFlightControlLabBackends();
            end
        case 9
            keepRunning = false;
    end
end

session.finishedUTC = datetime('now','TimeZone','UTC');
fprintf('\nLaboratory session complete. Runs retained in returned SESSION: %d\n', ...
    numel(session.runs));
end

function printWelcome(lab)
fprintf('============================================================\n');
fprintf('       C172S FLIGHT DYNAMICS AND AUTOPILOT LABORATORY\n');
fprintf('============================================================\n');
fprintf('Version                              : %s\n',lab.version);
fprintf('Laboratory artifact                  : %s\n', ...
    lab.artifactRevision);
fprintf('Controller / nonlinear plant         : %s / %s\n', ...
    lab.required.controllerRelease,lab.required.nonlinearPlantRelease);
fprintf('Nonlinear-autopilot release          : %s\n', ...
    lab.required.nonlinearAutopilotRelease);
fprintf('Frozen regression evidence           : %s\n', ...
    lab.required.externalRegressionEvidence);
fprintf('Trim                                 : %.0f ft / %.0f KTAS / %.0f lb\n', ...
    lab.trim.altitude_ft,lab.trim.TAS_KTAS,lab.trim.weight_lb);
fprintf(['\nThis laboratory operates the frozen near-trim research system. ', ...
    'It does not modify any released controller or plant.\n\n']);
end

function printMenu()
fprintf('------------------------------------------------------------\n');
fprintf(' 1  Quick heading and altitude demonstration\n');
fprintf(' 2  Build and run a custom nonlinear mission\n');
fprintf(' 3  Compare autopilot ON versus OFF\n');
fprintf(' 4  Run a disturbance, fallback or disconnect preset\n');
fprintf(' 5  Open the executable nonlinear-autopilot Simulink model\n');
fprintf(' 6  View system scope, limits and frozen releases\n');
fprintf(' 7  Run the complete 28/28 regression gate\n');
fprintf(' 8  Run the V1.1 laboratory and backend qualification\n');
fprintf(' 9  Exit laboratory\n');
fprintf('------------------------------------------------------------\n');
end

function flightRun = executeInteractive(scenario,model)
backend = chooseExecutionBackend();
exportResult = promptYesNo('Export MAT, CSV and summary after the run',false);
runOptions.ShowPlots = true;
runOptions.PrintSummary = true;
runOptions.ExportResults = exportResult;
runOptions.ExportFolder = '';
runOptions.Backend = backend;
flightRun = runC172sFlightControlScenario(scenario,runOptions,model);
end

function backend = chooseExecutionBackend()
choice = promptInteger([ ...
    'Execution: 1=MATLAB, 2=SIMULINK, 3=COMPARE both'],1,3,1);
names = {'MATLAB','SIMULINK','COMPARE'};
backend = names{choice};
end

function options = customMissionInputs(lab)
fprintf('\nCUSTOM NONLINEAR-AUTOPILOT MISSION\n');
fprintf('All inputs are bounded inside the declared V0.1 lab boundary.\n');
options.duration_s = promptNumber('Duration (s)', ...
    lab.defaults.duration_s,lab.boundary.duration_s(1), ...
    lab.boundary.duration_s(2));
options.engage = promptYesNo('Engage the autopilot',true);
options.engageTime_s = 0;
if options.engage
    maximumEngage = min(lab.boundary.engageTime_s(2), ...
        options.duration_s-0.01);
    options.engageTime_s = promptNumber('Engagement time (s)', ...
        lab.defaults.engageTime_s,0,maximumEngage);
    lateralChoice = promptInteger( ...
        'Lateral mode: 1=ROLL capture, 2=HEADING select',1,2,2);
    if lateralChoice == 1
        options.lateralMode = 'ROLL';
        options.headingChange_deg = 0;
        fprintf('ROLL will capture the current bank at engagement.\n');
    else
        options.lateralMode = 'HEADING';
        options.headingChange_deg = promptNumber( ...
            'Selected heading change (deg, signed)', ...
            lab.defaults.headingChange_deg, ...
            lab.boundary.headingChange_deg(1), ...
            lab.boundary.headingChange_deg(2));
    end
    verticalChoice = promptInteger( ...
        'Vertical mode: 1=PITCH capture, 2=ALTITUDE select',1,2,2);
    if verticalChoice == 1
        options.verticalMode = 'PITCH';
        options.altitudeChange_ft = 0;
        fprintf('PITCH will capture the current pitch at engagement.\n');
    else
        options.verticalMode = 'ALTITUDE';
        options.altitudeChange_ft = promptNumber( ...
            'Selected altitude change (ft, signed)', ...
            lab.defaults.altitudeChange_ft, ...
            lab.boundary.altitudeChange_ft(1), ...
            lab.boundary.altitudeChange_ft(2));
    end
else
    options.lateralMode = 'OFF';
    options.verticalMode = 'OFF';
    options.headingChange_deg = 0;
    options.altitudeChange_ft = 0;
end

options.headwind_mps = promptNumber( ...
    'Steady headwind (m/s; negative means tailwind)',0, ...
    lab.boundary.headwind_mps(1),lab.boundary.headwind_mps(2));
options.crosswindFromRight_mps = promptNumber( ...
    'Crosswind from right (m/s; negative means from left)',0, ...
    lab.boundary.crosswindFromRight_mps(1), ...
    lab.boundary.crosswindFromRight_mps(2));
options.windStart_s = min(lab.defaults.windStart_s,options.duration_s);
if options.headwind_mps ~= 0 || options.crosswindFromRight_mps ~= 0
    options.windStart_s = promptNumber('Wind start time (s)', ...
        options.windStart_s,0,options.duration_s);
end

options.upwardGust_mps = promptNumber('Upward gust magnitude (m/s)',0, ...
    lab.boundary.upwardGust_mps(1),lab.boundary.upwardGust_mps(2));
options.gustDuration_s = lab.defaults.gustDuration_s;
options.gustStart_s = min(lab.defaults.gustStart_s, ...
    options.duration_s-options.gustDuration_s);
if options.upwardGust_mps > 0
    options.gustDuration_s = promptNumber('Gust duration (s)', ...
        lab.defaults.gustDuration_s, ...
        lab.boundary.gustDuration_s(1), ...
        min(lab.boundary.gustDuration_s(2),options.duration_s));
    options.gustStart_s = promptNumber('Gust start time (s)', ...
        min(lab.defaults.gustStart_s, ...
        options.duration_s-options.gustDuration_s),0, ...
        options.duration_s-options.gustDuration_s);
end

options.eventType = 'NONE';
options.eventTime_s = min(lab.defaults.eventTime_s,options.duration_s-0.01);
if options.engage && options.duration_s-options.engageTime_s >= 0.25
    eventNames = {'NONE','DISCONNECT','OVERRIDE'};
    eventLabels = {'none','pilot disconnect','pilot override'};
    if strcmp(options.lateralMode,'HEADING')
        eventNames{end+1} = 'HEADING-LOSS'; %#ok<AGROW>
        eventLabels{end+1} = 'heading-source loss'; %#ok<AGROW>
    end
    if strcmp(options.verticalMode,'ALTITUDE')
        eventNames{end+1} = 'ALTITUDE-LOSS'; %#ok<AGROW>
        eventLabels{end+1} = 'altitude-source loss'; %#ok<AGROW>
    end
    fprintf('Available events:\n');
    for k = 1:numel(eventNames)
        fprintf(' %d  %s\n',k,eventLabels{k});
    end
    eventChoice = promptInteger('Select an event',1,numel(eventNames),1);
    options.eventType = eventNames{eventChoice};
    if ~strcmp(options.eventType,'NONE')
        minimumEvent = options.engageTime_s+0.01;
        options.eventTime_s = promptNumber('Event time (s)', ...
            max(min(lab.defaults.eventTime_s,options.duration_s-0.01), ...
            minimumEvent),minimumEvent,options.duration_s-0.01);
    end
elseif options.engage
    fprintf(['No event can be scheduled because less than 0.25 s ', ...
        'remains after engagement.\n']);
end
end

function [onRun,offRun,comparison] = runComparison(model,backend)
onScenario = createC172sFlightControlScenario('quick-demo',struct(),model);
offOptions = onScenario.configuration;
offOptions.engage = false;
offOptions.lateralMode = 'OFF';
offOptions.verticalMode = 'OFF';
offScenario = createC172sFlightControlScenario( ...
    'autopilot-off',offOptions,model);
silent.ShowPlots = false;
silent.PrintSummary = false;
silent.ExportResults = false;
silent.ExportFolder = '';
silent.Backend = backend;
fprintf('\nRunning autopilot-ON case ...\n');
onRun = runC172sFlightControlScenario(onScenario,silent,model);
fprintf('Running autopilot-OFF case ...\n');
offRun = runC172sFlightControlScenario(offScenario,silent,model);
comparison = compareC172sFlightControlRuns(onRun,offRun,true);
end

function key = choosePreset(lab)
fprintf('\nPRESET LIBRARY\n');
for k = 2:numel(lab.presets.keys)-1
    fprintf(' %d  %s\n',k-1,lab.presets.names{k});
end
fprintf(' 0  Return to main menu\n');
choice = promptInteger('Select a preset',0, ...
    numel(lab.presets.keys)-2,0);
if choice == 0
    key = '';
else
    key = lab.presets.keys{choice+1};
end
end

function openExecutableModel(lab)
assert(exist(lab.execution.simulinkBuilder,'file') == 2, ...
    'mainC172sFlightControlLab:MissingSimulinkBuilder', ...
    'The executable Simulink builder is not on the MATLAB path.');
modelName = feval(lab.execution.simulinkBuilder,false);
load_system(modelName);
open_system(modelName);
fprintf('Opened executable model: %s\n',modelName);
end

function showSystemInformation(lab,model)
fprintf('\n============================================================\n');
fprintf(' C172S FLIGHT-CONTROL LAB - SYSTEM INFORMATION\n');
fprintf('============================================================\n');
fprintf('Laboratory artifact                 : %s\n',lab.artifactRevision);
fprintf('Closed-loop backend                 : %s\n', ...
    model.artifactRevision);
fprintf('Controller release                  : %s\n', ...
    lab.required.controllerRelease);
fprintf('Nonlinear-plant release             : %s\n', ...
    lab.required.nonlinearPlantRelease);
fprintf('Nonlinear-autopilot release         : %s\n', ...
    lab.required.nonlinearAutopilotRelease);
fprintf('Regression evidence                 : %s\n', ...
    lab.required.externalRegressionEvidence);
fprintf('State count                         : %d plant + %d controller\n', ...
    model.state.plantCount,model.state.controllerCount);
fprintf('Fixed step                          : %.3f s\n', ...
    lab.execution.fixedStep_s);
fprintf('Execution backends                  : %s\n', ...
    strjoin(lab.execution.backends,', '));
fprintf('MATLAB/Simulink comparison gate     : %.1e over 58 signals\n', ...
    lab.execution.maximumBackendError);
fprintf('Custom heading change               : %+.0f to %+.0f deg\n', ...
    lab.boundary.headingChange_deg);
fprintf('Custom altitude change              : %+.0f to %+.0f ft\n', ...
    lab.boundary.altitudeChange_ft);
fprintf('Inner-mode command ownership        : capture current bank/pitch\n');
fprintf('Selectable references               : HEADING and ALTITUDE only\n');
fprintf('Validated TAS monitor               : %.0f to %.0f KTAS\n', ...
    lab.boundary.V_KTAS);
fprintf('Validated alpha monitor             : %.0f to %.0f deg\n', ...
    lab.boundary.alpha_deg);
fprintf('Validated beta monitor              : %.0f to %.0f deg\n', ...
    lab.boundary.beta_deg);
fprintf('\n%s\n\n',lab.limitations);
end

function value = promptNumber(prompt,default,minimum,maximum)
while true
    answer = strtrim(input(sprintf('%s [%g]: ',prompt,default),'s'));
    if isempty(answer)
        value = default;
    else
        value = str2double(answer);
    end
    if isfinite(value) && value >= minimum && value <= maximum
        return;
    end
    fprintf('Enter a number from %g to %g.\n',minimum,maximum);
end
end

function value = promptInteger(prompt,minimum,maximum,default)
while true
    value = promptNumber(prompt,default,minimum,maximum);
    if value == round(value)
        value = round(value);
        return;
    end
    fprintf('Enter a whole-number menu option.\n');
end
end

function yes = promptYesNo(prompt,default)
if default
    suffix = 'Y/n';
else
    suffix = 'y/N';
end
while true
    answer = lower(strtrim(input(sprintf('%s [%s]: ',prompt,suffix),'s')));
    if isempty(answer)
        yes = default;
        return;
    elseif any(strcmp(answer,{'y','yes'}))
        yes = true;
        return;
    elseif any(strcmp(answer,{'n','no'}))
        yes = false;
        return;
    end
    fprintf('Enter y or n.\n');
end
end
