function flightRun = runC172sFlightControlScenario(scenario,options,model)
%RUNC172SFLIGHTCONTROLSCENARIO Execute, analyze, plot and export a scenario.
%
%   FLIGHTRUN = RUNC172SFLIGHTCONTROLSCENARIO(SCENARIO) runs the frozen
%   nonlinear-autopilot backend, prints a concise engineering summary and
%   opens the laboratory plots.
%
%   OPTIONS fields are ShowPlots, PrintSummary, ExportResults,
%   ExportFolder and Backend. Backend may be MATLAB, SIMULINK or COMPARE.
%   All fields are optional; MATLAB remains the default.

if nargin < 2 || isempty(options)
    options = struct();
end
if nargin < 3 || isempty(model)
    model = c172sNonlinearAutopilotClosedLoopData();
end
options = runOptions(options);
lab = c172sFlightControlLabData(model);
assert(isstruct(scenario) && isfield(scenario,'labArtifactRevision') && ...
    strcmp(scenario.labArtifactRevision,lab.artifactRevision), ...
    'runC172sFlightControlScenario:InvalidScenario', ...
    'Use createC172sFlightControlScenario to create the scenario.');

backendComparison = struct();
backendWallTime_s = struct('matlab',NaN,'simulink',NaN);
switch options.Backend
    case 'MATLAB'
        timer = tic;
        result = simulateC172sNonlinearAutopilot(scenario,model);
        backendWallTime_s.matlab = toc(timer);
    case 'SIMULINK'
        timer = tic;
        result = simulateC172sNonlinearAutopilotSimulink(scenario,model);
        backendWallTime_s.simulink = toc(timer);
    case 'COMPARE'
        timer = tic;
        result = simulateC172sNonlinearAutopilot(scenario,model);
        backendWallTime_s.matlab = toc(timer);
        timer = tic;
        simulinkResult = simulateC172sNonlinearAutopilotSimulink( ...
            scenario,model);
        backendWallTime_s.simulink = toc(timer);
        backendComparison = compareC172sNonlinearAutopilotBackends( ...
            result,simulinkResult,lab.execution.maximumBackendError);
        simulinkMetrics = analyzeC172sFlightControlScenario( ...
            simulinkResult,scenario,model);
        backendComparison.matlabOverallPass = false;
        backendComparison.simulinkOverallPass = ...
            simulinkMetrics.overallPass;
        backendComparison.simulinkHealthPass = simulinkMetrics.health.pass;
        backendComparison.simulinkObjectivePass = ...
            simulinkMetrics.objective.pass;
    otherwise
        error('runC172sFlightControlScenario:InvalidBackend', ...
            'Unsupported execution backend %s.',options.Backend);
end
elapsed = sum([backendWallTime_s.matlab,backendWallTime_s.simulink], ...
    'omitnan');
metrics = analyzeC172sFlightControlScenario(result,scenario,model);
if strcmp(options.Backend,'COMPARE')
    backendComparison.matlabOverallPass = metrics.overallPass;
end

flightRun.labArtifactRevision = lab.artifactRevision;
flightRun.backendArtifactRevision = model.artifactRevision;
flightRun.scenario = scenario;
flightRun.result = result;
flightRun.metrics = metrics;
flightRun.wallTime_s = elapsed;
flightRun.backendWallTime_s = backendWallTime_s;
flightRun.executionBackend = options.Backend;
flightRun.backendComparison = backendComparison;
flightRun.comparisonResult = struct();
if strcmp(options.Backend,'COMPARE')
    flightRun.comparisonResult = simulinkResult;
end
flightRun.overallPass = metrics.overallPass;
if strcmp(options.Backend,'COMPARE')
    flightRun.overallPass = metrics.overallPass && ...
        backendComparison.simulinkOverallPass && backendComparison.pass;
end
flightRun.figures = gobjects(0,1);
flightRun.exportedFiles = struct();

if options.PrintSummary
    printSummary(flightRun);
end
if options.ShowPlots
    flightRun.figures = plotC172sFlightControlScenario( ...
        result,scenario,metrics);
    if strcmp(options.Backend,'COMPARE')
        comparisonFigure = plotC172sFlightControlBackendComparison( ...
            result,simulinkResult,scenario,backendComparison);
        flightRun.figures(end+1,1) = comparisonFigure;
    end
end
if options.ExportResults
    flightRun.exportedFiles = exportC172sFlightControlRun( ...
        flightRun,options.ExportFolder);
    if options.PrintSummary
        fprintf('Exported MAT                       : %s\n', ...
            flightRun.exportedFiles.mat);
        fprintf('Exported CSV                       : %s\n', ...
            flightRun.exportedFiles.csv);
        fprintf('Exported summary                   : %s\n', ...
            flightRun.exportedFiles.summary);
    end
end
end

function options = runOptions(provided)
assert(isstruct(provided) && isscalar(provided), ...
    'runC172sFlightControlScenario:InvalidOptions', ...
    'options must be a scalar structure.');
options.ShowPlots = true;
options.PrintSummary = true;
options.ExportResults = false;
options.ExportFolder = '';
options.Backend = 'MATLAB';
fields = fieldnames(provided);
unknown = setdiff(fields,fieldnames(options));
assert(isempty(unknown), ...
    'runC172sFlightControlScenario:UnknownOption', ...
    'Unknown option field(s): %s',strjoin(unknown,', '));
for k = 1:numel(fields)
    options.(fields{k}) = provided.(fields{k});
end
logicalFields = {'ShowPlots','PrintSummary','ExportResults'};
for k = 1:numel(logicalFields)
    value = options.(logicalFields{k});
    assert((islogical(value) || isnumeric(value)) && isscalar(value), ...
        'runC172sFlightControlScenario:InvalidOption', ...
        '%s must be scalar logical.',logicalFields{k});
    options.(logicalFields{k}) = logical(value);
end
if isstring(options.Backend) && isscalar(options.Backend)
    options.Backend = char(options.Backend);
end
assert(ischar(options.Backend) && isrow(options.Backend), ...
    'runC172sFlightControlScenario:InvalidBackend', ...
    'Backend must be MATLAB, SIMULINK or COMPARE.');
options.Backend = upper(strtrim(options.Backend));
assert(any(strcmp(options.Backend,{'MATLAB','SIMULINK','COMPARE'})), ...
    'runC172sFlightControlScenario:InvalidBackend', ...
    'Backend must be MATLAB, SIMULINK or COMPARE.');
end

function printSummary(flightRun)
m = flightRun.metrics;
c = flightRun.scenario.configuration;
fprintf('============================================================\n');
fprintf(' C172S FLIGHT DYNAMICS AND AUTOPILOT LABORATORY - RESULT\n');
fprintf('============================================================\n\n');
fprintf('%-35s: %s\n','Scenario',flightRun.scenario.name);
fprintf('%-35s: %.2f s\n','Simulated duration',m.duration_s);
fprintf('%-35s: %.2f s\n','Execution wall time',flightRun.wallTime_s);
fprintf('%-35s: %s\n','Execution backend',flightRun.executionBackend);
if strcmp(flightRun.executionBackend,'COMPARE')
    b = flightRun.backendComparison;
    fprintf('%-35s: %.2f / %.2f s\n','MATLAB / Simulink wall time', ...
        flightRun.backendWallTime_s.matlab, ...
        flightRun.backendWallTime_s.simulink);
    fprintf('%-35s: %.9g\n','Maximum 58-signal difference', ...
        b.maximumAbsoluteError);
    fprintf('%-35s: signal %d at %.3f s\n', ...
        'Maximum-difference location',b.maximumErrorSignalIndex, ...
        b.maximumErrorTime_s);
    fprintf('%-35s: %s\n','Backend equivalence',statusText(b.pass));
end
fprintf('%-35s: %s / %s\n','Requested modes', ...
    c.lateralMode,c.verticalMode);
fprintf('%-35s: %s / %s\n','Final modes', ...
    m.final.lateralMode,m.final.verticalMode);
if strcmp(m.final.lateralMode,'ROLL')
    fprintf('%-35s: %.6f deg\n','Final captured bank command', ...
        m.final.bankCommand_deg);
    fprintf('%-35s: %.6f deg\n','Final bank capture error', ...
        m.final.bankCaptureError_deg);
elseif strcmp(m.final.lateralMode,'HEADING')
    fprintf('%-35s: %.6f deg\n','Final heading error', ...
        m.final.headingError_deg);
else
    fprintf('%-35s: released/OFF\n','Final lateral tracking');
end
if strcmp(m.final.verticalMode,'PITCH')
    fprintf('%-35s: %.6f deg\n','Final captured pitch command', ...
        m.final.pitchCommand_deg);
    fprintf('%-35s: %.6f deg\n','Final pitch capture error', ...
        m.final.pitchCaptureError_deg);
elseif strcmp(m.final.verticalMode,'ALTITUDE')
    fprintf('%-35s: %.6f ft\n','Final altitude error', ...
        m.final.altitudeError_ft);
else
    fprintf('%-35s: released/OFF\n','Final vertical tracking');
end
fprintf('%-35s: %.5f / %.5f / %.5f deg/s\n','Peak p / q / r', ...
    m.peak.bodyRate_degps);
fprintf('%-35s: %.5f / %.5f / %.5f deg\n', ...
    'Peak elevator / aileron / rudder',m.peak.surface_deg);
fprintf('%-35s: %.3f to %.3f KTAS\n','TAS range', ...
    m.airData.minimumTAS_KTAS,m.airData.maximumTAS_KTAS);
fprintf('%-35s: %d\n','Envelope violation samples', ...
    m.health.envelopeViolationSamples);
fprintf('%-35s: %.3g\n','Maximum quaternion norm error', ...
    m.health.maximumQuaternionNormError);
fprintf('%-35s: %s\n','Numerical/envelope health', ...
    statusText(m.health.pass));
fprintf('%-35s: %s\n','Scenario objective', ...
    statusText(m.objective.pass));
fprintf('%-35s: %s\n','Overall result',statusText(flightRun.overallPass));

if isempty(m.events)
    fprintf('%-35s: none\n','Mode-manager events');
else
    fprintf('%-35s: %s at %.3f s\n','First mode-manager event', ...
        m.events(1).Name,m.events(1).FirstTime_s);
    for k = 2:numel(m.events)
        fprintf('%-35s  %s at %.3f s\n','', ...
            m.events(k).Name,m.events(k).FirstTime_s);
    end
end
fprintf(['\nNOTE: bounded near-trim research result with ideal sensors ', ...
    'and one-point throttle; not aircraft approval.\n\n']);
end

function text = statusText(status)
if status
    text = 'PASS';
else
    text = 'REVIEW';
end
end
