function files = exportC172sFlightControlRun(flightRun,folder)
%EXPORTC172SFLIGHTCONTROLRUN Save MAT, CSV and text scenario evidence.

assert(isstruct(flightRun) && isfield(flightRun,'scenario') && ...
    isfield(flightRun,'result') && isfield(flightRun,'metrics'), ...
    'exportC172sFlightControlRun:InvalidRun', ...
    'flightRun must contain scenario, result and metrics.');
if nargin < 2 || isempty(folder)
    projectRoot = fileparts(mfilename('fullpath'));
    lab = c172sFlightControlLabData();
    folder = fullfile(projectRoot,lab.export.defaultFolderName);
end
if isstring(folder) && isscalar(folder)
    folder = char(folder);
end
assert(ischar(folder) && isrow(folder), ...
    'exportC172sFlightControlRun:InvalidFolder', ...
    'folder must be a character vector or scalar string.');
if ~isfolder(folder)
    [made,message] = mkdir(folder);
    assert(made,'exportC172sFlightControlRun:CreateFailed', ...
        'Could not create export folder: %s',message);
end

stamp = char(datetime('now','TimeZone','UTC', ...
    'Format','yyyyMMdd''T''HHmmssSSS''Z'''));
name = regexprep(lower(flightRun.scenario.name),'[^a-z0-9]+','_');
name = regexprep(name,'^_+|_+$','');
base = fullfile(folder,[stamp '_' name]);
files.mat = [base '.mat'];
files.csv = [base '.csv'];
files.summary = [base '_summary.txt'];
assert(~isfile(files.mat) && ~isfile(files.csv) && ...
    ~isfile(files.summary), ...
    'exportC172sFlightControlRun:TargetExists', ...
    'An export with the same UTC timestamp already exists.');

flightRun.figures = gobjects(0,1);
save(files.mat,'flightRun');
tableData = resultTable(flightRun.result);
writetable(tableData,files.csv);
writeSummary(files.summary,flightRun);
end

function output = resultTable(result)
output = table();
output.Time_s = result.time_s;
output.u_mps = result.plantState(:,1);
output.v_mps = result.plantState(:,2);
output.w_mps = result.plantState(:,3);
output.p_degps = rad2deg(result.plantState(:,4));
output.q_degps = rad2deg(result.plantState(:,5));
output.r_degps = rad2deg(result.plantState(:,6));
output.Bank_deg = rad2deg(result.euler_rad(:,1));
output.Pitch_deg = rad2deg(result.euler_rad(:,2));
output.Heading_deg = mod(rad2deg(result.euler_rad(:,3)),360);
output.Altitude_ft = result.altitude_ft;
output.TAS_KTAS = result.TAS_mps/0.514444444444444;
output.Alpha_deg = rad2deg(result.alpha_rad);
output.Beta_deg = rad2deg(result.beta_rad);
output.Elevator_deg = rad2deg(result.control(:,1));
output.Aileron_deg = rad2deg(result.control(:,2));
output.Rudder_deg = rad2deg(result.control(:,3));
output.Throttle = result.control(:,4);
output.BankCommand_deg = result.bankCommand_deg;
output.PitchCommand_deg = result.pitchCommand_deg;
output.HeadingCommand_deg = result.headingCommand_deg;
output.AltitudeCommand_ft = result.altitudeCommand_ft;
output.Engaged = result.engaged;
output.LateralMode = result.lateralMode;
output.VerticalMode = result.verticalMode;
output.EventCode = result.eventCode;
output.WithinEnvelope = result.withinEnvelope;
output.QuaternionNorm = result.quaternionNorm;
end

function writeSummary(path,flightRun)
fid = fopen(path,'w');
assert(fid >= 0,'exportC172sFlightControlRun:SummaryWriteFailed', ...
    'Unable to write %s.',path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
m = flightRun.metrics;
c = flightRun.scenario.configuration;
fprintf(fid,'C172S FLIGHT DYNAMICS AND AUTOPILOT LABORATORY\n');
fprintf(fid,'Artifact: %s\n',flightRun.labArtifactRevision);
fprintf(fid,'Scenario: %s\n',flightRun.scenario.name);
fprintf(fid,'Preset: %s\n',flightRun.scenario.preset);
fprintf(fid,'Duration: %.3f s\n',m.duration_s);
fprintf(fid,'Execution backend: %s\n',flightRun.executionBackend);
if strcmp(flightRun.executionBackend,'COMPARE')
    fprintf(fid,'MATLAB wall time: %.9g s\n', ...
        flightRun.backendWallTime_s.matlab);
    fprintf(fid,'Simulink wall time: %.9g s\n', ...
        flightRun.backendWallTime_s.simulink);
    fprintf(fid,'Maximum 58-signal difference: %.9g\n', ...
        flightRun.backendComparison.maximumAbsoluteError);
    fprintf(fid,'Maximum-difference signal/time: %d / %.9g s\n', ...
        flightRun.backendComparison.maximumErrorSignalIndex, ...
        flightRun.backendComparison.maximumErrorTime_s);
    fprintf(fid,'Backend equivalence pass: %d\n', ...
        flightRun.backendComparison.pass);
end
fprintf(fid,'Requested modes: %s / %s\n', ...
    c.lateralMode,c.verticalMode);
fprintf(fid,'Selected heading: %.6f deg\n', ...
    flightRun.scenario.selectedHeading_deg);
fprintf(fid,'Selected altitude: %.6f ft\n', ...
    flightRun.scenario.selectedAltitude_ft);
fprintf(fid,'Final modes: %s / %s\n', ...
    m.final.lateralMode,m.final.verticalMode);
fprintf(fid,'Final captured bank command: %.9g deg\n', ...
    m.final.bankCommand_deg);
fprintf(fid,'Final bank capture error: %.9g deg\n', ...
    m.final.bankCaptureError_deg);
fprintf(fid,'Final captured pitch command: %.9g deg\n', ...
    m.final.pitchCommand_deg);
fprintf(fid,'Final pitch capture error: %.9g deg\n', ...
    m.final.pitchCaptureError_deg);
fprintf(fid,'Final heading error: %.9g deg\n',m.final.headingError_deg);
fprintf(fid,'Final altitude error: %.9g ft\n',m.final.altitudeError_ft);
fprintf(fid,'Peak p/q/r: %.6f / %.6f / %.6f deg/s\n', ...
    m.peak.bodyRate_degps);
fprintf(fid,'Peak elevator/aileron/rudder: %.6f / %.6f / %.6f deg\n', ...
    m.peak.surface_deg);
fprintf(fid,'Envelope violation samples: %d\n', ...
    m.health.envelopeViolationSamples);
fprintf(fid,'Maximum quaternion norm error: %.9g\n', ...
    m.health.maximumQuaternionNormError);
fprintf(fid,'Health pass: %d\n',m.health.pass);
fprintf(fid,'Objective pass: %d\n',m.objective.pass);
fprintf(fid,'Overall pass: %d\n',flightRun.overallPass);
fprintf(fid,['Qualification: near-trim V0.1 research simulation; ideal ', ...
    'sensors and one-point throttle; not aircraft approval.\n']);
end
