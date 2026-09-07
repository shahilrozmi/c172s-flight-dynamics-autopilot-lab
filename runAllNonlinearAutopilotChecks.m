function results = runAllNonlinearAutopilotChecks()
%RUNALLNONLINEARAUTOPILOTCHECKS Execute the nonlinear-autopilot V0.1 gate.
%
%   RESULTS = RUNALLNONLINEARAUTOPILOTCHECKS executes the three ordered
%   nonlinear-autopilot integration gates:
%     1. controller/nonlinear-plant interface audit (53 cases)
%     2. coupled closed-loop MATLAB audit (48 cases)
%     3. independent executable Simulink equivalence (11 scenarios)
%
%   The released flight-controls v1.0.0 and nonlinear-plant v0.1.0 source
%   packages remain external and frozen. Their saved 18/18 and 7/7 reports
%   are verified rather than regenerated. A complete record is written to
%   NonlinearAutopilotRegressionReport.txt beside this file.

clc;

projectRoot = fileparts(mfilename('fullpath'));
originalPath = path;
pathCleanup = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(projectRoot,'-begin');

flightReport = fullfile(projectRoot,'FlightControlRegressionReport.txt');
nonlinearReport = fullfile(projectRoot,'Nonlinear6DofRegressionReport.txt');
[flightPass,flightMessage] = verifyFlightControlEvidence(flightReport);
[nonlinearPass,nonlinearMessage] = verifyNonlinearEvidence( ...
    nonlinearReport);

checks = regressionManifest();
nCheck = numel(checks);
emptyResult = struct( ...
    'Stage','', ...
    'Name','', ...
    'Artifact','', ...
    'Checker','', ...
    'ExpectedCases',0, ...
    'QualifiedCases',0, ...
    'ObservedCases',0, ...
    'Path','', ...
    'Status','NOT RUN', ...
    'Duration_s',NaN, ...
    'Identifier','', ...
    'Message','', ...
    'Output','');
results = repmat(emptyResult,nCheck,1);

fprintf('============================================================\n');
fprintf(' C172S NONLINEAR AUTOPILOT V0.1 - CONSOLIDATED REGRESSION\n');
fprintf('============================================================\n');
fprintf('Nonlinear-autopilot gates            : %d\n',nCheck);
fprintf('Frozen flight-controls evidence      : %s\n',flightMessage);
fprintf('Frozen nonlinear 6-DOF evidence      : %s\n\n', ...
    nonlinearMessage);

for k = 1:nCheck
    results(k).Stage = checks(k).Stage;
    results(k).Name = checks(k).Name;
    results(k).Artifact = checks(k).Artifact;
    results(k).Checker = checks(k).Checker;
    results(k).ExpectedCases = checks(k).ExpectedCases;
    results(k).QualifiedCases = checks(k).QualifiedCases;

    checkerPath = resolveLocalChecker(projectRoot,checks(k).Checker);
    if isempty(checkerPath)
        results(k).Status = 'FAIL';
        results(k).Identifier = ...
            'runAllNonlinearAutopilotChecks:MissingChecker';
        results(k).Message = sprintf( ...
            'MATLAB could not locate %s.m.',checks(k).Checker);
        fprintf('[FAIL] %d/%d %-42s checker not found\n', ...
            k,nCheck,checks(k).Name);
        continue;
    end

    results(k).Path = checkerPath;
    fprintf('Running %d/%d [%s] %s ...\n', ...
        k,nCheck,checks(k).Stage,checks(k).Name);
    startTime = tic;
    gateResult = [];
    try
        results(k).Output = evalc( ...
            'gateResult = feval(checks(k).Checker);');
        results(k).Duration_s = toc(startTime);
        results(k).ObservedCases = numel(gateResult);

        assert(results(k).ObservedCases == checks(k).ExpectedCases, ...
            'runAllNonlinearAutopilotChecks:CaseCountChanged', ...
            '%s returned %d cases; %d are frozen.', ...
            checks(k).Checker,results(k).ObservedCases, ...
            checks(k).ExpectedCases);
        assert(returnedCasesPass(gateResult,checks(k).RequiresStatus), ...
            'runAllNonlinearAutopilotChecks:ReturnedFailure', ...
            '%s returned one or more failed cases.',checks(k).Checker);

        results(k).Status = 'PASS';
        fprintf('[PASS] %s (%d qualified cases, %.2f s)\n\n', ...
            checks(k).Name,checks(k).QualifiedCases, ...
            results(k).Duration_s);
    catch exception
        results(k).Duration_s = toc(startTime);
        results(k).Status = 'FAIL';
        results(k).Identifier = exception.identifier;
        results(k).Message = exception.message;
        exceptionReport = getReport(exception,'extended', ...
            'hyperlinks','off');
        if isempty(results(k).Output)
            results(k).Output = exceptionReport;
        else
            results(k).Output = sprintf('%s\n\n%s', ...
                results(k).Output,exceptionReport);
        end
        fprintf('[FAIL] %s (%.2f s)\n', ...
            checks(k).Name,results(k).Duration_s);
        fprintf('       %s\n\n',oneLine(exception.message));
    end
end

status = {results.Status};
passCount = sum(strcmp(status,'PASS'));
failCount = sum(strcmp(status,'FAIL'));
externalPass = flightPass && nonlinearPass;
combinedPassCount = 25+passCount;
combinedTotal = 25+nCheck;
reportFile = fullfile(projectRoot, ...
    'NonlinearAutopilotRegressionReport.txt');
writeRegressionReport(reportFile,results,passCount,failCount, ...
    flightPass,flightMessage,flightReport, ...
    nonlinearPass,nonlinearMessage,nonlinearReport);

assignin('base','nonlinearAutopilotRegressionResults',results);
assignin('base','nonlinearAutopilotRegressionReport',reportFile);

fprintf('============================================================\n');
fprintf('                    CONSOLIDATED RESULT\n');
fprintf('============================================================\n');
for k = 1:nCheck
    fprintf('%-16s %-50s : %s\n', ...
        results(k).Stage,results(k).Name,results(k).Status);
end
fprintf('------------------------------------------------------------\n');
fprintf('Nonlinear-autopilot gates passed     : %d/%d\n', ...
    passCount,nCheck);
fprintf('Frozen flight-controls evidence      : %s\n',flightMessage);
fprintf('Frozen nonlinear 6-DOF evidence      : %s\n',nonlinearMessage);
if externalPass
    fprintf('Combined project evidence            : %d/%d\n', ...
        combinedPassCount,combinedTotal);
else
    fprintf('Combined project evidence            : NOT ESTABLISHED\n');
end
fprintf('Regression report                    : %s\n',reportFile);

if failCount == 0 && externalPass
    fprintf(['\nPASS: all three nonlinear-autopilot gates pass and the ', ...
        'frozen controller/plant evidence remains 25/25.\n']);
    fprintf(['PASS: combined project regression evidence is 28/28 ', ...
        'without modifying either released dependency.\n']);
    fprintf(['NOTE: V0.1 remains near-trim with ideal sensors and ', ...
        'one-point throttle; it is not aircraft approval.\n']);
else
    if ~flightPass
        fprintf('\nFAIL: %s\n',flightMessage);
    end
    if ~nonlinearPass
        fprintf('\nFAIL: %s\n',nonlinearMessage);
    end
    error('runAllNonlinearAutopilotChecks:RegressionGateFailure', ...
        ['Nonlinear-autopilot gates passed %d/%d; flight-controls ', ...
         'evidence valid = %d; nonlinear evidence valid = %d. See %s.'], ...
        passCount,nCheck,flightPass,nonlinearPass,reportFile);
end
end

function checks = regressionManifest()
% The executable checker qualifies ten named captures plus deterministic
% replay. Its public result contains the ten capture records; replay is
% asserted internally before the function can return.
checks = [ ...
    makeCheck('Interface', ...
        'Controller/nonlinear-plant interface audit', ...
        'NONLINEAR-AUTOPILOT-V1-INTERFACE-AUDIT-MATLAB-R1', ...
        'auditC172sNonlinearAutopilotInterfaces',53,53,true); ...
    makeCheck('Closed-loop', ...
        'Coupled nonlinear closed-loop MATLAB audit', ...
        'NONLINEAR-AUTOPILOT-V1-CLOSED-LOOP-MATLAB-R1', ...
        'auditC172sNonlinearAutopilotClosedLoop',48,48,true); ...
    makeCheck('Executable', ...
        'Executable nonlinear-autopilot Simulink equivalence', ...
        'C172S_NonlinearAutopilot_V1', ...
        'runNonlinearAutopilotSimulinkCheck',10,11,false)];
end

function check = makeCheck(stage,name,artifact,checker,expectedCases, ...
    qualifiedCases,requiresStatus)
check = struct('Stage',stage,'Name',name,'Artifact',artifact, ...
    'Checker',checker,'ExpectedCases',expectedCases, ...
    'QualifiedCases',qualifiedCases, ...
    'RequiresStatus',requiresStatus);
end

function checkerPath = resolveLocalChecker(projectRoot,checker)
localCandidate = fullfile(projectRoot,[checker '.m']);
if isfile(localCandidate)
    checkerPath = localCandidate;
else
    checkerPath = which(checker);
end
end

function passed = returnedCasesPass(gateResult,requiresStatus)
passed = ~isempty(gateResult) && isstruct(gateResult);
if ~passed || ~requiresStatus
    return;
end
if ~isfield(gateResult,'Status')
    passed = false;
    return;
end
values = {gateResult.Status};
for k = 1:numel(values)
    value = values{k};
    if islogical(value) || isnumeric(value)
        passed = passed && isscalar(value) && logical(value);
    elseif ischar(value) || (isstring(value) && isscalar(value))
        passed = passed && any(strcmpi(char(value), ...
            {'PASS','PASSED','TRUE'}));
    else
        passed = false;
    end
end
end

function [passed,message] = verifyFlightControlEvidence(reportFile)
passed = false;
if ~isfile(reportFile)
    message = 'MISSING - run runAllFlightControlChecks first';
    return;
end
text = fileread(reportFile);
passed = ~isempty(regexp(text,'Passed:\s*18/18','once')) && ...
    ~isempty(regexp(text,'Failed:\s*0/18','once'));
if passed
    message = '18/18 PASS (external frozen release)';
else
    message = 'INVALID - saved report does not establish 18/18 PASS';
end
end

function [passed,message] = verifyNonlinearEvidence(reportFile)
passed = false;
if ~isfile(reportFile)
    message = 'MISSING - run runAllNonlinear6DofChecks first';
    return;
end
text = fileread(reportFile);
passed = ~isempty(regexp(text, ...
    'Nonlinear gates passed:\s*7/7','once')) && ...
    ~isempty(regexp(text, ...
    'Nonlinear gates failed:\s*0/7','once')) && ...
    ~isempty(regexp(text, ...
    'Combined project evidence:\s*25/25 PASS','once'));
if passed
    message = '7/7 PASS (external frozen release)';
else
    message = 'INVALID - saved report does not establish 7/7 PASS';
end
end

function writeRegressionReport(reportFile,results,passCount,failCount, ...
    flightPass,flightMessage,flightReport, ...
    nonlinearPass,nonlinearMessage,nonlinearReport)
fid = fopen(reportFile,'w');
if fid < 0
    error('runAllNonlinearAutopilotChecks:ReportWriteFailed', ...
        'Unable to write regression report: %s',reportFile);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

generated = datetime('now','TimeZone','UTC', ...
    'Format','yyyy-MM-dd''T''HH:mm:ss''Z''');
fprintf(fid, ...
    'C172S NONLINEAR AUTOPILOT V0.1 - CONSOLIDATED REGRESSION REPORT\n');
fprintf(fid,'Generated UTC: %s\n',char(generated));
fprintf(fid,'MATLAB: %s\n',version);
fprintf(fid,'Nonlinear-autopilot gates passed: %d/%d\n', ...
    passCount,numel(results));
fprintf(fid,'Nonlinear-autopilot gates failed: %d/%d\n', ...
    failCount,numel(results));
fprintf(fid,'Frozen flight-controls evidence valid: %d\n',flightPass);
fprintf(fid,'Frozen flight-controls evidence: %s\n',flightMessage);
fprintf(fid,'Frozen flight-controls report: %s\n',flightReport);
fprintf(fid,'Frozen nonlinear 6-DOF evidence valid: %d\n',nonlinearPass);
fprintf(fid,'Frozen nonlinear 6-DOF evidence: %s\n',nonlinearMessage);
fprintf(fid,'Frozen nonlinear 6-DOF report: %s\n',nonlinearReport);
if flightPass && nonlinearPass && failCount == 0
    fprintf(fid,'Combined project evidence: %d/%d PASS\n', ...
        25+passCount,25+numel(results));
elseif flightPass && nonlinearPass
    fprintf(fid,'Combined project evidence: %d/%d; NOT PASSING\n', ...
        25+passCount,25+numel(results));
else
    fprintf(fid,'Combined project evidence: NOT ESTABLISHED\n');
end
fprintf(fid,['Scope: frozen controller v1.0.0 operating one coupled ', ...
    'nonlinear 13-state research plant v0.1.0 through interface, ', ...
    'closed-loop MATLAB and independent executable Simulink gates.\n']);
fprintf(fid,['Qualification: near-trim attached flow, provisional mass ', ...
    'properties, ideal sensors and one-point throttle; no stall/spin, ', ...
    'sensor-imperfection, autothrottle or aircraft-truth claim; not ', ...
    'aircraft approval.\n\n']);

for k = 1:numel(results)
    fprintf(fid,'============================================================\n');
    fprintf(fid,'%d. [%s] %s\n',k,results(k).Stage,results(k).Name);
    fprintf(fid,'Artifact: %s\n',results(k).Artifact);
    fprintf(fid,'Checker: %s.m\n',results(k).Checker);
    fprintf(fid,'Path: %s\n',results(k).Path);
    fprintf(fid,'Status: %s\n',results(k).Status);
    fprintf(fid,'Returned records: %d/%d\n', ...
        results(k).ObservedCases,results(k).ExpectedCases);
    fprintf(fid,'Qualified cases: %d\n',results(k).QualifiedCases);
    fprintf(fid,'Duration: %.3f s\n',results(k).Duration_s);
    if ~isempty(results(k).Identifier)
        fprintf(fid,'Identifier: %s\n',results(k).Identifier);
    end
    if ~isempty(results(k).Message)
        fprintf(fid,'Message: %s\n',results(k).Message);
    end
    fprintf(fid,'\nCaptured output:\n%s\n\n',results(k).Output);
end
end

function text = oneLine(text)
text = regexprep(text,'\s+',' ');
text = strtrim(text);
end
