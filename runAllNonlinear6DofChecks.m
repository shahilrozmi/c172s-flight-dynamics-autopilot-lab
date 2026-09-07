function results = runAllNonlinear6DofChecks()
%RUNALLNONLINEAR6DOFCHECKS Execute the frozen nonlinear 6-DOF V0.1 gate.
%
%   RESULTS = RUNALLNONLINEAR6DOFCHECKS runs the seven ordered MATLAB and
%   Simulink qualification gates for the C172S Nonlinear 6-DOF Research
%   Validation Plant V0.1. The previously released flight-controls v1.0.0
%   regression remains external and is verified from its saved 18/18 report.
%
%   A complete record is written to Nonlinear6DofRegressionReport.txt in
%   the project folder. Structured results are also assigned to
%   nonlinear6DofRegressionResults in the base workspace.

clc;

projectRoot = fileparts(mfilename('fullpath'));
checks = regressionManifest();
nCheck = numel(checks);

flightReport = fullfile(projectRoot,'FlightControlRegressionReport.txt');
[flightEvidencePass,flightEvidenceMessage] = verifyFlightControlEvidence( ...
    flightReport);

emptyResult = struct( ...
    'Stage','', ...
    'Name','', ...
    'Artifact','', ...
    'Checker','', ...
    'ExpectedCases',0, ...
    'ObservedCases',0, ...
    'Path','', ...
    'Status','NOT RUN', ...
    'Duration_s',NaN, ...
    'Identifier','', ...
    'Message','', ...
    'Output','');
results = repmat(emptyResult,nCheck,1);

fprintf('============================================================\n');
fprintf(' C172S NONLINEAR 6-DOF V0.1 - CONSOLIDATED REGRESSION GATE\n');
fprintf('============================================================\n');
fprintf('Nonlinear qualification gates        : %d\n',nCheck);
fprintf('Frozen flight-controls evidence       : %s\n\n', ...
    flightEvidenceMessage);

for k = 1:nCheck
    results(k).Stage = checks(k).Stage;
    results(k).Name = checks(k).Name;
    results(k).Artifact = checks(k).Artifact;
    results(k).Checker = checks(k).Checker;
    results(k).ExpectedCases = checks(k).ExpectedCases;

    localCandidate = fullfile(projectRoot,[checks(k).Checker '.m']);
    if isfile(localCandidate)
        checkerPath = localCandidate;
    else
        checkerPath = which(checks(k).Checker);
    end

    if isempty(checkerPath)
        results(k).Status = 'FAIL';
        results(k).Identifier = ...
            'runAllNonlinear6DofChecks:MissingChecker';
        results(k).Message = sprintf( ...
            'MATLAB could not locate %s.m.',checks(k).Checker);
        fprintf('[FAIL] %d/%d %-39s checker not found\n', ...
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
            'runAllNonlinear6DofChecks:CaseCountChanged', ...
            '%s returned %d cases; %d are frozen.', ...
            checks(k).Checker,results(k).ObservedCases, ...
            checks(k).ExpectedCases);
        assert(allReturnedCasesPass(gateResult), ...
            'runAllNonlinear6DofChecks:ReturnedFailure', ...
            '%s returned one or more failed cases.',checks(k).Checker);

        results(k).Status = 'PASS';
        fprintf('[PASS] %s (%d cases, %.2f s)\n\n', ...
            checks(k).Name,results(k).ObservedCases, ...
            results(k).Duration_s);
    catch ME
        results(k).Duration_s = toc(startTime);
        results(k).Status = 'FAIL';
        results(k).Identifier = ME.identifier;
        results(k).Message = ME.message;
        if isempty(results(k).Output)
            results(k).Output = getReport(ME,'extended', ...
                'hyperlinks','off');
        else
            results(k).Output = sprintf('%s\n\n%s',results(k).Output, ...
                getReport(ME,'extended','hyperlinks','off'));
        end
        fprintf('[FAIL] %s (%.2f s)\n', ...
            checks(k).Name,results(k).Duration_s);
        fprintf('       %s\n\n',oneLine(ME.message));
    end
end

status = {results.Status};
passCount = sum(strcmp(status,'PASS'));
failCount = sum(strcmp(status,'FAIL'));
combinedPassCount = 18+passCount;
combinedTotal = 18+nCheck;
reportFile = fullfile(projectRoot,'Nonlinear6DofRegressionReport.txt');
writeRegressionReport(reportFile,results,passCount,failCount, ...
    flightEvidencePass,flightEvidenceMessage,flightReport);

assignin('base','nonlinear6DofRegressionResults',results);
assignin('base','nonlinear6DofRegressionReport',reportFile);

fprintf('============================================================\n');
fprintf('                    CONSOLIDATED RESULT\n');
fprintf('============================================================\n');
for k = 1:nCheck
    fprintf('%-18s %-48s : %s\n', ...
        results(k).Stage,results(k).Name,results(k).Status);
end
fprintf('------------------------------------------------------------\n');
fprintf('Nonlinear gates passed               : %d/%d\n', ...
    passCount,nCheck);
fprintf('Frozen flight-controls evidence       : %s\n', ...
    flightEvidenceMessage);
if flightEvidencePass
    fprintf('Combined project evidence             : %d/%d\n', ...
        combinedPassCount,combinedTotal);
else
    fprintf('Combined project evidence             : NOT ESTABLISHED\n');
end
fprintf('Regression report                     : %s\n',reportFile);

if failCount == 0 && flightEvidencePass
    fprintf(['\nPASS: all seven nonlinear research-plant gates pass and ', ...
        'the released flight-controls evidence remains 18/18.\n']);
    fprintf(['PASS: combined project regression evidence is 25/25 ', ...
        'without modifying the v1.0.0 controller release.\n']);
    fprintf(['NOTE: V0.1 remains a near-trim attached-flow research ', ...
        'plant with one-point thrust; it is not aircraft approval.\n']);
else
    if ~flightEvidencePass
        fprintf('\nFAIL: %s\n',flightEvidenceMessage);
    end
    error('runAllNonlinear6DofChecks:RegressionGateFailure', ...
        ['Nonlinear gates passed %d/%d; frozen flight-controls ', ...
         'evidence valid = %d. See %s.'], ...
        passCount,nCheck,flightEvidencePass,reportFile);
end
end

function checks = regressionManifest()
checks = [ ...
    makeCheck('Foundation','Source and convention audit', ...
        'NONLINEAR-6DOF-V1-SOURCE-AND-CONVENTION-AUDIT-MATLAB-R1', ...
        'auditC172sNonlinear6DofSources',31); ...
    makeCheck('Foundation','Rigid-body kernel audit', ...
        'NONLINEAR-6DOF-V1-RIGID-BODY-KERNEL-MATLAB-R1', ...
        'auditC172sRigidBody6DofEom',27); ...
    makeCheck('Environment','Atmosphere and air-data audit', ...
        'NONLINEAR-6DOF-V1-ATMOSPHERE-AIR-DATA-MATLAB-R1', ...
        'auditC172sAtmosphereAirData',27); ...
    makeCheck('Aerodynamics','Near-trim aerodynamics audit', ...
        'NONLINEAR-6DOF-V1-NEAR-TRIM-AERODYNAMICS-MATLAB-R1', ...
        'auditC172sNearTrimAerodynamics',27); ...
    makeCheck('Assembly','Trim and total-force audit', ...
        'NONLINEAR-6DOF-V1-TRIM-TOTAL-FORCE-MATLAB-R1', ...
        'auditC172sNonlinearTrim',27); ...
    makeCheck('Assembly','Integrated dynamics and linearization audit', ...
        'NONLINEAR-6DOF-V1-INTEGRATED-DYNAMICS-AND-LINEARIZATION-MATLAB-R1', ...
        'auditC172sIntegratedNonlinearDynamics',34); ...
    makeCheck('Executable','Executable Simulink equivalence', ...
        'C172S_Nonlinear6Dof_V1', ...
        'runNonlinear6DofSimulinkCheck',6)];
end

function check = makeCheck(stage,name,artifact,checker,expectedCases)
check = struct('Stage',stage,'Name',name,'Artifact',artifact, ...
    'Checker',checker,'ExpectedCases',expectedCases);
end

function passed = allReturnedCasesPass(gateResult)
passed = ~isempty(gateResult) && isstruct(gateResult);
if ~passed || ~isfield(gateResult,'Status')
    return;
end
values = {gateResult.Status};
for k = 1:numel(values)
    value = values{k};
    if islogical(value) || isnumeric(value)
        passed = passed && isscalar(value) && logical(value);
    elseif ischar(value) || (isstring(value) && isscalar(value))
        passed = passed && any(strcmpi(char(value),{'PASS','PASSED','TRUE'}));
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

function writeRegressionReport(reportFile,results,passCount,failCount, ...
    flightEvidencePass,flightEvidenceMessage,flightReport)
fid = fopen(reportFile,'w');
if fid < 0
    error('runAllNonlinear6DofChecks:ReportWriteFailed', ...
        'Unable to write regression report: %s',reportFile);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

generated = datetime('now','TimeZone','UTC', ...
    'Format','yyyy-MM-dd''T''HH:mm:ss''Z''');
fprintf(fid,'C172S NONLINEAR 6-DOF V0.1 - CONSOLIDATED REGRESSION REPORT\n');
fprintf(fid,'Generated UTC: %s\n',char(generated));
fprintf(fid,'MATLAB: %s\n',version);
fprintf(fid,'Nonlinear gates passed: %d/%d\n',passCount,numel(results));
fprintf(fid,'Nonlinear gates failed: %d/%d\n',failCount,numel(results));
fprintf(fid,'Frozen flight-controls evidence valid: %d\n',flightEvidencePass);
fprintf(fid,'Frozen flight-controls evidence: %s\n',flightEvidenceMessage);
fprintf(fid,'Frozen flight-controls report: %s\n',flightReport);
if flightEvidencePass
    fprintf(fid,'Combined project evidence: %d/%d PASS\n', ...
        18+passCount,18+numel(results));
else
    fprintf(fid,'Combined project evidence: NOT ESTABLISHED\n');
end
fprintf(fid,['Scope: C172S Nonlinear 6-DOF Research Validation Plant ', ...
    'V0.1 through independent executable Simulink equivalence.\n']);
fprintf(fid,['Qualification: near-trim attached flow, provisional mass ', ...
    'properties and one-point thrust; no stall/spin or aircraft-truth ', ...
    'claim; not aircraft approval.\n\n']);

for k = 1:numel(results)
    fprintf(fid,'============================================================\n');
    fprintf(fid,'%d. [%s] %s\n',k,results(k).Stage,results(k).Name);
    fprintf(fid,'Artifact: %s\n',results(k).Artifact);
    fprintf(fid,'Checker: %s.m\n',results(k).Checker);
    fprintf(fid,'Path: %s\n',results(k).Path);
    fprintf(fid,'Status: %s\n',results(k).Status);
    fprintf(fid,'Cases: %d/%d\n', ...
        results(k).ObservedCases,results(k).ExpectedCases);
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
