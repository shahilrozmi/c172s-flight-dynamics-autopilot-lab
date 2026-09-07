function comparison = compareC172sNonlinearAutopilotBackends( ...
    matlabResult,simulinkResult,tolerance)
%COMPAREC172SNONLINEARAUTOPILOTBACKENDS Compare all 58 public signals.

if nargin < 3 || isempty(tolerance)
    tolerance = 3.0e-7;
end
assert(isnumeric(tolerance) && isscalar(tolerance) && ...
    isfinite(tolerance) && tolerance > 0, ...
    'compareC172sNonlinearAutopilotBackends:InvalidTolerance', ...
    'tolerance must be a positive finite scalar.');
assert(isequal(size(matlabResult.time_s),size(simulinkResult.time_s)) && ...
    max(abs(matlabResult.time_s-simulinkResult.time_s)) <= 1.0e-10, ...
    'compareC172sNonlinearAutopilotBackends:TimeMismatch', ...
    'MATLAB and Simulink results use different time grids.');

reference = c172sNonlinearAutopilotResultMatrix(matlabResult);
executable = c172sNonlinearAutopilotResultMatrix(simulinkResult);
assert(isequal(size(reference),size(executable)), ...
    'compareC172sNonlinearAutopilotBackends:SizeMismatch', ...
    'MATLAB and Simulink result matrices have different sizes.');

absoluteError = abs(reference-executable);
comparison.signalCount = size(reference,2);
comparison.sampleCount = size(reference,1);
comparison.tolerance = tolerance;
[comparison.maximumAbsoluteError,linearIndex] = max(absoluteError(:));
[sampleIndex,signalIndex] = ind2sub(size(absoluteError),linearIndex);
comparison.maximumErrorTime_s = matlabResult.time_s(sampleIndex);
comparison.maximumErrorSignalIndex = signalIndex;
comparison.stateMaximumError = max(absoluteError(:,1:26),[],'all');
comparison.controlMaximumError = max(absoluteError(:,27:33),[],'all');
comparison.diagnosticMaximumError = max(absoluteError(:,34:58),[],'all');
comparison.discreteSignalsExact = isequal(reference(:,44:50), ...
    executable(:,44:50)) && isequal(reference(:,55:56), ...
    executable(:,55:56));
comparison.pass = comparison.maximumAbsoluteError <= tolerance && ...
    comparison.discreteSignalsExact;
comparison.referenceBackend = 'MATLAB';
comparison.executableBackend = 'SIMULINK';
end
