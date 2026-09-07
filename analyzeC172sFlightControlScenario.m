function metrics = analyzeC172sFlightControlScenario( ...
    result,scenario,model)
%ANALYZEC172SFLIGHTCONTROLSCENARIO Reduce a flight trace to useful metrics.

if nargin < 3 || isempty(model)
    model = c172sNonlinearAutopilotClosedLoopData();
end
validateInputs(result,scenario,model);

t = result.time_s(:);
manager = model.dependencies.manager;
lab = c172sFlightControlLabData(model);
heading_deg = mod(rad2deg(result.euler_rad(:,3)),360);
bank_deg = rad2deg(result.euler_rad(:,1));
pitch_deg = rad2deg(result.euler_rad(:,2));
rates_degps = rad2deg(result.plantState(:,4:6));
surface_deg = rad2deg(result.control(:,1:3));
surfaceRate_degps = rad2deg(result.surfaceRate_radps);
alpha_deg = rad2deg(result.alpha_rad);
beta_deg = rad2deg(result.beta_rad);

headingError_deg = rad2deg(wrapAngle( ...
    deg2rad(scenario.selectedHeading_deg)-result.euler_rad(:,3)));
altitudeError_ft = scenario.selectedAltitude_ft-result.altitude_ft;
bankError_deg = result.bankCommand_deg-bank_deg;
pitchError_deg = result.pitchCommand_deg-pitch_deg;

metrics.scenarioName = scenario.name;
metrics.duration_s = t(end)-t(1);
metrics.sampleCount = numel(t);
metrics.final.heading_deg = heading_deg(end);
metrics.final.altitude_ft = result.altitude_ft(end);
metrics.final.bank_deg = bank_deg(end);
metrics.final.pitch_deg = pitch_deg(end);
metrics.final.bankCommand_deg = result.bankCommand_deg(end);
metrics.final.pitchCommand_deg = result.pitchCommand_deg(end);
metrics.final.bankCaptureError_deg = bankError_deg(end);
metrics.final.pitchCaptureError_deg = pitchError_deg(end);
metrics.final.headingError_deg = headingError_deg(end);
metrics.final.altitudeError_ft = altitudeError_ft(end);
metrics.final.engaged = logical(result.engaged(end));
metrics.final.lateralModeCode = result.lateralMode(end);
metrics.final.verticalModeCode = result.verticalMode(end);
metrics.final.lateralMode = lateralModeName( ...
    result.lateralMode(end),manager);
metrics.final.verticalMode = verticalModeName( ...
    result.verticalMode(end),manager);

metrics.peak.bodyRate_degps = max(abs(rates_degps),[],1);
metrics.peak.surface_deg = max(abs(surface_deg),[],1);
metrics.peak.surfaceRate_degps = max(abs(surfaceRate_degps),[],1);
metrics.peak.bank_deg = max(abs(bank_deg));
metrics.peak.pitch_deg = max(abs(pitch_deg));
metrics.peak.alpha_deg = max(abs(alpha_deg));
metrics.peak.beta_deg = max(abs(beta_deg));
metrics.airData.minimumTAS_KTAS = min(result.TAS_mps)/0.514444444444444;
metrics.airData.maximumTAS_KTAS = max(result.TAS_mps)/0.514444444444444;

metrics.health.finiteTrace = allFiniteResult(result);
metrics.health.maximumQuaternionNormError = ...
    max(abs(result.quaternionNorm-1));
metrics.health.quaternionPass = ...
    metrics.health.maximumQuaternionNormError <= ...
    lab.analysis.maximumQuaternionNormError;
metrics.health.envelopeViolationSamples = sum(~result.withinEnvelope);
metrics.health.envelopeViolationTime_s = ...
    metrics.health.envelopeViolationSamples*model.execution.fixedStep_s;
metrics.health.envelopePass = metrics.health.envelopeViolationSamples == 0;
metrics.health.aileronSaturationSamples = sum(result.aileronSaturated);
metrics.health.elevatorSaturationSamples = sum(result.elevatorSaturated);
metrics.health.pass = metrics.health.finiteTrace && ...
    metrics.health.quaternionPass && metrics.health.envelopePass;

engagedIndices = find(result.engaged);
if isempty(engagedIndices)
    metrics.engagement.firstEngaged_s = NaN;
    metrics.engagement.lastEngaged_s = NaN;
else
    metrics.engagement.firstEngaged_s = t(engagedIndices(1));
    metrics.engagement.lastEngaged_s = t(engagedIndices(end));
end
metrics.engagement.engagedFraction = mean(double(result.engaged));

headingStep = abs(scenario.configuration.headingChange_deg);
headingBand = max(lab.analysis.minimumHeadingSettlingBand_deg, ...
    0.02*headingStep);
altitudeStep = abs(scenario.configuration.altitudeChange_ft);
altitudeBand = max(lab.analysis.minimumAltitudeSettlingBand_ft, ...
    0.02*altitudeStep);
metrics.tracking.headingSettlingBand_deg = headingBand;
metrics.tracking.altitudeSettlingBand_ft = altitudeBand;
metrics.tracking.bankSettlingBand_deg = ...
    lab.analysis.minimumBankSettlingBand_deg;
metrics.tracking.pitchSettlingBand_deg = ...
    lab.analysis.minimumPitchSettlingBand_deg;
metrics.tracking.headingSettlingTime_s = ...
    settlingTime(t,abs(headingError_deg),headingBand, ...
    scenario.configuration.engageTime_s);
metrics.tracking.altitudeSettlingTime_s = ...
    settlingTime(t,abs(altitudeError_ft),altitudeBand, ...
    scenario.configuration.engageTime_s);
metrics.tracking.bankSettlingTime_s = ...
    settlingTime(t,abs(bankError_deg), ...
    metrics.tracking.bankSettlingBand_deg, ...
    scenario.configuration.engageTime_s);
metrics.tracking.pitchSettlingTime_s = ...
    settlingTime(t,abs(pitchError_deg), ...
    metrics.tracking.pitchSettlingBand_deg, ...
    scenario.configuration.engageTime_s);

metrics.events = eventLedger(result.eventCode,t,manager);
metrics.objective.finalEngagementPass = ...
    metrics.final.engaged == scenario.expected.finalEngaged;
metrics.objective.finalLateralModePass = ...
    metrics.final.lateralModeCode == scenario.expected.finalLateralMode;
metrics.objective.finalVerticalModePass = ...
    metrics.final.verticalModeCode == scenario.expected.finalVerticalMode;
if scenario.expected.requiredEvent == manager.codes.event.NONE
    metrics.objective.requiredEventPass = true;
else
    metrics.objective.requiredEventPass = ...
        any(result.eventCode == scenario.expected.requiredEvent);
end

headingTrackingRequired = metrics.final.engaged && ...
    metrics.final.lateralModeCode == manager.codes.lateral.HEADING;
rollTrackingRequired = metrics.final.engaged && ...
    metrics.final.lateralModeCode == manager.codes.lateral.ROLL;
altitudeTrackingRequired = metrics.final.engaged && ...
    metrics.final.verticalModeCode == manager.codes.vertical.ALTITUDE;
pitchTrackingRequired = metrics.final.engaged && ...
    metrics.final.verticalModeCode == manager.codes.vertical.PITCH;
metrics.objective.headingTrackingPass = ~headingTrackingRequired || ...
    abs(metrics.final.headingError_deg) <= ...
    lab.analysis.headingTolerance_deg;
metrics.objective.altitudeTrackingPass = ~altitudeTrackingRequired || ...
    abs(metrics.final.altitudeError_ft) <= ...
    lab.analysis.altitudeTolerance_ft;
metrics.objective.bankCapturePass = ~rollTrackingRequired || ...
    abs(metrics.final.bankCaptureError_deg) <= ...
    lab.analysis.bankCaptureTolerance_deg;
metrics.objective.pitchCapturePass = ~pitchTrackingRequired || ...
    abs(metrics.final.pitchCaptureError_deg) <= ...
    lab.analysis.pitchCaptureTolerance_deg;
metrics.objective.pass = ...
    metrics.objective.finalEngagementPass && ...
    metrics.objective.finalLateralModePass && ...
    metrics.objective.finalVerticalModePass && ...
    metrics.objective.requiredEventPass && ...
    metrics.objective.headingTrackingPass && ...
    metrics.objective.altitudeTrackingPass && ...
    metrics.objective.bankCapturePass && ...
    metrics.objective.pitchCapturePass;
metrics.overallPass = metrics.health.pass && metrics.objective.pass;
end

function validateInputs(result,scenario,model)
requiredResult = {'time_s','plantState','control','surfaceRate_radps', ...
    'euler_rad','altitude_ft','TAS_mps','alpha_rad','beta_rad', ...
    'quaternionNorm','withinEnvelope','engaged','lateralMode', ...
    'verticalMode','eventCode','bankCommand_deg','pitchCommand_deg', ...
    'aileronSaturated','elevatorSaturated'};
assert(isstruct(result) && all(isfield(result,requiredResult)), ...
    'analyzeC172sFlightControlScenario:InvalidResult', ...
    'result does not contain the frozen nonlinear-autopilot trace.');
assert(isstruct(scenario) && isfield(scenario,'expected') && ...
    isfield(scenario,'configuration') && ...
    strcmp(scenario.backendArtifactRevision,model.artifactRevision), ...
    'analyzeC172sFlightControlScenario:InvalidScenario', ...
    'scenario does not belong to the frozen laboratory backend.');
end

function status = allFiniteResult(result)
numericFields = {'time_s','plantState','controllerState','control', ...
    'surfaceRate_radps','euler_rad','altitude_ft','TAS_mps', ...
    'alpha_rad','beta_rad','quaternionNorm','bankCommand_deg', ...
    'pitchCommand_deg','headingCommand_deg','altitudeCommand_ft'};
status = true;
for k = 1:numel(numericFields)
    if ~isfield(result,numericFields{k}) || ...
            ~all(isfinite(result.(numericFields{k})),'all')
        status = false;
        return;
    end
end
end

function value = settlingTime(time,errorMagnitude,tolerance,startTime)
eligible = find(time >= startTime);
value = NaN;
if isempty(eligible)
    return;
end

% The first permanently settled sample is one sample after the final band
% violation. This produces the same result as testing every remaining tail,
% while reducing the search from quadratic to linear complexity.
lastViolation = find( ...
    errorMagnitude(eligible) > tolerance,1,'last');
if isempty(lastViolation)
    value = time(eligible(1))-startTime;
elseif lastViolation < numel(eligible)
    value = time(eligible(lastViolation+1))-startTime;
end
end

function ledger = eventLedger(eventCode,time,manager)
codes = unique(eventCode(eventCode ~= manager.codes.event.NONE),'stable');
ledger = repmat(struct('Code',uint8(0),'Name','','FirstTime_s',0, ...
    'SampleCount',0),numel(codes),1);
for k = 1:numel(codes)
    selected = eventCode == codes(k);
    first = find(selected,1,'first');
    ledger(k).Code = codes(k);
    ledger(k).Name = eventName(codes(k),manager);
    ledger(k).FirstTime_s = time(first);
    ledger(k).SampleCount = sum(selected);
end
end

function name = eventName(code,manager)
eventFields = fieldnames(manager.codes.event);
name = sprintf('EVENT_%d',double(code));
for k = 1:numel(eventFields)
    if code == manager.codes.event.(eventFields{k})
        name = eventFields{k};
        return;
    end
end
end

function name = lateralModeName(code,manager)
name = codeName(code,manager.codes.lateral,'UNKNOWN');
end

function name = verticalModeName(code,manager)
name = codeName(code,manager.codes.vertical,'UNKNOWN');
end

function name = codeName(code,codes,fallback)
fields = fieldnames(codes);
name = fallback;
for k = 1:numel(fields)
    if code == codes.(fields{k})
        name = fields{k};
        return;
    end
end
end

function value = wrapAngle(value)
value = mod(value+pi,2*pi)-pi;
end
