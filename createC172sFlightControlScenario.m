function scenario = createC172sFlightControlScenario(preset,options,model)
%CREATEC172SFLIGHTCONTROLSCENARIO Build a bounded laboratory scenario.
%
%   SCENARIO = CREATEC172SFLIGHTCONTROLSCENARIO(PRESET) creates one of the
%   named laboratory presets. PRESET may also be 'custom'.
%
%   SCENARIO = CREATEC172SFLIGHTCONTROLSCENARIO(PRESET,OPTIONS) overrides
%   preset fields with a scalar structure. Public option fields are:
%     duration_s, engage, engageTime_s, lateralMode, verticalMode,
%     headingChange_deg, altitudeChange_ft, windStart_s, headwind_mps,
%     crosswindFromRight_mps, gustStart_s, gustDuration_s,
%     upwardGust_mps, eventType and eventTime_s.
%   ROLL and PITCH are capture modes: the frozen Mode Manager captures the
%   current bank and pitch at mode entry. Only HEADING and ALTITUDE expose
%   selectable references in the released 19-column request contract.

if nargin < 1 || isempty(preset)
    preset = 'quick-demo';
end
if nargin < 2 || isempty(options)
    options = struct();
end
if nargin < 3 || isempty(model)
    model = c172sNonlinearAutopilotClosedLoopData();
end
if isstring(preset) && isscalar(preset)
    preset = char(preset);
end
assert(ischar(preset) && isrow(preset), ...
    'createC172sFlightControlScenario:InvalidPreset', ...
    'preset must be a character vector or scalar string.');
assert(isstruct(options) && isscalar(options), ...
    'createC172sFlightControlScenario:InvalidOptions', ...
    'options must be a scalar structure.');

lab = c172sFlightControlLabData(model);
key = lower(strtrim(preset));
configuration = presetConfiguration(key,lab);
configuration = mergeOptions(configuration,options);
configuration = validateConfiguration(configuration,lab);

dt = model.execution.fixedStep_s;
configuration.duration_s = round(configuration.duration_s/dt)*dt;
time = (0:dt:configuration.duration_s).';
n = numel(time);
request = zeros(n,model.input.requestCount);
request(:,4:7) = 1;
request(:,18) = mod(lab.trim.heading_deg+ ...
    configuration.headingChange_deg,360);
request(:,19) = lab.trim.altitude_ft+configuration.altitudeChange_ft;

if configuration.engage
    engageIndex = nearestIndex(time,configuration.engageTime_s);
    request(engageIndex,1) = 1;
    request(engageIndex,10) = lateralCode( ...
        configuration.lateralMode,model);
    request(engageIndex,11) = verticalCode( ...
        configuration.verticalMode,model);
end

wind = zeros(n,3);
windActive = time >= configuration.windStart_s;
% Positive headwind means air moving toward the aircraft from ahead.
wind(windActive,1) = -configuration.headwind_mps;
% Positive crosswind-from-right means air moving from east toward west.
wind(windActive,2) = -configuration.crosswindFromRight_mps;

gust = zeros(n,3);
gustActive = time >= configuration.gustStart_s & ...
    time <= configuration.gustStart_s+configuration.gustDuration_s;
% NED down is positive, so an upward air-mass gust is negative NED down.
gust(gustActive,3) = -configuration.upwardGust_mps;

eventIndex = nearestIndex(time,configuration.eventTime_s);
switch configuration.eventType
    case 'NONE'
    case 'HEADING-LOSS'
        request(eventIndex:end,6) = 0;
    case 'ALTITUDE-LOSS'
        request(eventIndex:end,7) = 0;
    case 'DISCONNECT'
        request(eventIndex,2) = 1;
    case 'OVERRIDE'
        request(eventIndex,3) = 1;
    case 'AILERON-SATURATION'
        request(eventIndex:end,8) = 1;
    case 'ELEVATOR-SATURATION'
        request(eventIndex:end,9) = 1;
    otherwise
        error('createC172sFlightControlScenario:InvalidEvent', ...
            'Unsupported event type %s.',configuration.eventType);
end

scenario.name = scenarioName(key,configuration);
scenario.preset = key;
scenario.time_s = time;
scenario.request = request;
scenario.steadyWindNED_mps = wind;
scenario.gustNED_mps = gust;
scenario.configuration = configuration;
scenario.selectedHeading_deg = request(1,18);
scenario.selectedAltitude_ft = request(1,19);
scenario.expected = expectedOutcome(configuration,model);
scenario.labArtifactRevision = lab.artifactRevision;
scenario.backendArtifactRevision = model.artifactRevision;
end

function configuration = presetConfiguration(key,lab)
configuration = lab.defaults;
switch key
    case {'quick-demo','custom'}
    case 'roll-pitch-capture'
        configuration.duration_s = 10;
        configuration.lateralMode = 'ROLL';
        configuration.verticalMode = 'PITCH';
        configuration.headingChange_deg = 0;
        configuration.altitudeChange_ft = 0;
    case 'wind-gust'
        configuration.duration_s = 30;
        configuration.lateralMode = 'ROLL';
        configuration.verticalMode = 'PITCH';
        configuration.headingChange_deg = 0;
        configuration.altitudeChange_ft = 0;
        configuration.crosswindFromRight_mps = 2;
        configuration.upwardGust_mps = 0.5;
    case 'heading-fallback'
        configuration.duration_s = 20;
        configuration.eventType = 'HEADING-LOSS';
    case 'altitude-fallback'
        configuration.duration_s = 20;
        configuration.eventType = 'ALTITUDE-LOSS';
    case 'pilot-disconnect'
        configuration.duration_s = 15;
        configuration.eventType = 'DISCONNECT';
    case 'pilot-override'
        configuration.duration_s = 15;
        configuration.eventType = 'OVERRIDE';
    case 'autopilot-off'
        configuration.engage = false;
    otherwise
        error('createC172sFlightControlScenario:UnknownPreset', ...
            'Unknown preset "%s".',key);
end
end

function output = mergeOptions(base,override)
output = base;
allowed = fieldnames(base);
provided = fieldnames(override);
unknown = setdiff(provided,allowed);
assert(isempty(unknown), ...
    'createC172sFlightControlScenario:UnknownOption', ...
    'Unknown option field(s): %s',strjoin(unknown,', '));
for k = 1:numel(provided)
    output.(provided{k}) = override.(provided{k});
end
end

function c = validateConfiguration(c,lab)
numericFields = { ...
    'duration_s','engageTime_s','headingChange_deg','altitudeChange_ft', ...
    'windStart_s','headwind_mps','crosswindFromRight_mps', ...
    'gustStart_s','gustDuration_s','upwardGust_mps','eventTime_s'};
for k = 1:numel(numericFields)
    value = c.(numericFields{k});
    assert(isnumeric(value) && isreal(value) && isscalar(value) && ...
        isfinite(value), ...
        'createC172sFlightControlScenario:InvalidNumericOption', ...
        '%s must be a finite real scalar.',numericFields{k});
end
assert((islogical(c.engage) || isnumeric(c.engage)) && ...
    isscalar(c.engage) && any(double(c.engage) == [0,1]), ...
    'createC172sFlightControlScenario:InvalidEngage', ...
    'engage must be a scalar logical value.');
c.engage = logical(c.engage);

c.lateralMode = normalizedToken(c.lateralMode,'lateralMode');
c.verticalMode = normalizedToken(c.verticalMode,'verticalMode');
c.eventType = normalizedToken(c.eventType,'eventType');
assert(any(strcmp(c.lateralMode,{'OFF','ROLL','HEADING'})), ...
    'createC172sFlightControlScenario:InvalidLateralMode', ...
    'lateralMode must be OFF, ROLL or HEADING.');
assert(any(strcmp(c.verticalMode,{'OFF','PITCH','ALTITUDE'})), ...
    'createC172sFlightControlScenario:InvalidVerticalMode', ...
    'verticalMode must be OFF, PITCH or ALTITUDE.');
if c.engage
    assert(~strcmp(c.lateralMode,'OFF') && ...
        ~strcmp(c.verticalMode,'OFF'), ...
        'createC172sFlightControlScenario:InvalidEngagedMode', ...
        'An engaged scenario requires active lateral and vertical modes.');
end
validEvents = {'NONE','HEADING-LOSS','ALTITUDE-LOSS','DISCONNECT', ...
    'OVERRIDE','AILERON-SATURATION','ELEVATOR-SATURATION'};
assert(any(strcmp(c.eventType,validEvents)), ...
    'createC172sFlightControlScenario:InvalidEvent', ...
    'eventType is not supported.');

requireRange(c.duration_s,lab.boundary.duration_s,'duration_s');
requireRange(c.engageTime_s,lab.boundary.engageTime_s,'engageTime_s');
requireRange(c.headingChange_deg,lab.boundary.headingChange_deg, ...
    'headingChange_deg');
requireRange(c.altitudeChange_ft,lab.boundary.altitudeChange_ft, ...
    'altitudeChange_ft');
requireRange(c.headwind_mps,lab.boundary.headwind_mps,'headwind_mps');
requireRange(c.crosswindFromRight_mps, ...
    lab.boundary.crosswindFromRight_mps,'crosswindFromRight_mps');
requireRange(c.upwardGust_mps,lab.boundary.upwardGust_mps, ...
    'upwardGust_mps');
requireRange(c.gustDuration_s,lab.boundary.gustDuration_s, ...
    'gustDuration_s');
assert(c.engageTime_s < c.duration_s, ...
    'createC172sFlightControlScenario:InvalidEngageTime', ...
    'engageTime_s must occur before the scenario ends.');
assert(c.windStart_s >= 0 && c.windStart_s <= c.duration_s, ...
    'createC172sFlightControlScenario:InvalidWindTime', ...
    'windStart_s must lie inside the scenario.');
assert(c.gustStart_s >= 0 && ...
    c.gustStart_s+c.gustDuration_s <= c.duration_s, ...
    'createC172sFlightControlScenario:InvalidGustTime', ...
    'The complete gust interval must lie inside the scenario.');
if ~strcmp(c.eventType,'NONE')
    assert(c.eventTime_s > c.engageTime_s && ...
        c.eventTime_s < c.duration_s, ...
        'createC172sFlightControlScenario:InvalidEventTime', ...
        'eventTime_s must occur after engagement and before scenario end.');
end
if strcmp(c.eventType,'HEADING-LOSS')
    assert(strcmp(c.lateralMode,'HEADING'), ...
        'createC172sFlightControlScenario:IncompatibleEvent', ...
        'HEADING-LOSS requires HEADING mode.');
elseif strcmp(c.eventType,'ALTITUDE-LOSS')
    assert(strcmp(c.verticalMode,'ALTITUDE'), ...
        'createC172sFlightControlScenario:IncompatibleEvent', ...
        'ALTITUDE-LOSS requires ALTITUDE mode.');
end
if ~c.engage
    assert(strcmp(c.eventType,'NONE'), ...
        'createC172sFlightControlScenario:IncompatibleEvent', ...
        'An autopilot event cannot be requested when engage is false.');
end
end

function token = normalizedToken(value,name)
if isstring(value) && isscalar(value)
    value = char(value);
end
assert(ischar(value) && isrow(value), ...
    'createC172sFlightControlScenario:InvalidTextOption', ...
    '%s must be a character vector or scalar string.',name);
token = upper(strtrim(value));
end

function requireRange(value,bounds,name)
assert(value >= bounds(1) && value <= bounds(2), ...
    'createC172sFlightControlScenario:OutsideBoundary', ...
    '%s must lie in [%g,%g].',name,bounds(1),bounds(2));
end

function code = lateralCode(name,model)
code = model.dependencies.manager.codes.lateral.(name);
end

function code = verticalCode(name,model)
code = model.dependencies.manager.codes.vertical.(name);
end

function index = nearestIndex(time,value)
[~,index] = min(abs(time-value));
end

function name = scenarioName(key,c)
switch key
    case 'quick-demo'
        name = 'Quick heading and altitude capture';
    case 'custom'
        name = 'Custom nonlinear-autopilot mission';
    case 'roll-pitch-capture'
        name = 'Bumpless ROLL/PITCH engagement at trim';
    case 'wind-gust'
        name = 'Wind and vertical-gust rejection';
    case 'heading-fallback'
        name = 'Heading-source loss and ROLL fallback';
    case 'altitude-fallback'
        name = 'Altitude-source loss and PITCH fallback';
    case 'pilot-disconnect'
        name = 'Pilot disconnect and surface release';
    case 'pilot-override'
        name = 'Pilot override priority';
    case 'autopilot-off'
        name = 'Autopilot-off comparison case';
    otherwise
        name = sprintf('%s / %s-%s',key,c.lateralMode,c.verticalMode);
end
end

function expected = expectedOutcome(c,model)
manager = model.dependencies.manager;
expected.finalEngaged = c.engage;
expected.finalLateralMode = lateralCode(c.lateralMode,model);
expected.finalVerticalMode = verticalCode(c.verticalMode,model);
expected.requiredEvent = manager.codes.event.NONE;
switch c.eventType
    case 'HEADING-LOSS'
        expected.finalLateralMode = manager.codes.lateral.ROLL;
        expected.requiredEvent = manager.codes.event.HEADING_FALLBACK;
    case 'ALTITUDE-LOSS'
        expected.finalVerticalMode = manager.codes.vertical.PITCH;
        expected.requiredEvent = manager.codes.event.ALTITUDE_FALLBACK;
    case 'DISCONNECT'
        expected.finalEngaged = false;
        expected.finalLateralMode = manager.codes.lateral.OFF;
        expected.finalVerticalMode = manager.codes.vertical.OFF;
        expected.requiredEvent = manager.codes.event.DISCONNECT_COMMAND;
    case 'OVERRIDE'
        expected.finalEngaged = false;
        expected.finalLateralMode = manager.codes.lateral.OFF;
        expected.finalVerticalMode = manager.codes.vertical.OFF;
        expected.requiredEvent = manager.codes.event.DISCONNECT_OVERRIDE;
    case {'AILERON-SATURATION','ELEVATOR-SATURATION'}
        expected.finalEngaged = false;
        expected.finalLateralMode = manager.codes.lateral.OFF;
        expected.finalVerticalMode = manager.codes.vertical.OFF;
        expected.requiredEvent = manager.codes.event.DISCONNECT_SATURATION;
end
if ~c.engage
    expected.finalEngaged = false;
    expected.finalLateralMode = manager.codes.lateral.OFF;
    expected.finalVerticalMode = manager.codes.vertical.OFF;
    expected.requiredEvent = manager.codes.event.NONE;
end
end
