function [next,output] = c172sAutopilotModeManagerStep( ...
    state,input,manager,dt)
%C172SAUTOPILOTMODEMANAGERSTEP One deterministic supervisor update.
%
%   [NEXT,OUTPUT] = C172SAUTOPILOTMODEMANAGERSTEP(STATE,INPUT,MANAGER,DT)
%   advances the explicit mode-manager state by one fixed sample. Pass []
%   as STATE to initialize the manager. No persistent or global state is
%   used, which makes the logic independently reproducible and testable.
%
%   Inputs are scalar fields. Mode requests are numeric codes from MANAGER;
%   a request code of zero means no new request while engaged. OFF is only
%   entered by disengagement; it is not used as an in-flight mode request.

validateStepArguments(input,manager,dt);

if isempty(state)
    state = initialState(input,manager);
else
    validateState(state);
end

next = state;
next.lastEventCode = manager.codes.event.NONE;
next.lastEventText = 'NONE';
next.rollIntegratorTrackRequest = false;
next.pitchIntegratorTrackRequest = false;
next.disconnectToneRequest = false;

% Timers are updated before decisions. Exact thresholds therefore mean
% N consecutive invalid/saturated samples when threshold/dt = N.
next.coreInvalidTime_s = updateTimer(~input.coreSensorsValid, ...
    state.coreInvalidTime_s,dt);
next.headingInvalidTime_s = updateTimer(~input.headingValid, ...
    state.headingInvalidTime_s,dt);
next.altitudeInvalidTime_s = updateTimer(~input.altitudeValid, ...
    state.altitudeInvalidTime_s,dt);
next.aileronSaturationTime_s = updateTimer( ...
    input.aileronSaturated,state.aileronSaturationTime_s,dt);
next.elevatorSaturationTime_s = updateTimer( ...
    input.elevatorSaturated,state.elevatorSaturationTime_s,dt);

% Disconnect protection owns priority over engagement and mode requests.
if state.engaged
    if input.disconnectRequest
        next = disconnect(next,manager, ...
            manager.codes.event.DISCONNECT_COMMAND,'PILOT_DISCONNECT');
    elseif input.pilotOverride
        next = disconnect(next,manager, ...
            manager.codes.event.DISCONNECT_OVERRIDE,'PILOT_OVERRIDE');
    elseif abs(input.bank_deg) > ...
            manager.protection.maximumAbsoluteBank_deg || ...
            abs(input.pitch_deg) > ...
            manager.protection.maximumAbsolutePitch_deg
        next = disconnect(next,manager, ...
            manager.codes.event.DISCONNECT_ATTITUDE,'ABNORMAL_ATTITUDE');
    elseif next.coreInvalidTime_s + eps >= ...
            manager.protection.coreSensorDisconnectDelay_s
        next = disconnect(next,manager, ...
            manager.codes.event.DISCONNECT_CORE_SENSOR, ...
            'CORE_SENSOR_INVALID');
    elseif max(next.aileronSaturationTime_s, ...
            next.elevatorSaturationTime_s) + eps >= ...
            manager.protection.sustainedSaturationDisconnectDelay_s
        next = disconnect(next,manager, ...
            manager.codes.event.DISCONNECT_SATURATION, ...
            'SUSTAINED_ACTUATOR_SATURATION');
    end
end

% When disengaged, either remain safely released or accept a valid engage.
if ~next.engaged
    if ~state.engaged && input.engageRequest
        if engagementPermitted(input,manager)
            next = engage(next,input,manager);
        else
            next.lastEventCode = manager.codes.event.ENGAGE_REJECTED;
            next.lastEventText = 'ENGAGE_REJECTED';
        end
    end
    output = formOutput(next,manager);
    return;
end

% Confirmed outer-loop source loss degrades only the affected axis. The
% attitude captured at fallback makes the new inner-loop error initially
% zero, before the inner controller's explicit tracking action is applied.
if next.lateralMode == manager.codes.lateral.HEADING && ...
        next.headingInvalidTime_s + eps >= ...
        manager.protection.outerSensorFallbackDelay_s
    next.lateralMode = manager.codes.lateral.ROLL;
    next.bankReference_deg = captureBank(input.bank_deg,manager);
    next.rollIntegratorTrackRequest = true;
    next.lastEventCode = manager.codes.event.HEADING_FALLBACK;
    next.lastEventText = 'HEADING_TO_ROLL_FALLBACK';
end

if next.verticalMode == manager.codes.vertical.ALTITUDE && ...
        next.altitudeInvalidTime_s + eps >= ...
        manager.protection.outerSensorFallbackDelay_s
    next.verticalMode = manager.codes.vertical.PITCH;
    next.pitchReference_deg = capturePitch(input.pitch_deg,manager);
    next.pitchIntegratorTrackRequest = true;
    next.lastEventCode = manager.codes.event.ALTITUDE_FALLBACK;
    next.lastEventText = 'ALTITUDE_TO_PITCH_FALLBACK';
end

% A valid explicit mode request follows fallback processing. Invalid outer
% mode requests are rejected without disturbing the currently safe mode.
if input.lateralModeRequest ~= manager.codes.lateral.OFF
    if input.lateralModeRequest == manager.codes.lateral.ROLL
        next.lateralMode = manager.codes.lateral.ROLL;
        next.bankReference_deg = captureBank(input.bank_deg,manager);
        next.rollIntegratorTrackRequest = true;
        next.lastEventCode = manager.codes.event.MODE_CHANGED;
        next.lastEventText = 'LATERAL_MODE_ROLL';
    elseif input.lateralModeRequest == manager.codes.lateral.HEADING && ...
            input.headingValid
        next.lateralMode = manager.codes.lateral.HEADING;
        next.headingReference_deg = normalizeHeading( ...
            input.selectedHeading_deg);
        next.rollIntegratorTrackRequest = true;
        next.lastEventCode = manager.codes.event.MODE_CHANGED;
        next.lastEventText = 'LATERAL_MODE_HEADING';
    else
        next.lastEventCode = manager.codes.event.MODE_REJECTED;
        next.lastEventText = 'LATERAL_MODE_REJECTED';
    end
end

if input.verticalModeRequest ~= manager.codes.vertical.OFF
    if input.verticalModeRequest == manager.codes.vertical.PITCH
        next.verticalMode = manager.codes.vertical.PITCH;
        next.pitchReference_deg = capturePitch(input.pitch_deg,manager);
        next.pitchIntegratorTrackRequest = true;
        next.lastEventCode = manager.codes.event.MODE_CHANGED;
        next.lastEventText = 'VERTICAL_MODE_PITCH';
    elseif input.verticalModeRequest == ...
            manager.codes.vertical.ALTITUDE && input.altitudeValid
        next.verticalMode = manager.codes.vertical.ALTITUDE;
        next.altitudeReference_ft = input.selectedAltitude_ft;
        next.pitchIntegratorTrackRequest = true;
        next.lastEventCode = manager.codes.event.MODE_CHANGED;
        next.lastEventText = 'VERTICAL_MODE_ALTITUDE';
    else
        next.lastEventCode = manager.codes.event.MODE_REJECTED;
        next.lastEventText = 'VERTICAL_MODE_REJECTED';
    end
end

% Knob/reference updates are accepted only by their active outer mode.
if next.lateralMode == manager.codes.lateral.HEADING && ...
        input.updateHeadingReference && input.headingValid
    next.headingReference_deg = normalizeHeading( ...
        input.selectedHeading_deg);
end
if next.verticalMode == manager.codes.vertical.ALTITUDE && ...
        input.updateAltitudeReference && input.altitudeValid
    next.altitudeReference_ft = input.selectedAltitude_ft;
end

next.sequence = state.sequence + uint32(1);
output = formOutput(next,manager);
end

function state = initialState(input,manager)
state.engaged = false;
state.lateralMode = manager.codes.lateral.OFF;
state.verticalMode = manager.codes.vertical.OFF;
state.bankReference_deg = captureBank(input.bank_deg,manager);
state.pitchReference_deg = capturePitch(input.pitch_deg,manager);
state.headingReference_deg = normalizeHeading(input.heading_deg);
state.altitudeReference_ft = input.altitude_ft;
state.coreInvalidTime_s = 0.0;
state.headingInvalidTime_s = 0.0;
state.altitudeInvalidTime_s = 0.0;
state.aileronSaturationTime_s = 0.0;
state.elevatorSaturationTime_s = 0.0;
state.lastEventCode = manager.codes.event.NONE;
state.lastEventText = 'NONE';
state.disconnectReason = 'NONE';
state.rollIntegratorTrackRequest = false;
state.pitchIntegratorTrackRequest = false;
state.disconnectToneRequest = false;
state.sequence = uint32(0);
end

function permitted = engagementPermitted(input,manager)
permitted = true;
if manager.engagement.requiresTrimStable
    permitted = permitted && input.trimStable;
end
if manager.engagement.requiresCoreSensors
    permitted = permitted && input.coreSensorsValid;
end
if manager.engagement.requiresNoPilotOverride
    permitted = permitted && ~input.pilotOverride;
end
if manager.engagement.requiresUnsaturatedActuators
    permitted = permitted && ~input.aileronSaturated && ...
        ~input.elevatorSaturated;
end

if input.lateralModeRequest == manager.codes.lateral.HEADING
    permitted = permitted && input.headingValid;
end
if input.verticalModeRequest == manager.codes.vertical.ALTITUDE
    permitted = permitted && input.altitudeValid;
end
end

function state = engage(state,input,manager)
state.engaged = true;
state.disconnectReason = 'NONE';
state.disconnectToneRequest = false;
state.lastEventCode = manager.codes.event.ENGAGED;
state.lastEventText = 'ENGAGED';

if input.lateralModeRequest == manager.codes.lateral.HEADING
    state.lateralMode = manager.codes.lateral.HEADING;
    state.headingReference_deg = normalizeHeading( ...
        input.selectedHeading_deg);
else
    state.lateralMode = manager.defaultLateralMode;
    state.bankReference_deg = captureBank(input.bank_deg,manager);
end

if input.verticalModeRequest == manager.codes.vertical.ALTITUDE
    state.verticalMode = manager.codes.vertical.ALTITUDE;
    state.altitudeReference_ft = input.selectedAltitude_ft;
else
    state.verticalMode = manager.defaultVerticalMode;
    state.pitchReference_deg = capturePitch(input.pitch_deg,manager);
end

state.rollIntegratorTrackRequest = true;
state.pitchIntegratorTrackRequest = true;
state.sequence = state.sequence + uint32(1);
end

function state = disconnect(state,manager,eventCode,reason)
state.engaged = false;
state.lateralMode = manager.codes.lateral.OFF;
state.verticalMode = manager.codes.vertical.OFF;
state.lastEventCode = eventCode;
state.lastEventText = reason;
state.disconnectReason = reason;
state.rollIntegratorTrackRequest = false;
state.pitchIntegratorTrackRequest = false;
state.disconnectToneRequest = true;
state.sequence = state.sequence + uint32(1);
end

function output = formOutput(state,manager)
output.engaged = state.engaged;
output.lateralMode = state.lateralMode;
output.verticalMode = state.verticalMode;
output.rollModeEnable = state.engaged && ...
    state.lateralMode == manager.codes.lateral.ROLL;
output.headingModeEnable = state.engaged && ...
    state.lateralMode == manager.codes.lateral.HEADING;
output.pitchModeEnable = state.engaged && ...
    state.verticalMode == manager.codes.vertical.PITCH;
output.altitudeModeEnable = state.engaged && ...
    state.verticalMode == manager.codes.vertical.ALTITUDE;

output.bankCommand_deg = 0.0;
output.pitchCommand_deg = 0.0;
output.headingCommand_deg = state.headingReference_deg;
output.altitudeCommand_ft = state.altitudeReference_ft;
if output.rollModeEnable
    output.bankCommand_deg = state.bankReference_deg;
end
if output.pitchModeEnable
    output.pitchCommand_deg = state.pitchReference_deg;
end

output.rollIntegratorTrackRequest = ...
    state.rollIntegratorTrackRequest;
output.pitchIntegratorTrackRequest = ...
    state.pitchIntegratorTrackRequest;
output.disconnectToneRequest = state.disconnectToneRequest;
output.eventCode = state.lastEventCode;
output.eventText = state.lastEventText;
output.disconnectReason = state.disconnectReason;
end

function value = captureBank(bank_deg,manager)
if abs(bank_deg) < manager.capture.rollWingsLevelThreshold_deg
    value = 0.0;
else
    value = saturate(bank_deg, ...
        -manager.capture.bankCommandLimit_deg, ...
        manager.capture.bankCommandLimit_deg);
end
end

function value = capturePitch(pitch_deg,manager)
value = saturate(pitch_deg, ...
    -manager.capture.pitchCommandLimit_deg, ...
    manager.capture.pitchCommandLimit_deg);
end

function heading_deg = normalizeHeading(heading_deg)
heading_deg = mod(heading_deg,360.0);
if heading_deg < 0.0
    heading_deg = heading_deg + 360.0;
end
end

function value = saturate(value,lower,upper)
value = min(max(value,lower),upper);
end

function timer = updateTimer(condition,previous,dt)
if condition
    timer = previous + dt;
else
    timer = 0.0;
end
end

function validateStepArguments(input,manager,dt)
required = { ...
    'engageRequest','disconnectRequest','pilotOverride','trimStable', ...
    'coreSensorsValid','headingValid','altitudeValid', ...
    'aileronSaturated','elevatorSaturated','lateralModeRequest', ...
    'verticalModeRequest','updateHeadingReference', ...
    'updateAltitudeReference','bank_deg','pitch_deg','heading_deg', ...
    'altitude_ft','selectedHeading_deg','selectedAltitude_ft'};
for k = 1:numel(required)
    assert(isfield(input,required{k}), ...
        'c172sAutopilotModeManagerStep:MissingInput', ...
        'Missing input field: %s.',required{k});
end

assert(isstruct(manager) && isfield(manager,'artifactRevision'), ...
    'c172sAutopilotModeManagerStep:InvalidManager', ...
    'The mode-manager configuration is invalid.');
assert(isscalar(dt) && isfinite(dt) && dt > 0.0, ...
    'c172sAutopilotModeManagerStep:InvalidTimeStep', ...
    'dt must be a finite positive scalar.');

numericFields = {'bank_deg','pitch_deg','heading_deg','altitude_ft', ...
    'selectedHeading_deg','selectedAltitude_ft'};
for k = 1:numel(numericFields)
    value = input.(numericFields{k});
    assert(isscalar(value) && isfinite(value), ...
        'c172sAutopilotModeManagerStep:InvalidInput', ...
        '%s must be a finite scalar.',numericFields{k});
end

validLateral = input.lateralModeRequest == manager.codes.lateral.OFF || ...
    input.lateralModeRequest == manager.codes.lateral.ROLL || ...
    input.lateralModeRequest == manager.codes.lateral.HEADING;
validVertical = input.verticalModeRequest == manager.codes.vertical.OFF || ...
    input.verticalModeRequest == manager.codes.vertical.PITCH || ...
    input.verticalModeRequest == manager.codes.vertical.ALTITUDE;
assert(validLateral && validVertical, ...
    'c172sAutopilotModeManagerStep:InvalidModeRequest', ...
    'A mode request used an undefined numeric code.');
end

function validateState(state)
required = {'engaged','lateralMode','verticalMode','sequence', ...
    'coreInvalidTime_s','headingInvalidTime_s', ...
    'altitudeInvalidTime_s','aileronSaturationTime_s', ...
    'elevatorSaturationTime_s'};
for k = 1:numel(required)
    assert(isfield(state,required{k}), ...
        'c172sAutopilotModeManagerStep:InvalidState', ...
        'State is missing field: %s.',required{k});
end
end
