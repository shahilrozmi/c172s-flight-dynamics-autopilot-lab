function manager = c172sAutopilotModeManagerData()
%C172SAUTOPILOTMODEMANAGERDATA Supervisory-mode logic definition and audit ledger.
%
%   This file defines the deterministic supervisory logic that sits above
%   the already frozen longitudinal and lateral C172S project controllers.
%   It does not retune or replace any control law. All timing thresholds,
%   abnormal-attitude thresholds and fallback policies below are explicit
%   project assumptions unless a source classification says otherwise.

manager.version = 'Autopilot Mode Manager V1';
manager.artifactRevision = ...
    'AUTOPILOT-MODE-MANAGER-V1-AUDIT-MATLAB-R1';
manager.architecture = [ ...
    'Deterministic two-axis supervisor with one active lateral mode and ', ...
    'one active vertical mode, capture-on-engage references, explicit ', ...
    'fallbacks, transfer tracking handshakes and prioritized disconnects'];

%% Frozen dependency ledger

manager.required.longitudinalPlant = 'Longitudinal Plant V1';
manager.required.pitchRateDamper = 'Pitch-Rate Damper V1';
manager.required.pitchAttitudeController = 'Pitch-Attitude Hold V1';
manager.required.pitchLoop = 'Pitch-Attitude Hold V2';
manager.required.altitudeLoop = 'Altitude Hold V1';
manager.required.lateralPlant = 'Lateral-Directional Plant V0.1';
manager.required.yawController = 'YD-V1-MATLAB-R2';
manager.required.rudderAudit = 'YD-V2-AUDIT-MATLAB-R1';
manager.required.rollController = 'RAH-V1-MATLAB-R1';
manager.required.aileronAudit = 'AILERON-V1-AUDIT-MATLAB-R1';
manager.required.headingController = 'HH-V1-MATLAB-R1';
manager.required.headingSensorAudit = ...
    'HEADING-SENSOR-V1-AUDIT-MATLAB-R1';

%% Numeric mode codes for later Simulink/Stateflow integration

manager.codes.lateral.OFF = uint8(0);
manager.codes.lateral.ROLL = uint8(1);
manager.codes.lateral.HEADING = uint8(2);

manager.codes.vertical.OFF = uint8(0);
manager.codes.vertical.PITCH = uint8(1);
manager.codes.vertical.ALTITUDE = uint8(2);

manager.codes.event.NONE = uint8(0);
manager.codes.event.ENGAGED = uint8(1);
manager.codes.event.MODE_CHANGED = uint8(2);
manager.codes.event.MODE_REJECTED = uint8(3);
manager.codes.event.ENGAGE_REJECTED = uint8(4);
manager.codes.event.HEADING_FALLBACK = uint8(5);
manager.codes.event.ALTITUDE_FALLBACK = uint8(6);
manager.codes.event.DISCONNECT_COMMAND = uint8(7);
manager.codes.event.DISCONNECT_OVERRIDE = uint8(8);
manager.codes.event.DISCONNECT_CORE_SENSOR = uint8(9);
manager.codes.event.DISCONNECT_SATURATION = uint8(10);
manager.codes.event.DISCONNECT_ATTITUDE = uint8(11);

%% Default engagement and capture policy

% The project presently has no vertical-speed mode. Therefore the manager
% uses the frozen pitch-attitude loop as its default vertical mode. This is
% an explicit project substitution and is not claimed to reproduce a GFC
% 700 default-mode implementation.
manager.defaultLateralMode = manager.codes.lateral.ROLL;
manager.defaultVerticalMode = manager.codes.vertical.PITCH;

% Consistent with the roll-mode architecture already cited by RAH V1,
% engagement below six degrees commands wings level. Above six degrees the
% current bank is captured, subject to the frozen +/-20-deg bank authority.
manager.capture.rollWingsLevelThreshold_deg = 6.0;
manager.capture.bankCommandLimit_deg = 20.0;

% The pitch-capture bound is a conservative project supervisory limit. It
% matches the existing Altitude Hold V1 pitch-command authority so a mode
% transfer cannot silently demand a larger attitude step.
manager.capture.pitchCommandLimit_deg = 3.0;
manager.capture.headingRange_deg = [0.0,360.0];
manager.capture.headingErrorRange_deg = [-180.0,180.0];

manager.engagement.requiresTrimStable = true;
manager.engagement.requiresCoreSensors = true;
manager.engagement.requiresNoPilotOverride = true;
manager.engagement.requiresUnsaturatedActuators = true;

%% Mode-loss and disconnect policy

% Loss of only an outer-loop source degrades the affected axis to its
% inner attitude mode. Loss of core attitude/rate sensing disconnects the
% complete automatic-flight system after a short confirmation interval.
manager.protection.outerSensorFallbackDelay_s = 0.10;
manager.protection.coreSensorDisconnectDelay_s = 0.10;
manager.protection.sustainedSaturationDisconnectDelay_s = 2.00;

% These are logic-stress thresholds, not a validated C172S flight envelope.
manager.protection.maximumAbsoluteBank_deg = 30.0;
manager.protection.maximumAbsolutePitch_deg = 10.0;

% Highest priority first. A lower-priority request must never mask a
% disconnect condition occurring in the same sample.
manager.protection.priority = { ...
    'pilot disconnect request'; ...
    'pilot control override'; ...
    'abnormal bank or pitch attitude'; ...
    'confirmed core-sensor invalidity'; ...
    'confirmed sustained actuator saturation'; ...
    'outer-loop source fallback'; ...
    'mode selection or reference update'};

%% Bumpless-transfer interface contract

manager.transfer.captureActualAttitudeOnInnerModeEntry = true;
manager.transfer.requestIntegratorTrackingOnEngagement = true;
manager.transfer.requestIntegratorTrackingOnModeChange = true;
manager.transfer.trackingPulseSamples = 1;
manager.transfer.contract = [ ...
    'The manager emits one-sample roll/pitch integrator-tracking request ', ...
    'pulses whenever the corresponding inner-loop reference source is ', ...
    'engaged or changed. The later executable integration must preload or ', ...
    'track the frozen inner-loop integrator to its actual actuator path; ', ...
    'this audit verifies the supervisory handshake, not actuator torque.'];

%% Deterministic audit definition and gates

manager.validation.sampleTime_s = 0.01;
manager.validation.minimumAuditCases = 21;
manager.validation.maximumReferenceCaptureError_deg = 1.0e-12;
manager.validation.maximumHeadingNormalizationError_deg = 1.0e-12;
manager.validation.requireRepeatableTrace = true;

manager.requirements.defaultModesCorrect = true;
manager.requirements.modePairsExclusive = true;
manager.requirements.disconnectPriorityCorrect = true;
manager.requirements.outerSensorFallbackCorrect = true;
manager.requirements.transientFaultsRejected = true;
manager.requirements.sustainedFaultsProtected = true;
manager.requirements.transferHandshakePresent = true;
manager.requirements.inactiveOutputsReleased = true;

%% Evidence, assumptions and limitations

manager.references.operatingProcedure = [ ...
    'C172S/G1000 operating guidance establishes stabilized, trimmed ', ...
    'engagement, an A/P DISC control, an aural disconnect indication and ', ...
    'an over-powerable autopilot architecture.'];
manager.references.lateralArchitecture = [ ...
    'The existing project Heading Hold and Roll-Attitude Hold source ', ...
    'audits use Garmin Cessna Nav III mode-architecture references.'];
manager.references.classification = [ ...
    'Only the high-level operating architecture is source-informed. The ', ...
    'state machine, thresholds, timers and transfer policy are project ', ...
    'engineering selections.'];

manager.assumptions = { ...
    'ROLL plus PITCH are the project default engagement modes.'; ...
    'A trimStable Boolean represents the completed pilot stabilization check.'; ...
    'A pilotOverride Boolean represents force/torque override detection.'; ...
    'Outer-source loss falls back to the corresponding attitude mode.'; ...
    'Core-sensor loss disconnects both axes after 0.10 s.'; ...
    'Either sustained actuator-saturation flag disconnects after 2.00 s.'; ...
    'The 30-deg bank and 10-deg pitch thresholds are logic-stress values.'; ...
    'The later executable implementation will supply annunciation and tone.'};

manager.limitations = [ ...
    'This V1 audit validates deterministic supervisory decisions only. ', ...
    'It does not model switches, servo clutch force, trim runaway, circuit ', ...
    'breakers, electrical power, redundant computers, missed execution ', ...
    'deadlines, sensor voting, terrain, stall/overspeed protection, GPS ', ...
    'navigation, approach modes, flight-director-only operation or certified ', ...
    'human-factors behavior. It is limited to the frozen single-trim project ', ...
    'plants and declared assumptions and is not aircraft approval.'];

end
