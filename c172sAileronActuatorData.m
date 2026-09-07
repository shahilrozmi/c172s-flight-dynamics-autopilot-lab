function actuator = c172sAileronActuatorData()
%C172SAILERONACTUATORDATA Roll-channel actuator/sensor assumption ledger.
%
%   Certified aircraft geometry is kept separate from provisional control-
%   system assumptions. Servo dynamics, automatic authority, sensor data,
%   anti-windup and noise cases are not identified C172S or Garmin values.

actuator.version = 'Aileron Actuator and Roll-Sensor Protection V1';
actuator.integratedLoopVersion = 'Roll-Attitude Hold V1';
actuator.artifactRevision = 'AILERON-V1-AUDIT-MATLAB-R1';
actuator.requiredPlantVersion = 'Lateral-Directional Plant V0.1';
actuator.requiredYawDamperRevision = 'YD-V1-MATLAB-R2';
actuator.requiredRudderAuditRevision = 'YD-V2-AUDIT-MATLAB-R1';

%% Certified aileron travel and conservative equivalent hard stop

% FAA TCDS 3A12 Rev. 88 lists aileron movement as 20 deg up and
% 14 deg down. For either roll direction, one surface moves down; therefore
% the project's single equivalent deltaA input uses the smaller 14-deg
% magnitude as a conservative symmetric total-aileron hard stop.
actuator.physicalTravel.trailingEdgeUp_deg = 20.0;
actuator.physicalTravel.trailingEdgeDown_deg = 14.0;
actuator.physicalTravel.equivalentMagnitude_deg = 14.0;
actuator.physicalTravel.lowerLimit_rad = ...
    -deg2rad(actuator.physicalTravel.equivalentMagnitude_deg);
actuator.physicalTravel.upperLimit_rad = ...
    deg2rad(actuator.physicalTravel.equivalentMagnitude_deg);
actuator.physicalTravel.source = ...
    'FAA TCDS 3A12 Rev. 88, Control Surface Movements';
actuator.physicalTravel.classification = ...
    'certificated control-surface movement';

%% Provisional automatic-aileron authority

% Five degrees is 35.7 percent of the conservative 14-deg equivalent
% physical magnitude. It is a project protection limit, not a published
% C172S autopilot surface-authority limit.
actuator.authority.rightWingDown_deg = 5.0;
actuator.authority.leftWingDown_deg = 5.0;
actuator.authority.lowerLimit_rad = ...
    -deg2rad(actuator.authority.leftWingDown_deg);
actuator.authority.upperLimit_rad = ...
    deg2rad(actuator.authority.rightWingDown_deg);
actuator.authority.cases_deg = [4.0,5.0,6.0];
actuator.authority.selectedFractionOfEquivalentTravel = ...
    actuator.authority.rightWingDown_deg/ ...
    actuator.physicalTravel.equivalentMagnitude_deg;

%% Provisional aileron-servo dynamics

actuator.dynamics.timeConstant_s = 0.050;
actuator.dynamics.timeConstantCases_s = [0.035,0.050,0.075];
actuator.dynamics.rateLimit_degps = 30.0;
actuator.dynamics.rateLimitCases_degps = [20.0,30.0,40.0];

%% Provisional roll-sensor and processing model

% p and phi use the same first-order filter and common processing delay in
% this baseline. Pure delay is retained as exp(-s*Td) during the frequency-
% response audit rather than being hidden in the filter time constant.
actuator.sensor.filterTimeConstant_s = 0.020;
actuator.sensor.filterTimeConstantCases_s = [0.010,0.020,0.050];
actuator.sensor.nominalDelay_s = 0.010;
actuator.sensor.delayCases_s = [0.000,0.010,0.020,0.040,0.060,0.080];
actuator.sensor.maximumAcceptedDelay_s = 0.040;
actuator.sensor.rollRateNoiseRms_degps = 0.050;
actuator.sensor.bankAngleNoiseRms_deg = 0.050;
actuator.sensor.noiseStressMultiplier = 2.0;

%% Integral anti-windup

actuator.antiWindup.trackingTimeConstant_s = 0.50;
actuator.antiWindup.law = [ ...
    'xiDot = ePhi + (deltaATrack-deltaAUnsat)/(Ki*Tt), where ', ...
    'deltaATrack = deltaAActual + tauA*limitedRate'];

%% Protection, delay and noise gates

actuator.requirements.minimumBalancedDiskPhaseMargin_deg = 60.0;
actuator.requirements.minimumBalancedDiskGainMargin_dB = 12.0;
actuator.requirements.maximumOperationalActuatorRate_degps = 10.0;
actuator.requirements.maximumReleasedAileron_deg = 0.01;
% The stress case injects both 0.100-deg/s RMS roll-rate noise and
% 0.100-deg RMS bank-angle noise simultaneously. The 0.035-deg RMS gate is
% still less than one percent of the selected automatic authority.
actuator.requirements.maximumNoiseActuatorRms_deg = 0.035;
actuator.requirements.maximumNoiseActuatorPeak_deg = 0.050;

%% Nonlinear logic-stress definition

% A 30-deg initial bank lies outside the selected +/-20-deg command
% envelope and is used only to exercise position/rate limiting and
% anti-windup. It is not a Plant V0.1 aerodynamic-envelope claim.
actuator.stress.initialBank_deg = 30.0;
actuator.stress.timeStep_s = 0.001;
actuator.stress.stopTime_s = 40.0;
actuator.stress.recoveryThreshold_deg = 1.0;
actuator.stress.maximumRecoveryTime_s = 10.0;
actuator.stress.maximumFinalBank_deg = 0.10;
actuator.stress.maximumAntiWindupIntegratorRatio = 0.80;

actuator.noise.timeStep_s = 0.001;
actuator.noise.stopTime_s = 30.0;
actuator.noise.discardTime_s = 5.0;
actuator.noise.rollRateFrequency_radps = 50.0;
actuator.noise.bankAngleFrequency_radps = 5.0;

%% Evidence and limitation ledger

actuator.references.faaTravel = [ ...
    'FAA Dynamic Regulatory System, TCDS 3A12 Rev. 88: ailerons ', ...
    '20 deg up and 14 deg down.'];
actuator.references.servoArchitecture = [ ...
    'Garmin certified-autopilot material establishes a roll-servo/flight-', ...
    'director architecture but does not publish C172S surface authority, ', ...
    'surface-rate limit or identified servo time constant.'];

actuator.assumptions = { ...
    'Automatic aileron authority of +/-5 deg is a project selection.'; ...
    'The 0.050-s servo time constant is a project assumption.'; ...
    'The 30-deg/s servo rate is a project assumption.'; ...
    'The 0.020-s sensor filter and 0.010-s delay are assumptions.'; ...
    'The anti-windup and deterministic noise definitions are assumptions.'};

actuator.limitations = [ ...
    'Valid only for the project equivalent-aileron input and the frozen ', ...
    'single-trim Plant V0.1/Yaw Damper V2 combination. Differential ', ...
    'linkage geometry, cable friction/elasticity, servo torque and clutch, ', ...
    'backlash, AHRS bias/quantization, failures and full-envelope behavior ', ...
    'remain outside this audit. This is not approved aircraft data.'];

end
