function actuator = c172sRudderActuatorData()
%C172SRUDDERACTUATORDATA Yaw Damper V2 actuator/sensor assumption ledger.
%
%   This file separates aircraft geometry, equipment-architecture evidence
%   and provisional project assumptions. The automatic-rudder authority,
%   actuator dynamics, sensor dynamics and delay/noise cases are not
%   identified C172S or Garmin parameters. They are transparent engineering
%   assumptions that must be replaced if installation-specific data become
%   available.

actuator.version = 'Rudder Actuator and Sensor Protection V1';
actuator.integratedLoopVersion = 'Yaw Damper V2';
actuator.artifactRevision = 'YD-V2-AUDIT-MATLAB-R1';
actuator.requiredYawDamperRevision = 'YD-V1-MATLAB-R2';
actuator.requiredPlantVersion = 'Lateral-Directional Plant V0.1';

%% Aircraft physical rudder travel

% FAA TCDS 3A12 Rev. 88 lists Model 172S rudder movement as 16 degrees
% right and 16 degrees left. This conservative certified value is used as
% the total-rudder hard stop in the project model.
actuator.physicalTravel.right_deg = 16.0;
actuator.physicalTravel.left_deg = 16.0;
actuator.physicalTravel.lowerLimit_rad = ...
    -deg2rad(actuator.physicalTravel.left_deg);
actuator.physicalTravel.upperLimit_rad = ...
    deg2rad(actuator.physicalTravel.right_deg);
actuator.physicalTravel.source = 'FAA TCDS 3A12 Rev. 88, Model 172S';
actuator.physicalTravel.classification = ...
    'certificated control-surface movement';

% The maintenance-manual values are retained as traceability metadata, not
% as permission to enlarge the certified model hard stop. The difference
% is caused by the specified measurement reference.
actuator.riggingTravel.parallelWaterLine_deg = 16 + 10/60;
actuator.riggingTravel.perpendicularHingeLine_deg = 17 + 44/60;
actuator.riggingTravel.source = [ ...
    'Cessna Model 172 Maintenance Manual, 6-10-00, Mar 1/2009'];
actuator.riggingTravel.classification = ...
    'manufacturer rigging-reference geometry';

%% Provisional automatic-rudder authority

% Five degrees is 31.25 percent of the conservative 16-degree aircraft
% travel and matches the project's protected-elevator authority precedent.
% It is a project design limit, not a published C172S autopilot limit.
actuator.authority.right_deg = 5.0;
actuator.authority.left_deg = 5.0;
actuator.authority.lowerLimit_rad = -deg2rad(actuator.authority.left_deg);
actuator.authority.upperLimit_rad = deg2rad(actuator.authority.right_deg);
actuator.authority.cases_deg = [3.0,5.0,7.0];
actuator.authority.selectedFractionOfPhysicalTravel = ...
    actuator.authority.right_deg/actuator.physicalTravel.right_deg;

%% Provisional rudder-servo dynamics

% The selected values are the center of declared sensitivity ranges. The
% time-constant cases enter the complete structured linear audit. Position
% and rate limits are assessed in nonlinear operational and stress cases.
actuator.dynamics.timeConstant_s = 0.050;
actuator.dynamics.timeConstantCases_s = [0.035,0.050,0.075];
actuator.dynamics.rateLimit_degps = 20.0;
actuator.dynamics.rateLimitCases_degps = [10.0,20.0,30.0];

%% Provisional yaw-rate sensor model

% First-order sensor filtering is retained in the V2 baseline. Pure delay
% is not hidden inside the filter: it is audited independently using the
% exact exp(-s*Td) frequency-response factor.
actuator.sensor.filterTimeConstant_s = 0.020;
actuator.sensor.filterTimeConstantCases_s = [0.010,0.020,0.050];
actuator.sensor.nominalDelay_s = 0.010;
actuator.sensor.delayCases_s = [0.000,0.010,0.020,0.040,0.060,0.080];
actuator.sensor.maximumAcceptedDelay_s = 0.060;
actuator.sensor.noiseRms_degps = 0.050;
actuator.sensor.noiseStressRms_degps = 0.100;
actuator.sensor.noiseSampleTime_s = 0.010;
actuator.sensor.noiseRandomSeed = 172;

%% Linear robustness gates

% Existing V1 Dutch-roll and disk-margin gates are preserved. The roll
% metric changes from a real-pole ratio to a matched-mode frequency ratio
% because actuator interaction may create a highly damped roll/servo pair.
actuator.requirements.minimumDutchDampingRatio = 0.35;
actuator.requirements.minimumDutchNaturalFrequency_radps = 0.40;
actuator.requirements.minimumDutchDecayRate_radps = 0.15;
actuator.requirements.minimumBalancedDiskPhaseMargin_deg = 60.0;
actuator.requirements.minimumBalancedDiskGainMargin_dB = 12.0;
actuator.requirements.minimumRollModeFrequencyRatio = 0.80;
actuator.requirements.maximumRollModeFrequencyRatio = 1.25;
actuator.requirements.minimumRollModeDampingRatio = 0.95;
actuator.requirements.minimumSpiralDecayRateRatio = 0.88;

%% Operational and nonlinear-protection gates

actuator.requirements.maximumInitialYawRateIntegralRatio = 0.82;
actuator.requirements.maximumInitialBetaIntegralRatio = 0.77;
actuator.requirements.maximumInitialActuatorPosition_deg = 2.0;
actuator.requirements.maximumInitialActuatorRate_degps = 20.0;
actuator.requirements.maximumAileronYawIntegralRatio = 0.95;
actuator.requirements.minimumAileronBankPeakRatio = 0.90;
actuator.requirements.maximumAileronBankPeakRatio = 1.05;
actuator.requirements.maximumRudderYawIntegralRatio = 0.80;
actuator.requirements.maximumRudderBetaPeakRatio = 0.85;
actuator.requirements.maximumPulseActuatorPosition_deg = 1.0;
actuator.requirements.maximumPulseActuatorRate_degps = 5.0;
actuator.requirements.maximumReleasedRudder_deg = 0.01;
actuator.requirements.maximumNoiseActuatorRms_deg = 0.010;
actuator.requirements.maximumNoiseActuatorPeak_deg = 0.050;

%% Validation definitions

actuator.validation.timeStep_s = 0.01;
actuator.validation.stopTime_s = 30.0;
actuator.validation.diskMarginFrequencyRange_radps = [1.0e-4,1.0e4];
actuator.validation.diskMarginGridPointCount = 2001;

% This deliberately large initial rate is a protection-logic stress case,
% not a Plant V0.1 aerodynamic-envelope claim.
actuator.stress.initialYawRate_degps = 25.0;
actuator.stress.timeStep_s = 0.001;
actuator.stress.stopTime_s = 30.0;
% The washout intentionally releases slow/steady yaw instead of acting as
% a heading-hold loop. The loose final-r gate only verifies substantial
% recovery from the 25-deg/s logic upset; rudder release is checked
% separately and is the relevant washout requirement.
actuator.stress.maximumFinalYawRate_degps = 1.0;

actuator.noise.timeStep_s = 0.001;
actuator.noise.stopTime_s = 60.0;
actuator.noise.discardTime_s = 10.0;

%% Source and assumption classification

actuator.references.servoArchitecture = [ ...
    'Garmin certified-autopilot documentation states that servo motor ', ...
    'control limits maximum speed and torque and that the servo can be ', ...
    'mechanically overridden. It does not publish a C172S yaw-servo ', ...
    'rudder-angle authority, surface-rate limit or identified time constant.'];

actuator.assumptions = { ...
    'Automatic authority of +/-5 deg is a project design selection.'; ...
    'The 0.050 s actuator time constant is a project assumption.'; ...
    'The 20 deg/s servo rate is a project assumption.'; ...
    'The 0.020 s sensor filter and 0.010 s nominal delay are assumptions.'; ...
    'Noise levels and the sensitivity grids are non-probabilistic tests.'};

actuator.limitations = [ ...
    'Valid only with YD-V1-MATLAB-R2 and provisional Lateral-Directional ', ...
    'Plant V0.1 at 4000 ft ISA, 110 KTAS and 2550 lb. Torque saturation, ', ...
    'cable elasticity/friction, servo clutch mechanics, sensor bias, ', ...
    'quantization, failures, gust/turbulence, gain scheduling and full-', ...
    'envelope nonlinear aerodynamics remain outside this audit. This is ', ...
    'an educational research model and is not approved for aircraft use.'];

end
