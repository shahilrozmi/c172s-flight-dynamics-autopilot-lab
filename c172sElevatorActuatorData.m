function actuator = c172sElevatorActuatorData()
%C172SELEVATORACTUATORDATA Protected elevator-actuator definition.
%
%   This file deliberately separates certified C172S surface travel from
%   provisional control-system assumptions. Only the physical travel values
%   are aircraft data. Servo time constant, servo rate and the five-degree
%   automatic-control authority are transparent engineering assumptions for
%   V1 and must be replaced if installation-specific data become available.

actuator.version = 'Elevator Actuator and Anti-Windup V1';
actuator.integratedLoopVersion = 'Pitch-Attitude Hold V2';

%% Aircraft physical surface travel

% FAA TCDS 3A12 Rev. 88 lists elevator travel as 28 deg up and 26 deg down.
% Project convention: positive elevator is trailing-edge down.
actuator.physicalTravel.trailingEdgeUp_deg = 28.0;
actuator.physicalTravel.trailingEdgeDown_deg = 26.0;
actuator.physicalTravel.lowerLimit_rad = ...
    -deg2rad(actuator.physicalTravel.trailingEdgeUp_deg);
actuator.physicalTravel.upperLimit_rad = ...
    deg2rad(actuator.physicalTravel.trailingEdgeDown_deg);
actuator.physicalTravel.source = 'FAA TCDS 3A12 Rev. 88';
actuator.physicalTravel.classification = 'certified aircraft geometry';

%% Provisional automatic-control authority and actuator dynamics

% The automatic controller is intentionally restricted to a small fraction
% of the available physical travel. This is a project design choice, not a
% published C172S autopilot limit.
actuator.authority.trailingEdgeUp_deg = 5.0;
actuator.authority.trailingEdgeDown_deg = 5.0;
actuator.authority.lowerLimit_rad = ...
    -deg2rad(actuator.authority.trailingEdgeUp_deg);
actuator.authority.upperLimit_rad = ...
    deg2rad(actuator.authority.trailingEdgeDown_deg);
actuator.authority.uncertainty_deg = [4.0,5.0,6.0];

actuator.dynamics.timeConstant_s = 0.035;
actuator.dynamics.timeConstantRange_s = [0.020,0.050];
actuator.dynamics.timeConstantCases_s = [0.020,0.035,0.050];
actuator.dynamics.rateLimit_degps = 20.0;
actuator.dynamics.rateLimitRange_degps = [15.0,25.0];
actuator.dynamics.rateLimitCases_degps = [15.0,20.0,25.0];

actuator.antiWindup.trackingTimeConstant_s = 0.50;
actuator.antiWindup.law = [ ...
    'xiDot = eTheta + (deltaEUnsat - deltaETrack)/(Ki*Tt), ', ...
    'where deltaETrack = deltaE + tau*limitedRate'];

%% Actuator-integrated q-damper compensation

% Kp and Ki remain frozen at their V1 values. The original Kq=0.110 s does
% not retain zeta >= 0.60 at tau=0.050 s. A robust 0.005-grid search selects
% the smallest compensating value that restores the damping gate.
actuator.integration.Kq_s = 0.120;
actuator.integration.KqSearchRange_s = [0.110,0.160];
actuator.integration.KqSearchIncrement_s = 0.005;
actuator.integration.minimumFastModeZeta = 0.60;

%% Validation gates

actuator.requirements.minimumFastModeZeta = 0.60;
actuator.requirements.minimumPhaseMargin_deg = 50.0;
actuator.requirements.maximumGainCrossover_radps = 1.50;
actuator.requirements.minimumBandwidthSeparationRatio = 2.80;
actuator.requirements.maximumStepOvershoot_pct = 3.0;
actuator.requirements.minimumRiseTime_s = 2.0;
actuator.requirements.maximumRiseTime_s = 4.0;
actuator.requirements.maximumSettlingTime_s = 35.0;
actuator.requirements.maximumPitchRate_degps = 2.5;
actuator.requirements.maximumAutomaticElevator_deg = 4.0;
actuator.requirements.maximumFinalAttitudeError_deg = 0.05;
actuator.requirements.maximumSpeedPerturbationFraction = 0.10;
actuator.requirements.maximumAlphaPerturbation_deg = 1.0;
actuator.requirements.maximumNominalActuatorRate_degps = 1.0;
actuator.requirements.maximumAttitudeDifferenceFromIdeal_deg = 0.03;

actuator.validation.commandStep_deg = 1.0;
actuator.validation.stopTime_s = 60.0;
actuator.validation.timeStep_s = 0.01;

% This deliberately out-of-envelope initial condition is a controller-logic
% stress test only. It is not an aerodynamic or flight-envelope claim.
actuator.stress.initialAttitude_deg = 30.0;
actuator.stress.stopTime_s = 40.0;
actuator.stress.timeStep_s = 0.001;
actuator.stress.recoveryThreshold_deg = 0.5;
actuator.stress.maximumPeakIntegratorRatio = 0.40;
actuator.stress.maximumSaturationDurationRatio = 0.65;
actuator.stress.maximumFinalAttitude_deg = 0.05;

actuator.assumptionClassification = [ ...
    'tau, rate limit, automatic authority and anti-windup tracking time ', ...
    'are provisional engineering assumptions; they are not C172S or ', ...
    'Garmin GFC 700 identified parameters.'];

end
