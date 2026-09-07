function altitude = c172sAltitudeHoldControllerData()
%C172SALTITUDEHOLDCONTROLLERDATA Altitude-hold controller definition.
%
%   The controller wraps around the frozen actuator-protected pitch-
%   attitude loop. Project state convention is
%
%       gamma ~= theta - alpha
%       hDot  ~= Vtrim*(theta - alpha)
%
%   and the outer-loop law is
%
%       hError        = hCommandFiltered - h
%       hDotCommand   = sat(Kh*hError, +/-hDotCommandLimit)
%       thetaCommand  = sat(Kv*(hDotCommand - hDot), +/-thetaLimit)
%
%   There is deliberately no altitude-error integrator. Altitude already
%   contributes a kinematic integrator, while the proportional altitude
%   path gives zero step error in the ideal linear model and avoids a new
%   windup state during bounded altitude captures.

altitude.version = 'Altitude Hold V1';
altitude.architecture = [ ...
    'Filtered altitude reference with bounded vertical-speed command and ', ...
    'vertical-speed-damped pitch command around frozen Pitch-Attitude Hold V2'];

%% Controller gains and command shaping

altitude.altitudeGain_per_s = 0.600;
altitude.verticalSpeedGain_radPerMps = 0.020;
altitude.commandFilterTimeConstant_s = 5.0;

altitude.verticalSpeedCommandLimit_fpm = 300.0;
altitude.verticalSpeedCommandLimit_mps = ...
    altitude.verticalSpeedCommandLimit_fpm/196.850393700787;
altitude.pitchCommandLimit_deg = 3.0;
altitude.pitchCommandLimit_rad = deg2rad(altitude.pitchCommandLimit_deg);

%% Robust-search definition

altitude.design.verticalSpeedGainSearchRange = [0.0100,0.0400];
altitude.design.verticalSpeedGainIncrement = 0.0025;
altitude.design.minimumVerticalSpeedFeedbackPhaseMargin_deg = 90.0;

altitude.design.altitudeGainSearchRange = [0.20,0.80];
altitude.design.altitudeGainIncrement = 0.05;
altitude.design.minimumAltitudePhaseMargin_deg = 70.0;
altitude.design.minimumAttitudeToAltitudeBandwidthRatio = 3.0;

altitude.design.description = [ ...
    'The largest vertical-speed feedback gain retaining at least 90 deg ', ...
    'phase margin is selected first. The largest altitude gain retaining ', ...
    'at least 70 deg altitude-loop phase margin and 3:1 separation from ', ...
    'the frozen attitude loop is then selected.'];

%% Linear small-signal validation

altitude.validation.smallSignalCommand_ft = 10.0;
altitude.validation.smallSignalCommand_m = ...
    altitude.validation.smallSignalCommand_ft*0.3048;
altitude.validation.operationalCommand_ft = 100.0;
altitude.validation.operationalCommand_m = ...
    altitude.validation.operationalCommand_ft*0.3048;
altitude.validation.stopTime_s = 180.0;
altitude.validation.timeStep_s = 0.02;

altitude.requirements.maximumSmallSignalOvershoot_pct = 2.0;
altitude.requirements.minimumSmallSignalRiseTime_s = 10.0;
altitude.requirements.maximumSmallSignalRiseTime_s = 20.0;
altitude.requirements.maximumSmallSignalSettlingTime_s = 70.0;
altitude.requirements.maximumSmallSignalFinalError_ft = 0.10;
altitude.requirements.maximumSmallSignalPitchCommand_deg = 1.0;
altitude.requirements.maximumSmallSignalVerticalSpeed_fpm = 100.0;

%% Bounded 100-ft operational capture validation

altitude.requirements.maximumOperationalOvershoot_pct = 2.0;
altitude.requirements.minimumOperationalRiseTime_s = 30.0;
altitude.requirements.maximumOperationalRiseTime_s = 50.0;
altitude.requirements.maximumOperationalSettlingTime_s = 90.0;
altitude.requirements.maximumOperationalFinalError_ft = 1.0;
altitude.requirements.maximumOperationalPitchCommand_deg = 3.0;
altitude.requirements.maximumOperationalPitch_deg = 2.0;
altitude.requirements.maximumOperationalVerticalSpeed_fpm = 250.0;
altitude.requirements.maximumOperationalElevator_deg = 1.0;
altitude.requirements.maximumOperationalActuatorRate_degps = 1.0;
altitude.requirements.maximumSpeedPerturbationFraction = 0.10;
altitude.requirements.maximumAlphaPerturbation_deg = 1.0;

%% Kinematic and assumption metadata

altitude.kinematics.law = 'hDot = Vtrim*(theta - alpha)';
altitude.kinematics.classification = [ ...
    'small-disturbance level-trim kinematics using the project state convention'];

altitude.assumptionClassification = [ ...
    'Altitude/vertical-speed gains, the 5 s command filter, the 300 ft/min ', ...
    'vertical-speed command authority and the 3 deg pitch-command authority ', ...
    'are project engineering assumptions, not identified C172S or GFC 700 parameters.'];

altitude.limitations = [ ...
    'Valid for the Longitudinal Plant V1 trim point with fixed incremental ', ...
    'thrust. This is elevator-only altitude capture, not total-energy ', ...
    'control. Wind, sensor dynamics, terrain clearance, engine transients ', ...
    'and full-envelope nonlinear aerodynamics are outside V1 scope.'];

end
