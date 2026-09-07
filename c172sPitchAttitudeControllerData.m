function controller = c172sPitchAttitudeControllerData()
%C172SPITCHATTITUDECONTROLLERDATA Pitch-attitude hold definition.
%
%   The controller wraps around Pitch-Rate Damper V1. With the project
%   convention that positive elevator is trailing-edge down,
%
%       eTheta        = thetaFilteredCommand - theta
%       xiDot         = eTheta
%       deltaECommand = -Kp*eTheta - Ki*xi
%       deltaE        = deltaECommand + Kq*q
%
%   A positive pitch-attitude error therefore commands trailing-edge-up
%   elevator, which produces the required nose-up pitching moment.

controller.version = 'Pitch-Attitude Hold V1';
controller.architecture = 'Filtered-command PI wrapped around Pitch-Rate Damper V1';

controller.Kp_radPerRad = 0.450;
controller.Ki_radPerRadPerS = 0.450;
controller.commandFilterTimeConstant_s = 2.0;

% The PI zero is fixed at 1 rad/s. The scalar gain was selected from a
% robust search as the largest 0.005-grid value retaining at least 0.61
% damping in the fast oscillatory mode across the 36-case uncertainty set.
controller.design.gainSearchRange = [0.10, 0.70];
controller.design.gainSearchIncrement = 0.005;
controller.design.piZero_radps = ...
    controller.Ki_radPerRadPerS/controller.Kp_radPerRad;
controller.design.minimumSearchFastModeZeta = 0.61;
controller.design.description = [ ...
    'Kp and Ki use the constrained family Kp = Ki, placing the PI zero ', ...
    'at 1 rad/s. The selected gain preserves fast-mode damping and ', ...
    'bandwidth separation; a 2 s command filter limits command transients.'];

% Project engineering requirements for this small-signal design stage.
% These are validation gates, not certification claims.
controller.requirements.minimumFastModeZeta = 0.60;
controller.requirements.minimumPhaseMargin_deg = 50.0;
controller.requirements.maximumGainCrossover_radps = 1.50;
controller.requirements.minimumBandwidthSeparationRatio = 2.80;
controller.requirements.maximumStepOvershoot_pct = 3.0;
controller.requirements.minimumRiseTime_s = 2.0;
controller.requirements.maximumRiseTime_s = 4.0;
controller.requirements.maximumSettlingTime_s = 35.0;
controller.requirements.maximumPitchRate_degps = 2.5;
controller.requirements.maximumElevator_deg = 4.0;
controller.requirements.maximumFinalAttitudeError_deg = 0.05;
controller.requirements.maximumSpeedPerturbationFraction = 0.10;
controller.requirements.maximumAlphaPerturbation_deg = 1.0;

% Sustained tests are deliberately small so the linear perturbation model
% remains near its 110-KTAS trim condition. Larger commands belong in the
% later nonlinear, actuator-aware validation stage.
controller.validation.commandStep_deg = 1.0;
controller.validation.initialAttitudeDisturbance_deg = 1.0;
controller.validation.constantElevatorBias_deg = 0.1;
controller.validation.stopTime_s = 60.0;
controller.validation.timeStep_s = 0.02;

controller.limitations = [ ...
    'Valid only for the Longitudinal Plant V1 small-disturbance model at ', ...
    '4000 ft ISA, 110 KTAS and 2550 lb. Sustained V1 tests are limited by ', ...
    '|u|/Vtrim <= 0.10 and |alpha| <= 1 deg. Actuator dynamics, saturation, ', ...
    'sensor dynamics and nonlinear-envelope validation remain separate ', ...
    'future stages.'];

end
