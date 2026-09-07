function controller = c172sRollAttitudeControllerData()
%C172SROLLATTITUDECONTROLLERDATA Protected roll-attitude hold definition.
%
%   Project signs:
%     * positive p and phi are right-wing-down;
%     * positive deltaA commands right-wing-down roll;
%     * ePhi = phiCommandFiltered - phiSensor;
%
%   Control law:
%
%     deltaAUnsat = Kphi*ePhi + Ki*integral(ePhi) - Kp*pSensor
%
%   The roll-rate term supplies damping, the bank-angle proportional term
%   supplies attitude stiffness, and the slow integral term rejects the
%   finite steady error and constant aileron bias of a proportional-only
%   loop. Yaw Damper V2 remains active and frozen.

controller.version = 'Roll-Attitude Hold V1';
controller.artifactRevision = 'RAH-V1-MATLAB-R1';
controller.requiredPlantVersion = 'Lateral-Directional Plant V0.1';
controller.requiredYawDamperRevision = 'YD-V1-MATLAB-R2';
controller.requiredRudderAuditRevision = 'YD-V2-AUDIT-MATLAB-R1';
controller.requiredAileronAuditRevision = ...
    'AILERON-V1-AUDIT-MATLAB-R1';
controller.architecture = [ ...
    'Filtered bank-angle PI feedback plus filtered roll-rate damping, ', ...
    'protected aileron actuation, and frozen protected yaw damping'];

%% Frozen controller gains

controller.bankProportionalGain = 0.300;
controller.bankIntegralGain_per_s = 0.090;
controller.rollRateGain_s = 0.075;
controller.commandFilterTimeConstant_s = 2.00;
controller.integralZero_radps = ...
    controller.bankIntegralGain_per_s/controller.bankProportionalGain;

controller.feedback.bankErrorSign = 1.0;
controller.feedback.rollRateSign = -1.0;
controller.feedback.bankErrorDefinition = ...
    'ePhi = phiCommandFiltered - phiSensor';

%% Automatic bank-command protection

% Garmin's G1000 Cessna Nav III reference describes Roll Hold behavior as
% wings-level below 6 deg, holding 6-22 deg, and limiting above 22 deg.
% The project uses a slightly conservative +/-20-deg command limit. This is
% an architectural reference and project selection, not permission to
% claim an installation-identical GFC 700 controller.
controller.commandAuthority.rightWingDown_deg = 20.0;
controller.commandAuthority.leftWingDown_deg = 20.0;
controller.commandAuthority.lowerLimit_rad = ...
    -deg2rad(controller.commandAuthority.leftWingDown_deg);
controller.commandAuthority.upperLimit_rad = ...
    deg2rad(controller.commandAuthority.rightWingDown_deg);
controller.commandAuthority.source = [ ...
    'Garmin G1000 Cockpit Reference Guide for Cessna Nav III, ', ...
    'P/N 190-00384-09 Rev. A, Sec. 6-18; project limit selected at 20 deg'];
controller.commandAuthority.classification = ...
    'manufacturer architecture reference plus project design selection';

%% Reproducible gain-search policy

% Kphi and Ki define the selected PI family after a documented response and
% control-effort trade study. Kp is then selected objectively as the first
% 0.005-s grid point that satisfies the complete structured modal and
% accepted-delay disk-margin gates. The next lower point, 0.070 s, fails
% the minimum gain-margin gate in the declared family.
controller.design.fixedBankProportionalGain = 0.300;
controller.design.fixedBankIntegralGain_per_s = 0.090;
controller.design.rollRateGainSearchRange_s = [0.050,0.100];
controller.design.rollRateGainIncrement_s = 0.005;
controller.design.minimumDynamicDecayRate_per_s = 0.35;
controller.design.minimumOscillatoryDampingRatio = 0.35;
controller.design.minimumBalancedDiskPhaseMargin_deg = 60.0;
controller.design.minimumBalancedDiskGainMargin_dB = 12.0;
controller.design.selectionPolicy = [ ...
    'Select the smallest Kp on the declared grid that satisfies all ', ...
    '729 roll-plant x 3 aileron-actuator modal gates and all accepted-', ...
    'delay disk-margin gates, with Kphi and Ki held at the declared ', ...
    'trade-study values.'];

%% Structured roll-design uncertainty set

% This family deliberately replaces the earlier independent rudder-
% effectiveness scale with an aileron-effectiveness scale. Frozen Yaw
% Damper V2 has already been separately validated against its complete
% rudder-effectiveness uncertainty family.
controller.uncertainty.derivativeScaleValues = [0.80,1.00,1.20];
controller.uncertainty.inertiaScaleValues = [0.80,1.00,1.20];
controller.uncertainty.IxzNormalizedValues = [-0.10,0.00,0.10];
controller.uncertainty.caseCount = 729;
controller.uncertainty.groups = { ...
    'CY_beta, CY_p and CY_r scaled together'; ...
    'Cl_beta, Cl_p and Cl_r scaled together'; ...
    'Cn_beta, Cn_p and Cn_r scaled together'; ...
    'CY_deltaA, Cl_deltaA and Cn_deltaA scaled together'; ...
    'Ixx and Izz scaled together'; ...
    'Ixz/sqrt(Ixx*Izz) varied independently'};
controller.uncertainty.interpretation = [ ...
    'Structured, non-probabilistic robustness stress test at the single ', ...
    'Plant V0.1 trim point; not a full-envelope uncertainty model.'];

%% Linear and command-response release gates

controller.requirements.minimumDynamicDecayRate_per_s = 0.35;
controller.requirements.minimumOscillatoryDampingRatio = 0.35;
controller.requirements.minimumBalancedDiskPhaseMargin_deg = 60.0;
controller.requirements.minimumBalancedDiskGainMargin_dB = 12.0;
controller.requirements.maximumStepOvershoot_pct = 10.0;
controller.requirements.minimumRiseTime_s = 2.5;
controller.requirements.maximumRiseTime_s = 4.0;
controller.requirements.maximumSettlingTime_s = 15.0;
controller.requirements.maximumFinalBankError_deg = 0.01;
controller.requirements.maximumRollRate_degps = 5.0;
controller.requirements.maximumAutomaticAileron_deg = 2.0;
controller.requirements.maximumAutomaticRudder_deg = 0.50;
controller.requirements.maximumSideslip_deg = 0.75;
controller.requirements.maximumYawRate_degps = 2.50;
controller.requirements.maximumRegulationResidual_deg = 0.05;

%% Validation definitions

controller.validation.commandStep_deg = 10.0;
controller.validation.timeStep_s = 0.02;
controller.validation.stopTime_s = 40.0;
controller.validation.initialBankDisturbance_deg = 5.0;
controller.validation.constantAileronBias_deg = 0.20;
controller.validation.diskMarginFrequencyRange_radps = [1.0e-4,1.0e4];
controller.validation.diskMarginGridPointCount = 2001;

%% Source and scope classification

controller.references.rollModeArchitecture = [ ...
    'Garmin G1000 Cessna Nav III documentation identifies Roll Hold as ', ...
    'the default lateral mode and describes its bank-angle behavior.'];
controller.references.controlStructure = [ ...
    'Bank-angle PI feedback with roll-rate damping is a project classical-', ...
    'control architecture, not a reverse-engineered Garmin control law.'];
controller.references.marginMethod = [ ...
    'Balanced SISO disk margins are evaluated over the complete frequency ', ...
    'grid with the exact exp(-s*Td) delay factor.'];

controller.assumptions = { ...
    'Kphi, Ki, Kp and the 2-s command filter are project selections.'; ...
    'The +/-20-deg bank-command limit is a conservative project limit.'; ...
    'Roll-sensor and aileron-servo data come from the separate audit ledger.'; ...
    'The structured uncertainty set is non-probabilistic.'};

controller.limitations = [ ...
    'Valid only with the frozen protected yaw damper and provisional ', ...
    'Lateral-Directional Plant V0.1 at 4000 ft ISA, 110 KTAS and 2550 lb. ', ...
    'Heading/course capture, turn coordination feedforward, wind/gusts, ', ...
    'failures, torque/friction, gain scheduling and full-envelope ', ...
    'nonlinear aerodynamics remain outside V1. This educational research ', ...
    'controller is not approved for aircraft installation or operation.'];

end
