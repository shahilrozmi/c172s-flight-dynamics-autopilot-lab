function controller = c172sYawDamperData()
%C172SYAWDAMPERDATA Yaw Damper V1 definition and validation ledger.
%
%   The controller uses washout-filtered yaw-rate feedback:
%
%       xwDot          = (r - xw)/Tw
%       rWashout       = r - xw
%       deltaRDamper   = -Kr*rWashout
%       deltaRTotal    = deltaRPilot + deltaRDamper
%
%   Project signs are positive yaw rate nose-right and positive rudder
%   nose-right. Negative feedback therefore requires the explicit minus
%   sign in deltaRDamper.

controller.version = 'Yaw Damper V1';
controller.artifactRevision = 'YD-V1-MATLAB-R2';
controller.requiredPlantVersion = 'Lateral-Directional Plant V0.1';
controller.architecture = ...
    'Washout-filtered yaw-rate feedback to ideal rudder input';

controller.rudderGain_s = 0.280;
controller.washoutTimeConstant_s = 1.00;
controller.feedbackSign = -1.0;

% The washout time constant is a project design selection. Its 1 rad/s
% corner retains more than 90 percent magnitude at the nominal Dutch-roll
% frequency while strongly attenuating very slow/steady turn content.
controller.washoutCorner_radps = ...
    1.0/controller.washoutTimeConstant_s;

%% Gain-search definition

controller.design.gainSearchRange_s = [0.000,0.500];
controller.design.gainSearchIncrement_s = 0.005;
controller.design.minimumDutchDampingRatio = 0.35;
controller.design.minimumRollPoleMagnitudeRatio = 0.95;
controller.design.maximumRollPoleMagnitudeRatio = 1.05;
controller.design.minimumSpiralDecayRateRatio = 0.88;
controller.design.selectionPolicy = [ ...
    'Select the smallest gain on the declared grid that satisfies all ', ...
    '729-case dynamic-stability, Dutch-roll damping, roll-mode ', ...
    'preservation and spiral-mode preservation gates.'];

%% Structured sensitivity set

% These scale factors are explicit engineering sensitivity assumptions.
% They are not statistical confidence bounds, certification tolerances or
% C172S flight-test uncertainty estimates.
controller.uncertainty.derivativeScaleValues = [0.80,1.00,1.20];
controller.uncertainty.inertiaScaleValues = [0.80,1.00,1.20];
controller.uncertainty.IxzNormalizedValues = [-0.10,0.00,0.10];
controller.uncertainty.caseCount = 729;
controller.uncertainty.groups = { ...
    'CY_beta, CY_p and CY_r scaled together'; ...
    'Cl_beta, Cl_p and Cl_r scaled together'; ...
    'Cn_beta, Cn_p and Cn_r scaled together'; ...
    'CY_deltaR, Cl_deltaR and Cn_deltaR scaled together'; ...
    'Ixx and Izz scaled together'; ...
    'Ixz/sqrt(Ixx*Izz) varied independently'};
controller.uncertainty.interpretation = [ ...
    'Structured robustness stress test at the single Plant V0.1 trim ', ...
    'condition; not a probabilistic or full-envelope uncertainty model.'];

%% Project release gates

% MIL-F-8785C Class I, Category B Level 1 values are retained only as
% comparative flying-qualities guidance. MIL-F-8785C is inactive, applies
% to military aircraft, and is not a C172S certification basis.
controller.guidance.MILF8785C.minimumDutchDampingRatio = 0.08;
controller.guidance.MILF8785C.minimumDutchNaturalFrequency_radps = 0.40;
controller.guidance.MILF8785C.minimumDutchDecayRate_radps = 0.15;

controller.requirements.minimumDutchDampingRatio = 0.35;
controller.requirements.minimumDutchNaturalFrequency_radps = 0.40;
controller.requirements.minimumDutchDecayRate_radps = 0.15;
controller.requirements.minimumBalancedDiskPhaseMargin_deg = 60.0;
controller.requirements.minimumBalancedDiskGainMargin_dB = 12.0;
controller.requirements.minimumRollPoleMagnitudeRatio = 0.95;
controller.requirements.maximumRollPoleMagnitudeRatio = 1.05;
controller.requirements.minimumSpiralDecayRateRatio = 0.88;
controller.requirements.maximumInitialYawRateIntegralRatio = 0.82;
controller.requirements.maximumInitialBetaIntegralRatio = 0.77;
controller.requirements.maximumInitialDisturbanceRudder_deg = 2.0;
controller.requirements.maximumSlowFrequencyWashoutMagnitude = 0.06;
controller.requirements.minimumDutchBandWashoutMagnitude = 0.90;
controller.requirements.maximumFourSecondWashoutResidual = 0.02;

% Nominal pulse-response preservation/improvement gates.
controller.requirements.maximumAileronYawIntegralRatio = 0.95;
controller.requirements.minimumAileronBankPeakRatio = 0.90;
controller.requirements.maximumAileronBankPeakRatio = 1.05;
controller.requirements.maximumRudderYawIntegralRatio = 0.80;
controller.requirements.maximumRudderBetaPeakRatio = 0.85;
controller.requirements.maximumPulseAutomaticRudder_deg = 1.0;

%% Validation definitions

controller.validation.initialYawRate_degps = 5.0;
controller.validation.aileronPulse_deg = 1.0;
controller.validation.aileronPulseStart_s = 0.0;
controller.validation.aileronPulseEnd_s = 1.0;
controller.validation.rudderPulse_deg = 1.0;
controller.validation.rudderPulseStart_s = 5.0;
controller.validation.rudderPulseEnd_s = 6.0;
controller.validation.slowTurnFrequency_radps = 0.05;
controller.validation.washoutResidualTime_s = 4.0;
controller.validation.timeStep_s = 0.02;
controller.validation.stopTime_s = 30.0;
controller.validation.diskMarginFrequencyRange_radps = [1.0e-4,1.0e4];
controller.validation.diskMarginGridPointCount = 2001;

%% Source and assumption classification

controller.references.faaStability = [ ...
    '14 CFR 23.2145 requires dynamic Dutch-roll stability in normal ', ...
    'operations but supplies no numerical controller-design target.'];
controller.references.flyingQualitiesGuidance = [ ...
    'MIL-F-8785C (5 Nov 1980), Table VI, Class I Category B Level 1; ', ...
    'used as inactive comparative guidance only.'];
controller.references.washoutRationale = [ ...
    'A.H. Vaillard and J.D. Paduano, Development of a Sensitivity ', ...
    'Analysis Technique for Multiloop Flight Control Systems, 1985; ', ...
    'NASA NTRS 19860007888.'];
controller.references.designCrossCheck = [ ...
    'MathWorks Control System Toolbox example, Design a Yaw Damper for ', ...
    'a 747 Jet Transport; washout preserves the slow spiral response.'];
controller.references.stabilityMarginMethod = [ ...
    'Balanced disk margins are used because the yaw loop can have ', ...
    'multiple unity-gain crossings. The metric covers all frequencies ', ...
    'and simultaneous gain/phase variation; classical margin() values ', ...
    'are retained neither as a gate nor as a robustness claim.'];

controller.assumptions = { ...
    'Kr and Tw are project controller-design selections, not C172S data.'; ...
    'Yaw-rate sensing and rudder actuation are ideal in V1.'; ...
    'The sensitivity grid is structured and non-probabilistic.'; ...
    'Fixed-gain validity is assessed only at the Plant V0.1 trim point.'};

controller.limitations = [ ...
    'Valid only for the provisional Lateral-Directional Plant V0.1 ', ...
    'small-disturbance model at 4000 ft ISA, 110 KTAS and 2550 lb. ', ...
    'Rudder actuator dynamics, position/rate authority, sensor dynamics, ', ...
    'noise, delay, failures, gust/turbulence response, nonlinear envelope ', ...
    'behavior and operating-point gain scheduling remain future stages. ', ...
    'This is an educational research baseline and is not approved for ', ...
    'aircraft installation or operation.'];

end
