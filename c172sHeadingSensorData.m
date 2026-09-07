function sensor = c172sHeadingSensorData()
%C172SHEADINGSENSORDATA Heading-source and circular-logic audit ledger.
%
%   The filter, delay, bias and noise values in this file are transparent
%   project assumptions. They are not identified C172S, Garmin G1000 or
%   magnetometer hardware data.

sensor.version = 'Heading Sensor and Circular Logic V1';
sensor.artifactRevision = 'HEADING-SENSOR-V1-AUDIT-MATLAB-R1';
sensor.requiredPlantVersion = 'Lateral-Directional Plant V0.1';
sensor.requiredRollControllerRevision = 'RAH-V1-MATLAB-R1';

%% Circular heading convention and error law

sensor.headingRange_deg = [0.0,360.0];
sensor.period_deg = 360.0;
sensor.errorRange_deg = [-180.0,180.0];
sensor.errorLaw = [ ...
    'ePsi = atan2(sin(psiCommand-psiSensor),', ...
    'cos(psiCommand-psiSensor))'];
sensor.turnSelectionPolicy = [ ...
    'Select the shortest signed angular displacement. Positive error ', ...
    'commands a nose-right/right-bank turn; negative error commands ', ...
    'a nose-left/left-bank turn. The exactly 180-deg tie is resolved ', ...
    'by the sign returned by atan2 for the supplied command difference.'];

%% Provisional heading-source dynamics

% The circular error is formed before control limiting. In the small-error
% linear audit the heading channel is represented by a first-order filter
% plus an independently applied exact pure delay.
sensor.filterTimeConstant_s = 0.100;
sensor.filterTimeConstantCases_s = [0.050,0.100,0.200];
sensor.nominalDelay_s = 0.050;
sensor.delayCases_s = [0.000,0.050,0.100,0.200,0.400];
sensor.maximumAcceptedDelay_s = 0.200;

%% Deterministic error-source tests

sensor.noiseRms_deg = 0.250;
sensor.noiseStressRms_deg = 0.500;
sensor.biasStress_deg = 2.000;
sensor.noiseFrequency_radps = 0.800;
sensor.noiseTimeStep_s = 0.002;
sensor.noiseStopTime_s = 80.0;
sensor.noiseDiscardTime_s = 10.0;

%% Release gates for the sensor and circular logic

sensor.requirements.maximumNoiseBankRms_deg = 0.200;
sensor.requirements.maximumNoiseAileronRms_deg = 0.040;
sensor.requirements.maximumNoiseAileronPeak_deg = 0.050;
sensor.requirements.maximumMeasuredBiasCaseError_deg = 0.050;
sensor.requirements.maximumBiasResidualMismatch_deg = 0.100;
sensor.requirements.maximumWrapCaptureError_deg = 0.050;

%% Evidence, assumptions and limitations

sensor.references.headingMode = [ ...
    'Garmin G1000 Cockpit Reference Guide for the Cessna Nav III, ', ...
    'P/N 190-00384-09 Rev. A, Sec. 6-19: Heading Select acquires and ', ...
    'maintains the Selected Heading.'];
sensor.references.turnDirection = [ ...
    'The same Garmin reference commands turns in the direction of ', ...
    'Selected Heading Bug movement, including commands beyond 180 deg.'];
sensor.references.garminHeadingModeUrl = [ ...
    'https://www8.garmin.com/manuals/webhelp/', ...
    'GUID-1FA95EDE-BA0A-4000-B109-7E73945389F7/EN-US/', ...
    'GUID-275A136F-5854-4A5D-B0DC-5B05F4075F68.html'];

sensor.assumptions = { ...
    'The 0.100-s heading filter is a project assumption.'; ...
    'The 0.050-s nominal heading/processing delay is an assumption.'; ...
    'The accepted 0-to-0.200-s delay envelope is a project gate.'; ...
    'The noise and bias levels are deterministic project stress tests.'; ...
    'Shortest-path wrap logic replaces Garmin knob-movement history.'};

sensor.limitations = [ ...
    'No magnetic-field model, compass calibration, AHRS aiding, latitude ', ...
    'effects, magnetic variation, heading-source switching, quantization, ', ...
    'dropouts, failures or installation-specific sensor data are modeled. ', ...
    'Because knob-event history is absent, this V1 does not reproduce the ', ...
    'Garmin same-direction-as-bug-movement rule for commands beyond ', ...
    '180 deg. This is not approved aircraft equipment data.'];

end
