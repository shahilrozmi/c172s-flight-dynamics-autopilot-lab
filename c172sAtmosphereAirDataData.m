function data = c172sAtmosphereAirDataData()
%C172SATMOSPHEREAIRDATADATA Contract for the ISA/wind/air-data layer.

source = c172sNonlinear6DofSourceData();
kernel = c172sRigidBody6DofData();
longitudinal = c172sLongitudinalData();

data.artifactRevision = ...
    'NONLINEAR-6DOF-V1-ATMOSPHERE-AIR-DATA-MATLAB-R1';
data.required.sourceAudit = source.artifactRevision;
data.required.rigidBodyKernel = kernel.artifactRevision;
data.required.longitudinalPlant = source.required.longitudinalPlant;
data.required.masterRegressionPasses = 18;

data.constants.g_mps2 = 9.80665;
data.constants.Rair_JpkgK = 287.05287;
data.constants.gamma = 1.4;
data.constants.T0_K = 288.15;
data.constants.p0_Pa = 101325.0;
data.constants.rho0_kgpm3 = 1.225;
data.constants.lapse_Kpm = 0.0065;
data.constants.sutherlandReference_K = 273.15;
data.constants.sutherlandReferenceViscosity_Pas = 1.716e-5;
data.constants.sutherlandConstant_K = 110.4;

data.geometry.cBar_m = longitudinal.geometry.cBar_m;

data.atmosphere.model = '1976 ISA troposphere';
data.atmosphere.minimumGeopotentialAltitude_m = -1000.0;
data.atmosphere.maximumGeopotentialAltitude_m = 11000.0;
data.atmosphere.ISADeviation_K = 0.0;

data.wind.definition = ...
    'NED velocity of the air mass relative to Earth';
data.wind.airRelativeDefinition = ...
    'velocityAirNED = velocityGroundNED - windNED - gustNED';
data.wind.steadyUnits = 'm/s NED';
data.wind.gustUnits = 'm/s NED';

data.airData.minimumValidAirspeed_mps = 1.0;
data.airData.lowSpeedAlpha_rad = 0.0;
data.airData.lowSpeedBeta_rad = 0.0;
data.airData.betaArgumentLimit = 1.0;
data.airData.alphaDefinition = 'atan2(wAir,uAir)';
data.airData.betaDefinition = 'asin(clamp(vAir/TAS,-1,+1))';
data.airData.dynamicPressureDefinition = '0.5*rho*TAS^2';
data.airData.machDefinition = 'TAS/speedOfSound';
data.airData.reynoldsDefinition = 'rho*TAS*cBar/dynamicViscosity';

data.trim.pressureAltitude_ft = source.trim.pressureAltitude_ft;
data.trim.altitude_m = longitudinal.trim.altitude_m;
data.trim.V_KTAS = source.trim.V_KTAS;
data.trim.V_mps = longitudinal.trim.V_mps;
data.trim.temperature_K = longitudinal.trim.temperature_K;
data.trim.pressure_Pa = longitudinal.trim.pressure_Pa;
data.trim.rho_kgpm3 = longitudinal.trim.rho_kgpm3;
data.trim.qbar_Pa = longitudinal.trim.qbar_Pa;

% The conventional rho0 value is rounded to 1.225 kg/m^3, whereas p0/(R*T0)
% differs by about 1.8e-8. The gate retains that published rounding only.
data.validation.maximumISAAbsoluteError = 1.0e-7;
data.validation.maximumKinematicError = 1.0e-11;
data.validation.maximumAngleError_rad = 1.0e-12;
data.validation.maximumDerivativeRelativeError = 1.0e-6;
data.validation.requireFiniteLowSpeedOutputs = true;
data.validation.requireDeterministicReplay = true;
data.validation.aerodynamicsIncluded = false;
data.validation.propulsionIncluded = false;
data.notForAircraftApproval = true;

end
