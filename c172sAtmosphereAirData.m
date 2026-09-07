function output = c172sAtmosphereAirData( ...
    state,steadyWindNED_mps,gustNED_mps,model)
%C172SATMOSPHEREAIRDATA ISA atmosphere and air-relative state calculation.
%
% STATE follows the frozen 13-state order
% [u v w p q r pN pE pD q0 q1 q2 q3].' Body velocity is ground-relative.
% Wind inputs are air-mass velocities relative to Earth in NED axes.

validateattributes(state,{'double'}, ...
    {'real','finite','column','numel',13},mfilename,'state',1);
validateattributes(steadyWindNED_mps,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'steadyWindNED_mps',2);
validateattributes(gustNED_mps,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'gustNED_mps',3);

required = {'constants','geometry','atmosphere','airData'};
for k = 1:numel(required)
    if ~isfield(model,required{k})
        error('c172sAtmosphereAirData:MissingModelField', ...
            'model.%s is required.',required{k});
    end
end

quaternion = state(10:13);
quaternionNorm = norm(quaternion);
if quaternionNorm < 1.0e-12
    error('c172sAtmosphereAirData:InvalidQuaternion', ...
        'The quaternion norm is too small.');
end
if abs(quaternionNorm-1.0) > 1.0e-2
    error('c172sAtmosphereAirData:QuaternionNotUnit', ...
        'Quaternion norm error exceeds 0.01.');
end
quaternion = quaternion/quaternionNorm;
bodyToNED = quaternionToDCM(quaternion);

altitude_m = -state(9);
if altitude_m < model.atmosphere.minimumGeopotentialAltitude_m || ...
        altitude_m > model.atmosphere.maximumGeopotentialAltitude_m
    error('c172sAtmosphereAirData:AltitudeOutsideModel', ...
        'Altitude %.3f m lies outside the ISA troposphere contract.', ...
        altitude_m);
end

c = model.constants;
temperatureISA_K = c.T0_K-c.lapse_Kpm*altitude_m;
temperature_K = temperatureISA_K+model.atmosphere.ISADeviation_K;
if temperature_K <= 0
    error('c172sAtmosphereAirData:InvalidTemperature', ...
        'Atmospheric temperature must remain positive.');
end
pressure_Pa = c.p0_Pa*(temperatureISA_K/c.T0_K)^( ...
    c.g_mps2/(c.Rair_JpkgK*c.lapse_Kpm));
density_kgpm3 = pressure_Pa/(c.Rair_JpkgK*temperature_K);
speedOfSound_mps = sqrt(c.gamma*c.Rair_JpkgK*temperature_K);
dynamicViscosity_Pas = c.sutherlandReferenceViscosity_Pas* ...
    (temperature_K/c.sutherlandReference_K)^(3/2)* ...
    (c.sutherlandReference_K+c.sutherlandConstant_K)/ ...
    (temperature_K+c.sutherlandConstant_K);

velocityGroundBody_mps = state(1:3);
velocityGroundNED_mps = bodyToNED*velocityGroundBody_mps;
windNED_mps = steadyWindNED_mps+gustNED_mps;
velocityAirNED_mps = velocityGroundNED_mps-windNED_mps;
velocityAirBody_mps = bodyToNED.'*velocityAirNED_mps;
TAS_mps = norm(velocityAirBody_mps);
valid = TAS_mps >= model.airData.minimumValidAirspeed_mps;

if valid
    alpha_rad = atan2(velocityAirBody_mps(3),velocityAirBody_mps(1));
    betaArgument = velocityAirBody_mps(2)/TAS_mps;
    betaArgument = min(model.airData.betaArgumentLimit, ...
        max(-model.airData.betaArgumentLimit,betaArgument));
    beta_rad = asin(betaArgument);
else
    alpha_rad = model.airData.lowSpeedAlpha_rad;
    beta_rad = model.airData.lowSpeedBeta_rad;
end

dynamicPressure_Pa = 0.5*density_kgpm3*TAS_mps^2;
Mach = TAS_mps/speedOfSound_mps;
Reynolds = density_kgpm3*TAS_mps*model.geometry.cBar_m/ ...
    dynamicViscosity_Pas;
EAS_mps = TAS_mps*sqrt(density_kgpm3/c.rho0_kgpm3);

output.altitude_m = altitude_m;
output.temperature_K = temperature_K;
output.pressure_Pa = pressure_Pa;
output.density_kgpm3 = density_kgpm3;
output.speedOfSound_mps = speedOfSound_mps;
output.dynamicViscosity_Pas = dynamicViscosity_Pas;
output.bodyToNED = bodyToNED;
output.velocityGroundBody_mps = velocityGroundBody_mps;
output.velocityGroundNED_mps = velocityGroundNED_mps;
output.steadyWindNED_mps = steadyWindNED_mps;
output.gustNED_mps = gustNED_mps;
output.windNED_mps = windNED_mps;
output.velocityAirNED_mps = velocityAirNED_mps;
output.velocityAirBody_mps = velocityAirBody_mps;
output.TAS_mps = TAS_mps;
output.EAS_mps = EAS_mps;
output.alpha_rad = alpha_rad;
output.beta_rad = beta_rad;
output.dynamicPressure_Pa = dynamicPressure_Pa;
output.Mach = Mach;
output.Reynolds = Reynolds;
output.valid = valid;

end

function matrix = quaternionToDCM(q)
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
matrix = [ ...
    q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2); ...
    2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1); ...
    2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end
