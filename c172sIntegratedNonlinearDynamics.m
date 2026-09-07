function [stateDerivative,output] = c172sIntegratedNonlinearDynamics( ...
    state,controls,steadyWindNED_mps,gustNED_mps,model)
%C172SINTEGRATEDNONLINEARDYNAMICS Coupled 13-state V0.1 plant RHS.
%
% Alpha-dot aerodynamic terms are retained through an exact scalar
% algebraic consistency solve. Wind and gust inputs are treated as constant
% NED air-mass velocities during this right-hand-side evaluation.

validateattributes(state,{'double'}, ...
    {'real','finite','column','numel',13},mfilename,'state',1);
validateattributes(controls,{'double'}, ...
    {'real','finite','column','numel',4},mfilename,'controls',2);
validateattributes(steadyWindNED_mps,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'steadyWindNED_mps',3);
validateattributes(gustNED_mps,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'gustNED_mps',4);

zero = evaluateIntegratedForces(state,controls,0, ...
    steadyWindNED_mps,gustNED_mps,model);
unit = evaluateIntegratedForces(state,controls,1, ...
    steadyWindNED_mps,gustNED_mps,model);

windBody = zero.aerodynamics.airData.bodyToNED.'* ...
    (steadyWindNED_mps+gustNED_mps);
airVelocity = zero.aerodynamics.airData.velocityAirBody_mps;
omega = state(4:6);
alphaRateZero = kinematicAlphaRate(airVelocity, ...
    zero.stateDerivative(1:3)+cross(omega,windBody));
alphaRateUnit = kinematicAlphaRate(airVelocity, ...
    unit.stateDerivative(1:3)+cross(omega,windBody));
slope = alphaRateUnit-alphaRateZero;
denominator = 1-slope;
if abs(denominator) < model.alphaDot.minimumDenominatorMagnitude
    error('c172sIntegratedNonlinearDynamics:AlphaDotSingularity', ...
        'The alpha-dot algebraic denominator is too small.');
end
alphaDot = alphaRateZero/denominator;

total = evaluateIntegratedForces(state,controls,alphaDot, ...
    steadyWindNED_mps,gustNED_mps,model);
stateDerivative = total.stateDerivative;
airAcceleration = stateDerivative(1:3)+cross(omega,windBody);
alphaDotCheck = kinematicAlphaRate(airVelocity,airAcceleration);

output.alphaDot_radps = alphaDot;
output.alphaDotConsistencyError_radps = alphaDot-alphaDotCheck;
output.alphaDotAffineSlope = slope;
output.alphaDotDenominator = denominator;
output.totalForces = total;
output.airAccelerationBody_mps2 = airAcceleration;
output.modelArtifactRevision = model.artifactRevision;

end

function total = evaluateIntegratedForces( ...
    state,controls,alphaDot,steadyWindNED,gustNED,model)
% Extend the one-point thrust balance using the declared compatibility axis.
total = c172sTrimTotalForces(state,controls,alphaDot, ...
    steadyWindNED,gustNED,model.trimModel);
alpha = total.aerodynamics.airData.alpha_rad;
thrust = total.thrust_N;
propulsiveForce = thrust*[cos(alpha);0;sin(alpha)];
total.propulsiveForceBody_N = propulsiveForce;
total.totalForceBody_N = total.aerodynamicForceBody_N+propulsiveForce;
[total.stateDerivative,total.rigidBody] = c172sRigidBody6DofEom( ...
    state,total.totalForceBody_N,total.totalMomentBody_Nm,model.trimModel.kernel);
end

function alphaDot = kinematicAlphaRate(velocity,acceleration)
u = velocity(1); w = velocity(3);
denominator = u^2+w^2;
if denominator < 1e-8
    error('c172sIntegratedNonlinearDynamics:AlphaUndefined', ...
        'Longitudinal airspeed is too small to define alpha-dot.');
end
alphaDot = (u*acceleration(3)-w*acceleration(1))/denominator;
end
