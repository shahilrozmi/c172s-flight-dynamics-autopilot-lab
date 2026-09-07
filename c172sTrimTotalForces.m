function output = c172sTrimTotalForces( ...
    state,controls,alphaDot_radps,steadyWindNED_mps,gustNED_mps,model)
%C172STRIMTOTALFORCES Total near-trim aero/propulsive force and moment.

validateattributes(controls,{'double'}, ...
    {'real','finite','column','numel',4},mfilename,'controls',2);
if any(controls < model.controls.lower) || any(controls > model.controls.upper)
    error('c172sTrimTotalForces:ControlOutsideBounds', ...
        'One or more controls lie outside the declared V0.1 bounds.');
end

aero = c172sNearTrimAerodynamics(state,controls(1:3), ...
    alphaDot_radps,steadyWindNED_mps,gustNED_mps,model.aerodynamics);
qbarS = aero.airData.dynamicPressure_Pa*model.aerodynamics.geometry.S_m2;
lift_N = qbarS*aero.coefficients.CL;
drag_N = qbarS*aero.coefficients.CD;
sideForce_N = qbarS*aero.coefficients.CY;
alpha = aero.airData.alpha_rad;

aerodynamicForceBody_N = [ ...
    -drag_N*cos(alpha)+lift_N*sin(alpha); ...
    sideForce_N; ...
    -drag_N*sin(alpha)-lift_N*cos(alpha)];
propulsiveForceBody_N = [controls(4)* ...
    model.propulsion.maximumThrustAtTrim_N;0;0];
propulsiveMomentBody_Nm = zeros(3,1);
totalForceBody_N = aerodynamicForceBody_N+propulsiveForceBody_N;
totalMomentBody_Nm = aero.momentBody_Nm+propulsiveMomentBody_Nm;
[stateDerivative,rigidBody] = c172sRigidBody6DofEom( ...
    state,totalForceBody_N,totalMomentBody_Nm,model.kernel);

output.totalForceBody_N = totalForceBody_N;
output.totalMomentBody_Nm = totalMomentBody_Nm;
output.aerodynamicForceBody_N = aerodynamicForceBody_N;
output.aerodynamicMomentBody_Nm = aero.momentBody_Nm;
output.propulsiveForceBody_N = propulsiveForceBody_N;
output.propulsiveMomentBody_Nm = propulsiveMomentBody_Nm;
output.lift_N = lift_N;
output.drag_N = drag_N;
output.sideForce_N = sideForce_N;
output.thrust_N = propulsiveForceBody_N(1);
output.stateDerivative = stateDerivative;
output.aerodynamics = aero;
output.rigidBody = rigidBody;

end
