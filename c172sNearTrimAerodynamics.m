function output = c172sNearTrimAerodynamics( ...
    state,controls_rad,alphaDot_radps,steadyWindNED_mps,gustNED_mps,model)
%C172SNEARTRIMAERODYNAMICS Near-trim incremental forces and moments.
%
% This compatibility layer evaluates the frozen linear aerodynamic
% coefficients using nonlinear dynamic pressure, air angles and normalized
% rates. It is not an attached-flow schedule or stall model.

validateattributes(state,{'double'}, ...
    {'real','finite','column','numel',13},mfilename,'state',1);
validateattributes(controls_rad,{'double'}, ...
    {'real','finite','column','numel',3},mfilename,'controls_rad',2);
validateattributes(alphaDot_radps,{'double'}, ...
    {'real','finite','scalar'},mfilename,'alphaDot_radps',3);

air = c172sAtmosphereAirData( ...
    state,steadyWindNED_mps,gustNED_mps,model.airDataModel);
if ~air.valid
    error('c172sNearTrimAerodynamics:InvalidAirData', ...
        'Near-trim aerodynamics requires valid airspeed.');
end
if air.TAS_mps < model.envelope.V_mps(1) || ...
        air.TAS_mps > model.envelope.V_mps(2)
    error('c172sNearTrimAerodynamics:AirspeedOutsideEnvelope', ...
        'TAS %.3f m/s lies outside the V0.1 attached-flow boundary.', ...
        air.TAS_mps);
end
if air.alpha_rad < model.envelope.alpha_rad(1) || ...
        air.alpha_rad > model.envelope.alpha_rad(2)
    error('c172sNearTrimAerodynamics:AlphaOutsideEnvelope', ...
        'Angle of attack lies outside the V0.1 boundary.');
end
if air.beta_rad < model.envelope.beta_rad(1) || ...
        air.beta_rad > model.envelope.beta_rad(2)
    error('c172sNearTrimAerodynamics:BetaOutsideEnvelope', ...
        'Sideslip lies outside the V0.1 boundary.');
end

deltaE = controls_rad(1);
deltaA = controls_rad(2);
deltaR = controls_rad(3);
p = state(4);
q = state(5);
r = state(6);
V = air.TAS_mps;
qbarS = air.dynamicPressure_Pa*model.geometry.S_m2;
qbarS0 = model.trim.qbar_Pa*model.geometry.S_m2;

pHat = p*model.geometry.b_m/(2*V);
qHat = q*model.geometry.cBar_m/(2*V);
rHat = r*model.geometry.b_m/(2*V);
alphaDotHat = alphaDot_radps*model.geometry.cBar_m/(2*V);

lon = model.longitudinal.aero;
lat = model.lateral.aero;

CL = model.trim.CL + lon.CL_alpha*air.alpha_rad + ...
    lon.CL_q*qHat + lon.CL_alphaDot*alphaDotHat + ...
    lon.CL_deltaE*deltaE;
CD = model.trim.CD + lon.CD_alpha*air.alpha_rad;
Cm = lon.Cm_alpha*air.alpha_rad + lon.Cm_q*qHat + ...
    lon.Cm_alphaDot*alphaDotHat + lon.Cm_deltaE*deltaE;

CY = lat.CY_beta*air.beta_rad + lat.CY_p*pHat + lat.CY_r*rHat + ...
    lat.CY_deltaA*deltaA + lat.CY_deltaR*deltaR;
Cl = lat.Cl_beta*air.beta_rad + lat.Cl_p*pHat + lat.Cl_r*rHat + ...
    lat.Cl_deltaA*deltaA + lat.Cl_deltaR*deltaR;
Cn = lat.Cn_beta*air.beta_rad + lat.Cn_p*pHat + lat.Cn_r*rHat + ...
    lat.Cn_deltaA*deltaA + lat.Cn_deltaR*deltaR;

liftIncrement_N = qbarS*CL-qbarS0*model.trim.CL;
dragIncrement_N = qbarS*CD-qbarS0*model.trim.CD;
sideForce_N = qbarS*CY;
rollMoment_Nm = qbarS*model.geometry.b_m*Cl;
pitchMoment_Nm = qbarS*model.geometry.cBar_m*Cm;
yawMoment_Nm = qbarS*model.geometry.b_m*Cn;

alpha = air.alpha_rad;
forceBody_N = [ ...
    -dragIncrement_N*cos(alpha)+liftIncrement_N*sin(alpha); ...
    sideForce_N; ...
    -dragIncrement_N*sin(alpha)-liftIncrement_N*cos(alpha)];
momentBody_Nm = [rollMoment_Nm;pitchMoment_Nm;yawMoment_Nm];

output.forceBody_N = forceBody_N;
output.momentBody_Nm = momentBody_Nm;
output.dragIncrement_N = dragIncrement_N;
output.liftIncrement_N = liftIncrement_N;
output.sideForce_N = sideForce_N;
output.rollMoment_Nm = rollMoment_Nm;
output.pitchMoment_Nm = pitchMoment_Nm;
output.yawMoment_Nm = yawMoment_Nm;
output.coefficients = struct('CL',CL,'CD',CD,'CY',CY, ...
    'Cl',Cl,'Cm',Cm,'Cn',Cn);
output.normalizedRates = struct('pHat',pHat,'qHat',qHat, ...
    'rHat',rHat,'alphaDotHat',alphaDotHat);
output.airData = air;

end
