function linearization = linearizeC172sIntegratedNonlinearDynamics(model)
%LINEARIZEC172SINTEGRATEDNONLINEARDYNAMICS Numerical trim Jacobians.

if nargin < 1 || isempty(model)
    model = c172sIntegratedNonlinearDynamicsData();
end
trim = solveC172sNonlinearTrim([],model.trimModel);
z0 = zeros(9,1);
u0 = trim.controls;

A = zeros(9); B = zeros(9,4);
for column = 1:9
    h = model.linearization.stateSteps(column);
    plus = z0; minus = z0;
    plus(column)=h; minus(column)=-h;
    A(:,column) = (reducedDerivative(plus,u0,model)- ...
        reducedDerivative(minus,u0,model))/(2*h);
end
for column = 1:4
    h = model.linearization.controlSteps(column);
    plus = u0; minus = u0;
    plus(column)=plus(column)+h; minus(column)=minus(column)-h;
    B(:,column) = (reducedDerivative(z0,plus,model)- ...
        reducedDerivative(z0,minus,model))/(2*h);
end

rawA = zeros(13); rawB = zeros(13,4);
for column = 1:13
    h = model.linearization.rawStateSteps(column);
    plus=trim.state; minus=trim.state;
    plus(column)=plus(column)+h; minus(column)=minus(column)-h;
    rawA(:,column) = (rhs(plus,u0,model)-rhs(minus,u0,model))/(2*h);
end
for column = 1:4
    h = model.linearization.controlSteps(column);
    plus=u0; minus=u0;
    plus(column)=plus(column)+h; minus(column)=minus(column)-h;
    rawB(:,column) = (rhs(trim.state,plus,model)- ...
        rhs(trim.state,minus,model))/(2*h);
end

linearization.artifactRevision = model.artifactRevision;
linearization.trim = trim;
linearization.reduced.A = A;
linearization.reduced.B = B;
linearization.reduced.stateNames = model.linearization.reducedStateNames;
linearization.reduced.controlNames = model.linearization.controlNames;
linearization.raw13.A = rawA;
linearization.raw13.B = rawB;
linearization.raw13.stateNames = model.state.names;
linearization.raw13.controlNames = model.control.names;
linearization.longitudinal.A = A(1:4,1:4);
linearization.longitudinal.B = B(1:4,1);
linearization.lateral.A = A(5:9,5:9);
linearization.lateral.B = B(5:9,2:3);
linearization.cross.longitudinalFromLateral = A(1:4,5:9);
linearization.cross.lateralFromLongitudinal = A(5:9,1:4);

end

function derivative = rhs(state,controls,model)
derivative = c172sIntegratedNonlinearDynamics( ...
    state,controls,zeros(3,1),zeros(3,1),model);
end

function derivative = reducedDerivative(z,controls,model)
state = reducedToState(z,model);
[xdot,out] = c172sIntegratedNonlinearDynamics( ...
    state,controls,zeros(3,1),zeros(3,1),model);
air = out.totalForces.aerodynamics.airData;
v = air.velocityAirBody_mps;
a = out.airAccelerationBody_mps2;
V = air.TAS_mps;
Vdot = dot(v,a)/V;
betaArgument = min(1,max(-1,v(2)/V));
betaDenominator = max(1e-12,sqrt(1-betaArgument^2));
betaDot = (a(2)*V-v(2)*Vdot)/(V^2*betaDenominator);

[phi,theta,~] = quaternionToEuler(state(10:13));
p=state(4); q=state(5); r=state(6);
phiDot = p+tan(theta)*(q*sin(phi)+r*cos(phi));
thetaDot = q*cos(phi)-r*sin(phi);
psiDot = (q*sin(phi)+r*cos(phi))/cos(theta);
derivative = [Vdot;out.alphaDot_radps;xdot(5);thetaDot; ...
    betaDot;xdot(4);xdot(6);phiDot;psiDot];
end

function state = reducedToState(z,model)
V=model.trimModel.trim.TAS_mps+z(1);
alpha=z(2); beta=z(5);
velocity=V*[cos(alpha)*cos(beta);sin(beta);sin(alpha)*cos(beta)];
q=eulerToQuaternion(z(8),z(4),z(9));
state=[velocity;z(6);z(3);z(7);0;0; ...
    -model.trimModel.trim.altitude_m;q];
end

function q = eulerToQuaternion(phi,theta,psi)
cr=cos(phi/2); sr=sin(phi/2); cp=cos(theta/2); sp=sin(theta/2);
cy=cos(psi/2); sy=sin(psi/2);
q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy; ...
    cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
end

function [phi,theta,psi] = quaternionToEuler(q)
q=q/norm(q); q0=q(1); q1=q(2); q2=q(3); q3=q(4);
phi=atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
theta=asin(min(1,max(-1,2*(q0*q2-q3*q1))));
psi=atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
end
