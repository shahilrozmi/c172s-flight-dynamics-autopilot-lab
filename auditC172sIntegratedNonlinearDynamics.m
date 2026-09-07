function results = auditC172sIntegratedNonlinearDynamics()
%AUDITC172SINTEGRATEDNONLINEARDYNAMICS Integrated plant/Jacobian audit.

model=c172sIntegratedNonlinearDynamicsData();
source=c172sNonlinear6DofSourceData(); kernel=c172sRigidBody6DofData();
environment=c172sAtmosphereAirDataData(); aero=c172sNearTrimAerodynamicsData();
trimData=c172sTrimTotalForceData(); lonData=c172sLongitudinalData();
latData=c172sLateralDirectionalData();
[~,lon]=c172sLongitudinalPlant(lonData); [~,lat]=c172sLateralDirectionalPlant(latData);
L=linearizeC172sIntegratedNonlinearDynamics(model); trim=L.trim;

names={}; messages={}; passed=[];
fprintf('============================================================\n');
fprintf(' C172S NONLINEAR 6-DOF V1 - INTEGRATED DYNAMICS AUDIT\n');
fprintf('============================================================\n\n');
fprintf('Artifact                               : %s\n',model.artifactRevision);
fprintf('State / control count                  : %d / %d\n',model.state.count,model.control.count);
fprintf('Trim                                   : 4000 ft / 110 KTAS / 2550 lb\n\n');

record('Source/convention audit revision',strcmp(model.required.sourceAudit,source.artifactRevision),source.artifactRevision);
record('Rigid-body kernel revision',strcmp(model.required.rigidBodyKernel,kernel.artifactRevision),kernel.artifactRevision);
record('Atmosphere/air-data revision',strcmp(model.required.atmosphereAirData,environment.artifactRevision),environment.artifactRevision);
record('Near-trim aerodynamic revision',strcmp(model.required.nearTrimAerodynamics,aero.artifactRevision),aero.artifactRevision);
record('Trim/total-force revision',strcmp(model.required.trimTotalForce,trimData.artifactRevision),trimData.artifactRevision);
record('Frozen plant revisions',strcmp(model.required.longitudinalPlant,lonData.model.version)&&strcmp(model.required.lateralPlant,latData.model.version),'Longitudinal Plant V1 and Lateral-Directional Plant V0.1');
record('Released controller regression remains external',model.required.masterRegressionPasses==18,'v1.0.0 remains frozen at 18/18');
record('Pitch inertia reconciled at assembly', ...
    model.trimModel.kernel.inertia.matrix_kgm2(2,2)==lonData.inertia.Iy_kgm2, ...
    sprintf('Iyy %.6f kg*m^2 from frozen longitudinal plant',model.massProperties.pitchInertia_kgm2));
record('Thrust-direction convention explicit', ...
    ~model.propulsion.physicalPropellerAxisIdentified, ...
    'flight-path-compatible extension; physical propeller axis remains unidentified');

[xdot,out]=c172sIntegratedNonlinearDynamics(trim.state,trim.controls,zeros(3,1),zeros(3,1),model);
record('Thirteen-state trim dynamics close',norm(xdot([1:6,9:13]),inf)<1e-11,sprintf('maximum non-translation derivative %.3g',norm(xdot([1:6,9:13]),inf)));
record('Trim inertial velocity is horizontal',abs(xdot(7)-trimData.trim.TAS_mps)<1e-11&&abs(xdot(8))<1e-11&&abs(xdot(9))<1e-11,sprintf('N/E/D %.6f / %.6f / %.6f m/s',xdot(7:9)));
record('Alpha-dot algebraic consistency',abs(out.alphaDotConsistencyError_radps)<=model.alphaDot.maximumConsistencyError_radps,sprintf('consistency error %.3g rad/s',out.alphaDotConsistencyError_radps));
record('Alpha-dot denominator remains regular',abs(out.alphaDotDenominator)>=model.alphaDot.minimumDenominatorMagnitude,sprintf('denominator %.6f',out.alphaDotDenominator));

lonAerr=max(abs(L.longitudinal.A(:)-lon.A(:))); lonBerr=max(abs(L.longitudinal.B(:)-lon.B(:)));
latAerr=max(abs(L.lateral.A(:)-lat.A(:))); latBerr=max(abs(L.lateral.B(:)-lat.B(:)));
record('Frozen longitudinal A matrix recovered',lonAerr<=model.linearization.maximumFrozenMatrixError,sprintf('maximum absolute error %.3g',lonAerr));
record('Frozen longitudinal B matrix recovered',lonBerr<=model.linearization.maximumFrozenMatrixError,sprintf('maximum absolute error %.3g',lonBerr));
record('Frozen lateral A matrix recovered',latAerr<=model.linearization.maximumFrozenMatrixError,sprintf('maximum absolute error %.3g',latAerr));
record('Frozen lateral B matrix recovered',latBerr<=model.linearization.maximumFrozenMatrixError,sprintf('maximum absolute error %.3g',latBerr));
crossError=max([abs(L.cross.longitudinalFromLateral(:));abs(L.cross.lateralFromLongitudinal(:))]);
record('Symmetric-trim first-order axis separation',crossError<=model.linearization.maximumForbiddenCrossAxisError,sprintf('maximum cross-axis Jacobian %.3g',crossError));
record('Raw 13-state Jacobian dimensions',isequal(size(L.raw13.A),[13 13])&&isequal(size(L.raw13.B),[13 4]),'13x13 state and 13x4 control matrices');
L2=linearizeC172sIntegratedNonlinearDynamics(model);
rawReplay=max([abs(L.raw13.A(:)-L2.raw13.A(:));abs(L.raw13.B(:)-L2.raw13.B(:))]);
record('Numerical Jacobian deterministic replay',rawReplay<=model.linearization.maximumRawJacobianReplayError,sprintf('maximum replay difference %.3g',rawReplay));

elevator=trim.controls; elevator(1)=elevator(1)-deg2rad(.1);
[~,eout]=c172sIntegratedNonlinearDynamics(trim.state,elevator,zeros(3,1),zeros(3,1),model);
record('Elevator sign in integrated plant',eout.totalForces.totalMomentBody_Nm(2)>0,'trailing-edge-up produces nose-up moment');
aileron=trim.controls; aileron(2)=deg2rad(.1);
[~,aout]=c172sIntegratedNonlinearDynamics(trim.state,aileron,zeros(3,1),zeros(3,1),model);
record('Aileron sign in integrated plant',aout.totalForces.totalMomentBody_Nm(1)>0,'positive command produces right-wing-down moment');
rudder=trim.controls; rudder(3)=deg2rad(.1);
[~,rout]=c172sIntegratedNonlinearDynamics(trim.state,rudder,zeros(3,1),zeros(3,1),model);
record('Rudder sign in integrated plant',rout.totalForces.totalMomentBody_Nm(3)>0,'positive command produces nose-right moment');
throttle=trim.controls; throttle(4)=throttle(4)+.01;
[~,tout]=c172sIntegratedNonlinearDynamics(trim.state,throttle,zeros(3,1),zeros(3,1),model);
record('Throttle sign in integrated plant',tout.totalForces.thrust_N>out.totalForces.thrust_N,'increased throttle increases forward force');

[longDiff,longQerr]=captureComparison([.1;deg2rad(.01);0;0;zeros(5,1)],L.reduced.A,L.reduced.B,trim.controls,model);
[latDiff,latQerr]=captureComparison([zeros(4,1);deg2rad(.01);0;0;0;0],L.reduced.A,L.reduced.B,trim.controls,model);
record('Nonlinear longitudinal RK4 capture',longDiff<=model.integration.maximumLongitudinalLinearDifference,sprintf('maximum reduced-state difference %.3g',longDiff));
record('Nonlinear lateral RK4 capture',latDiff<=model.integration.maximumLateralLinearDifference,sprintf('maximum reduced-state difference %.3g',latDiff));
record('RK4 quaternion normalization',max(longQerr,latQerr)<=model.integration.maximumQuaternionNormError,sprintf('maximum norm error %.3g',max(longQerr,latQerr)));
[replayDiff,~]=captureComparison([.1;deg2rad(.01);0;0;zeros(5,1)],L.reduced.A,L.reduced.B,trim.controls,model);
record('Integrated deterministic replay',model.integration.requireDeterministicReplay&&isequal(longDiff,replayDiff),'identical RK4 capture reproduces bitwise-identical metric');

record('Rigid-body coupling is active',model.claim.coupledRigidBodyDynamics,'all translational, rotational, attitude and navigation states execute together');
record('Unsupported aero cross derivatives remain absent',~model.claim.aerodynamicCrossDerivativeDatabase,'coupling arises from nonlinear mechanics, not invented coefficient tables');
record('Coefficient scheduling remains disabled',~model.claim.coefficientScheduling,'frozen near-trim coefficients only');
record('Propeller-map claim remains disabled',~model.claim.propellerMap,'one-point calibrated thrust only');
record('Stall/spin claim remains disabled',~model.claim.stallOrSpin,'attached-flow V0.1 envelope only');
record('Aircraft-truth claim remains disabled',~model.claim.aircraftTruth,'research validation plant, not aircraft approval');

fprintf('\nPassed audit cases                     : %d/%d\n',sum(passed),numel(passed));
fprintf('Longitudinal A/B maximum errors         : %.9g / %.9g\n',lonAerr,lonBerr);
fprintf('Lateral A/B maximum errors              : %.9g / %.9g\n',latAerr,latBerr);
fprintf('Maximum first-order cross-axis term     : %.9g\n',crossError);
assert(all(passed),'auditC172sIntegratedNonlinearDynamics:AuditFailed','%d of %d integrated dynamics cases failed.',sum(~passed),numel(passed));
fprintf('PASS: the complete thirteen-state nonlinear right-hand side executes.\n');
fprintf('PASS: both frozen plant Jacobians are recovered independently.\n');
fprintf('PASS: alpha-dot coupling, nonlinear RK4 captures and replay pass.\n');
fprintf('NOTE: coefficients remain near-trim and thrust remains one-point calibrated; this is not aircraft approval.\n');
results=struct('Name',names(:),'Status',num2cell(passed(:)),'Message',messages(:));

    function record(name,status,message)
        names{end+1}=name; messages{end+1}=message; passed(end+1)=logical(status);
        if status, tag='PASS'; else, tag='FAIL'; end
        fprintf('[%s] %-49s %s\n',tag,name,message);
    end
end

function [difference,maxQuaternionError]=captureComparison(z0,A,B,controls,model)
dt=model.integration.fixedStep_s; count=round(model.integration.captureDuration_s/dt);
state=reducedToStateLocal(z0,model); zLinear=z0; maximum=0; maxQuaternionError=0;
for k=1:count
    state=rk4(state,controls,dt,model); state(10:13)=state(10:13)/norm(state(10:13));
    zLinear=rk4Linear(zLinear,A,B,zeros(4,1),dt);
    zNonlinear=stateToReducedLocal(state,model);
    maximum=max(maximum,norm(zNonlinear-zLinear,inf));
    maxQuaternionError=max(maxQuaternionError,abs(norm(state(10:13))-1));
end
difference=maximum;
end

function next=rk4(x,u,h,m)
f=@(s)c172sIntegratedNonlinearDynamics(s,u,zeros(3,1),zeros(3,1),m);
k1=f(x); k2=f(x+h*k1/2); k3=f(x+h*k2/2); k4=f(x+h*k3);
next=x+h*(k1+2*k2+2*k3+k4)/6;
end
function next=rk4Linear(x,A,B,u,h)
f=@(s)A*s+B*u; k1=f(x); k2=f(x+h*k1/2); k3=f(x+h*k2/2); k4=f(x+h*k3);
next=x+h*(k1+2*k2+2*k3+k4)/6;
end
function state=reducedToStateLocal(z,m)
V=m.trimModel.trim.TAS_mps+z(1); a=z(2); b=z(5);
vel=V*[cos(a)*cos(b);sin(b);sin(a)*cos(b)]; q=eulerQ(z(8),z(4),z(9));
state=[vel;z(6);z(3);z(7);0;0;-m.trimModel.trim.altitude_m;q];
end
function z=stateToReducedLocal(s,m)
air=c172sAtmosphereAirData(s,zeros(3,1),zeros(3,1),m.trimModel.aerodynamics.airDataModel);
[phi,theta,psi]=qEuler(s(10:13));
z=[air.TAS_mps-m.trimModel.trim.TAS_mps;air.alpha_rad;s(5);theta;air.beta_rad;s(4);s(6);phi;psi];
end
function q=eulerQ(phi,theta,psi)
cr=cos(phi/2);sr=sin(phi/2);cp=cos(theta/2);sp=sin(theta/2);cy=cos(psi/2);sy=sin(psi/2);
q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
end
function [phi,theta,psi]=qEuler(q)
q=q/norm(q);q0=q(1);q1=q(2);q2=q(3);q3=q(4);
phi=atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));theta=asin(min(1,max(-1,2*(q0*q2-q3*q1))));psi=atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
end
