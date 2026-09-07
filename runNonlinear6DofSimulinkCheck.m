function results=runNonlinear6DofSimulinkCheck()
%RUNNONLINEAR6DOFSIMULINKCHECK Executable nonlinear plant equivalence.
clc; close all; load_system('simulink');
model=c172sIntegratedNonlinearDynamicsData(); trim=solveC172sNonlinearTrim([],model.trimModel);
modelName=buildC172sNonlinear6DofSimulink(true); dt=0.01; stopTime=2.0;
plantPath=[modelName '/Independent Nonlinear 6DOF Plant']; root=sfroot;
chart=root.find('-isa','Stateflow.EMChart','Path',plantPath); source=chart.Script;
for forbidden={'c172sIntegratedNonlinearDynamics','c172sTrimTotalForces','c172sRigidBody6DofEom','c172sAtmosphereAirData'}
    assert(~contains(source,forbidden{1}),'Executable plant calls reference function %s.',forbidden{1});
end
assert(strcmp(get_param(modelName,'Solver'),'ode4')&&strcmp(get_param(modelName,'SolverType'),'Fixed-step'),'Generated plant solver changed.');
assert(abs(str2double(get_param(modelName,'FixedStep'))-dt)<eps,'Generated fixed step changed.');

scenarios=scenarioLedger(trim,model); captures=cell(numel(scenarios),1);
for k=1:numel(scenarios), captures{k}=runCapture(modelName,scenarios(k),model,dt,stopTime); end
tolerance=2e-7;
for k=1:numel(captures)
    assert(captures{k}.maximumError<=tolerance,'%s maximum error %.9g exceeds %.9g.',scenarios(k).name,captures{k}.maximumError,tolerance);
end
repeat=runCapture(modelName,scenarios(4),model,dt,stopTime);
assert(isequal(captures{4}.simulated,repeat.simulated),'Repeated combined capture was not bitwise identical.');
trimCap=captures{1};
dynamicIndex=[1:6 9:13];
assert(max(abs(trimCap.simulated(:,dynamicIndex)-trimCap.simulated(1,dynamicIndex)),[],'all')<2e-9,'Executable trim state drift exceeded its gate.');
allQuaternionError=0; allConsistency=0;
for k=1:numel(captures)
    q=captures{k}.simulated(:,10:13); allQuaternionError=max(allQuaternionError,max(abs(sqrt(sum(q.^2,2))-1)));
    allConsistency=max(allConsistency,max(abs(captures{k}.simulated(:,14)-captures{k}.reference(:,14))));
end
assert(allQuaternionError<2e-9,'Executable quaternion norm error %.9g exceeded its gate.',allQuaternionError);

figure('Name','C172S Nonlinear 6-DOF V1 - executable captures','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact'); c=captures{4};
nexttile; plot(c.time,c.simulated(:,1),'LineWidth',1.4);hold on;plot(c.time,c.reference(:,1),'--','LineWidth',1.1);grid on;xlabel('Time (s)');ylabel('u (m/s)');title('Combined: forward velocity');legend('Simulink','Independent RK4');
nexttile; plot(c.time,rad2deg(c.simulated(:,5)),'LineWidth',1.4);hold on;plot(c.time,rad2deg(c.reference(:,5)),'--','LineWidth',1.1);grid on;xlabel('Time (s)');ylabel('q (deg/s)');title('Pitch rate');
nexttile; plot(c.time,rad2deg(c.simulated(:,4)),'LineWidth',1.4);hold on;plot(c.time,rad2deg(c.reference(:,4)),'--','LineWidth',1.1);grid on;xlabel('Time (s)');ylabel('p (deg/s)');title('Roll rate');
nexttile; plot(c.time,rad2deg(c.simulated(:,6)),'LineWidth',1.4);hold on;plot(c.time,rad2deg(c.reference(:,6)),'--','LineWidth',1.1);grid on;xlabel('Time (s)');ylabel('r (deg/s)');title('Yaw rate');

fprintf('============================================================\n');fprintf(' C172S NONLINEAR 6-DOF V1 - EXECUTABLE SIMULINK CHECK\n');fprintf('============================================================\n\n');
fprintf('Model                                  : %s\n',modelName);fprintf('Artifact                               : %s\n',model.artifactRevision);fprintf('Solver / fixed step                    : ode4 / %.4f s\n',dt);fprintf('Verification signals                   : 23 = 13 states + 10 diagnostics\n\n');
for k=1:numel(captures),fprintf('[PASS] %-34s maximum error %.9g\n',scenarios(k).name,captures{k}.maximumError);end
fprintf('\nPassed executable captures             : %d/%d\n',numel(captures),numel(captures));
fprintf('Maximum quaternion norm error          : %.9g\n',allQuaternionError);
fprintf('Maximum alpha-dot implementation error : %.9g rad/s\n',allConsistency);
fprintf('PASS: Simulink matches independent hybrid RK4 references.\n');
fprintf('PASS: trim, coupled perturbations, controls, wind and gust execute.\n');
fprintf('PASS: the executable block does not call the MATLAB reference plant.\n');
fprintf('NOTE: near-trim coefficients and one-point thrust limits remain; this is not aircraft approval.\n');
results=repmat(struct('Name','','MaximumError',0),numel(captures),1);
for k=1:numel(captures)
    results(k).Name=scenarios(k).name;
    results(k).MaximumError=captures{k}.maximumError;
end
try, close_system(modelName,0);load_system([modelName '.slx']);open_system(modelName);fprintf('Reloaded generated model to clear transient editor diagnostics.\n');catch,end
end

function scenarios=scenarioLedger(trim,model)
base=struct('name','','state',trim.state,'input',[trim.controls;zeros(6,1)]);
scenarios=repmat(base,6,1);
scenarios(1).name='trim equilibrium';
scenarios(2).name='longitudinal perturbation';scenarios(2).state=stateFromPerturbation([.1;deg2rad(.01);0;0;zeros(5,1)],model);
scenarios(3).name='lateral perturbation';scenarios(3).state=stateFromPerturbation([zeros(4,1);deg2rad(.01);0;0;0;0],model);
scenarios(4).name='combined six-DOF perturbation';scenarios(4).state=stateFromPerturbation([.08;deg2rad(.008);deg2rad(.01);deg2rad(.005);deg2rad(.008);deg2rad(.01);deg2rad(-.008);deg2rad(.006);0],model);
scenarios(5).name='combined control offsets';scenarios(5).input(1:4)=trim.controls+[deg2rad(-.05);deg2rad(.04);deg2rad(.03);.002];
scenarios(6).name='steady wind plus gust';scenarios(6).input(5:10)=[-1.0;.3;0;.2;-.1;-.15];
end

function capture=runCapture(modelName,scenario,model,dt,stopTime)
assignin('base','n6InitialState',scenario.state);assignin('base','n6InputSignal',[0 scenario.input.';stopTime scenario.input.']);
out=sim(modelName,'StopTime',num2str(stopTime),'ReturnWorkspaceOutputs','on');t=out.get('tout');y=normalizeLoggedOutput(out.get('yout'),t);
reference=zeros(numel(t),23);state=scenario.state;reference(1,:)=referenceRow(state,scenario.input,model);
for k=1:numel(t)-1
    h=t(k+1)-t(k);state=rk4(state,scenario.input,h,model);reference(k+1,:)=referenceRow(state,scenario.input,model);
end
assert(size(y,2)==23&&size(y,1)==size(reference,1), ...
    'Executable verification interface changed: received %dx%d, expected %dx23.', ...
    size(y,1),size(y,2),size(reference,1));
capture.time=t;capture.simulated=y;capture.reference=reference;capture.maximumError=max(abs(y-reference),[],'all');
end

function y=normalizeLoggedOutput(raw,t)
if isa(raw,'Simulink.SimulationData.Dataset')
    element=raw.getElement(1); raw=element.Values;
end
if isa(raw,'timeseries')
    raw=raw.Data;
elseif isstruct(raw)
    if isfield(raw,'signals')
        raw=raw.signals.values;
    elseif isfield(raw,'Values')
        raw=raw.Values.Data;
    end
end
y=squeeze(raw);
if isvector(y)&&numel(t)==1, y=reshape(y,1,[]); end
if size(y,1)==23&&size(y,2)==numel(t), y=y.'; end
if size(y,2)==24&&size(y,1)==numel(t)&& ...
        max(abs(y(:,1)-t(:)))<1e-10
    y=y(:,2:end);
end
end

function next=rk4(x,input,h,m)
f=@(s)c172sIntegratedNonlinearDynamics(s,input(1:4),input(5:7),input(8:10),m);
k1=f(x);k2=f(x+h*k1/2);k3=f(x+h*k2/2);k4=f(x+h*k3);next=x+h*(k1+2*k2+2*k3+k4)/6;
end
function row=referenceRow(x,input,m)
[~,o]=c172sIntegratedNonlinearDynamics(x,input(1:4),input(5:7),input(8:10),m);t=o.totalForces;a=t.aerodynamics.airData;
row=[x;o.alphaDot_radps;t.totalForceBody_N;t.totalMomentBody_Nm;a.TAS_mps;a.alpha_rad;a.beta_rad].';
end
function state=stateFromPerturbation(z,m)
V=m.trimModel.trim.TAS_mps+z(1);a=z(2);b=z(5);vel=V*[cos(a)*cos(b);sin(b);sin(a)*cos(b)];q=eulerQ(z(8),z(4),z(9));state=[vel;z(6);z(3);z(7);0;0;-m.trimModel.trim.altitude_m;q];
end
function q=eulerQ(phi,theta,psi)
cr=cos(phi/2);sr=sin(phi/2);cp=cos(theta/2);sp=sin(theta/2);cy=cos(psi/2);sy=sin(psi/2);q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
end
