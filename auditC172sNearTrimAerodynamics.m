function results = auditC172sNearTrimAerodynamics()
%% C172S nonlinear 6-DOF V1 - near-trim aerodynamic Jacobian audit

clear;
clc;

aero = c172sNearTrimAerodynamicsData();
source = c172sNonlinear6DofSourceData();
kernel = c172sRigidBody6DofData();
airData = c172sAtmosphereAirDataData();
longitudinal = c172sLongitudinalData();
lateral = c172sLateralDirectionalData();
[~,longPlant] = c172sLongitudinalPlant(longitudinal);
[~,latPlant] = c172sLateralDirectionalPlant(lateral);

name = strings(0,1);
detail = strings(0,1);
passed = false(0,1);

record('Source/convention audit revision', ...
    strcmp(aero.required.sourceAudit,source.artifactRevision), ...
    source.artifactRevision);
record('Rigid-body kernel revision', ...
    strcmp(aero.required.rigidBodyKernel,kernel.artifactRevision), ...
    kernel.artifactRevision);
record('Atmosphere/air-data revision', ...
    strcmp(aero.required.atmosphereAirData,airData.artifactRevision), ...
    airData.artifactRevision);
record('Frozen plant revisions', ...
    strcmp(aero.required.longitudinalPlant,longitudinal.model.version) && ...
    strcmp(aero.required.lateralPlant,lateral.model.version), ...
    'Longitudinal Plant V1 and Lateral-Directional Plant V0.1');
record('Released controller regression remains external', ...
    aero.required.masterRegressionPasses == 18, ...
    'v1.0.0 remains frozen at 18/18');

zeroPoint = zeros(10,1);
trimOutput = evaluatePoint(zeroPoint,aero);
trimVector = outputVector(trimOutput);
record('Incremental trim residual is zero', ...
    norm(trimVector,inf) <= aero.validation.maximumTrimResidual && ...
    norm(trimOutput.forceBody_N,inf) <= aero.validation.maximumTrimResidual, ...
    sprintf('maximum force/moment residual %.3g',norm(trimVector,inf)));
record('Equilibrium forces remain external', ...
    ~aero.output.equilibriumForcesIncluded, ...
    'future trim layer must supply weight/thrust/equilibrium balances');

steps = aero.validation.finiteDifferenceSteps;
jacobian = zeros(6,10);
for column = 1:10
    plusPoint = zeroPoint;
    minusPoint = zeroPoint;
    plusPoint(column) = steps(column);
    minusPoint(column) = -steps(column);
    jacobian(:,column) = ( ...
        outputVector(evaluatePoint(plusPoint,aero))- ...
        outputVector(evaluatePoint(minusPoint,aero)))/(2*steps(column));
end

expected = expectedDimensionalJacobian(longPlant,latPlant);
relativeJacobianError = scaledMatrixError(jacobian,expected);
record('Dimensional derivative Jacobian recovered', ...
    relativeJacobianError <= ...
    aero.validation.maximumDimensionalDerivativeRelativeError, ...
    sprintf('maximum scaled derivative error %.3g',relativeJacobianError));

longitudinalColumns = [1,2,5,7,8];
longitudinalRows = [1,2,5];
longitudinalDerivativeError = scaledMatrixError( ...
    jacobian(longitudinalRows,longitudinalColumns), ...
    expected(longitudinalRows,longitudinalColumns));
record('Longitudinal force/moment derivatives recovered', ...
    longitudinalDerivativeError <= ...
    aero.validation.maximumDimensionalDerivativeRelativeError, ...
    sprintf('maximum scaled error %.3g',longitudinalDerivativeError));

lateralColumns = [3,4,6,9,10];
lateralRows = [3,4,6];
lateralDerivativeError = scaledMatrixError( ...
    jacobian(lateralRows,lateralColumns), ...
    expected(lateralRows,lateralColumns));
record('Lateral force/moment derivatives recovered', ...
    lateralDerivativeError <= ...
    aero.validation.maximumDimensionalDerivativeRelativeError, ...
    sprintf('maximum scaled error %.3g',lateralDerivativeError));

[longA,longB] = reconstructLongitudinalMatrices( ...
    jacobian,longitudinal,aero);
longAError = max(abs(longA(:)-longPlant.A(:)));
longBError = max(abs(longB(:)-longPlant.B(:)));
record('Frozen longitudinal A matrix reconstructed', ...
    longAError <= aero.validation.maximumMatrixAbsoluteError, ...
    sprintf('maximum absolute error %.3g',longAError));
record('Frozen longitudinal B matrix reconstructed', ...
    longBError <= aero.validation.maximumMatrixAbsoluteError, ...
    sprintf('maximum absolute error %.3g',longBError));

[latA,latB] = reconstructLateralMatrices(jacobian,lateral,aero);
latAError = max(abs(latA(:)-latPlant.A(:)));
latBError = max(abs(latB(:)-latPlant.B(:)));
record('Frozen lateral A matrix reconstructed', ...
    latAError <= aero.validation.maximumMatrixAbsoluteError, ...
    sprintf('maximum absolute error %.3g',latAError));
record('Frozen lateral B matrix reconstructed', ...
    latBError <= aero.validation.maximumMatrixAbsoluteError, ...
    sprintf('maximum absolute error %.3g',latBError));

elevatorUp = zeroPoint; elevatorUp(8) = -deg2rad(1);
elevatorOutput = evaluatePoint(elevatorUp,aero);
record('Elevator sign preserved', ...
    elevatorOutput.pitchMoment_Nm > 0, ...
    'trailing-edge-up command produces positive nose-up moment');
aileronRight = zeroPoint; aileronRight(9) = deg2rad(1);
aileronOutput = evaluatePoint(aileronRight,aero);
record('Aileron sign preserved', ...
    aileronOutput.rollMoment_Nm > 0, ...
    'right-wing-down command produces positive roll moment');
rudderRight = zeroPoint; rudderRight(10) = deg2rad(1);
rudderOutput = evaluatePoint(rudderRight,aero);
record('Rudder sign preserved', ...
    rudderOutput.yawMoment_Nm > 0, ...
    'nose-right command produces positive yaw moment');

positiveAlpha = zeroPoint; positiveAlpha(2) = deg2rad(1);
alphaOutput = evaluatePoint(positiveAlpha,aero);
record('Static longitudinal stability signs', ...
    alphaOutput.liftIncrement_N > 0 && ...
    alphaOutput.dragIncrement_N > 0 && ...
    alphaOutput.pitchMoment_Nm < 0, ...
    'positive alpha increases lift/drag and gives restoring pitch moment');
positiveBeta = zeroPoint; positiveBeta(3) = deg2rad(1);
betaOutput = evaluatePoint(positiveBeta,aero);
record('Static lateral stability signs', ...
    betaOutput.sideForce_N < 0 && ...
    betaOutput.rollMoment_Nm < 0 && ...
    betaOutput.yawMoment_Nm > 0, ...
    'positive beta gives restoring side force, roll and yaw derivatives');

positiveRates = zeroPoint;
positiveRates(4:6) = [0.2;0.3;0.4];
rateOutput = evaluatePoint(positiveRates,aero);
record('Rate normalization uses current TAS', ...
    abs(rateOutput.normalizedRates.pHat- ...
    0.2*aero.geometry.b_m/(2*aero.trim.V_mps)) <= 1.0e-15 && ...
    abs(rateOutput.normalizedRates.qHat- ...
    0.3*aero.geometry.cBar_m/(2*aero.trim.V_mps)) <= 1.0e-15 && ...
    abs(rateOutput.normalizedRates.rHat- ...
    0.4*aero.geometry.b_m/(2*aero.trim.V_mps)) <= 1.0e-15, ...
    'pHat/rHat use span and qHat uses mean aerodynamic chord');

positiveAlphaDot = zeroPoint;
positiveAlphaDot(7) = 0.1;
alphaDotOutput = evaluatePoint(positiveAlphaDot,aero);
record('Alpha-dot derivatives retained', ...
    alphaDotOutput.liftIncrement_N > 0 && ...
    alphaDotOutput.pitchMoment_Nm < 0, ...
    'positive alphaDot contributes positive lift and negative pitch moment');

replay1 = evaluatePoint([0.2;0.01;-0.02;0.03;-0.04;0.05; ...
    0.01;-0.005;0.004;-0.003],aero);
replay2 = evaluatePoint([0.2;0.01;-0.02;0.03;-0.04;0.05; ...
    0.01;-0.005;0.004;-0.003],aero);
record('Deterministic replay', ...
    aero.validation.requireDeterministicReplay && isequal(replay1,replay2), ...
    'identical state/control perturbations reproduce identical outputs');

record('Attached-flow scheduling remains disabled', ...
    ~aero.envelope.schedulingImplemented && ...
    ~aero.validation.aerodynamicSchedulingIncluded, ...
    'coefficients are frozen near-trim anchors, not scheduled tables');
record('Stall and post-stall remain disabled', ...
    ~aero.envelope.stallImplemented && ~aero.envelope.postStallImplemented, ...
    'no stall, departure or spin claim enters this layer');
record('Propulsion remains excluded', ...
    ~aero.validation.propulsionIncluded, ...
    'thrust and propeller effects remain outside the aerodynamic Jacobian');

outOfAlpha = zeroPoint;
outOfAlpha(2) = aero.envelope.alpha_rad(2)+deg2rad(0.1);
record('Angle-of-attack envelope enforced', ...
    expectError(@() evaluatePoint(outOfAlpha,aero)), ...
    'alpha outside the V0.1 attached-flow boundary is rejected');
outOfBeta = zeroPoint;
outOfBeta(3) = aero.envelope.beta_rad(2)+deg2rad(0.1);
record('Sideslip envelope enforced', ...
    expectError(@() evaluatePoint(outOfBeta,aero)), ...
    'beta outside the V0.1 attached-flow boundary is rejected');

fprintf('============================================================\n');
fprintf(' C172S NONLINEAR 6-DOF V1 - NEAR-TRIM AERO AUDIT\n');
fprintf('============================================================\n\n');
fprintf('%-39s: %s\n','Artifact',aero.artifactRevision);
fprintf('%-39s: %s\n','Longitudinal anchor',aero.required.longitudinalPlant);
fprintf('%-39s: %s\n','Lateral anchor',aero.required.lateralPlant);
fprintf('%-39s: %.0f ft / %.0f KTAS\n\n','Linearization point', ...
    source.trim.pressureAltitude_ft,source.trim.V_KTAS);
for k = 1:numel(passed)
    label = 'PASS';
    if ~passed(k), label = 'FAIL'; end
    fprintf('[%s] %-47s %s\n',label,name(k),detail(k));
end
fprintf('\nPassed audit cases                     : %d/%d\n', ...
    sum(passed),numel(passed));
fprintf('Maximum dimensional Jacobian error     : %.9g\n', ...
    relativeJacobianError);
fprintf('Longitudinal A/B maximum errors         : %.9g / %.9g\n', ...
    longAError,longBError);
fprintf('Lateral A/B maximum errors              : %.9g / %.9g\n', ...
    latAError,latBError);

results = struct('Name',cellstr(name),'Status',num2cell(passed), ...
    'Detail',cellstr(detail));
assert(all(passed),'auditC172sNearTrimAerodynamics:AuditFailed', ...
    '%d of %d near-trim aerodynamic gates failed.',sum(~passed),numel(passed));
fprintf('PASS: nonlinear force/moment Jacobians reproduce all frozen derivatives.\n');
fprintf('PASS: both frozen A/B matrix pairs are independently reconstructed.\n');
fprintf('PASS: control, stability, rate and alpha-dot signs are preserved.\n');
fprintf(['NOTE: this is a near-trim compatibility layer; coefficient scheduling, ', ...
    'propulsion and stall aerodynamics remain unimplemented.\n']);
fprintf('NOTE: this is an educational research model, not aircraft approval.\n');

function record(caseName,condition,caseDetail)
name(end+1,1) = string(caseName); %#ok<AGROW>
passed(end+1,1) = logical(condition); %#ok<AGROW>
detail(end+1,1) = string(caseDetail); %#ok<AGROW>
end
end

function output = evaluatePoint(point,model)
% point = [du alpha beta p q r alphaDot deltaE deltaA deltaR].'
V = model.trim.V_mps+point(1);
alpha = point(2);
beta = point(3);
velocityBody = V*[cos(alpha)*cos(beta);sin(beta);sin(alpha)*cos(beta)];
state = zeros(13,1);
state(1:3) = velocityBody;
state(4:6) = point(4:6);
state(9) = -model.trim.altitude_m;
state(10) = 1.0;
controls = point(8:10);
output = c172sNearTrimAerodynamics( ...
    state,controls,point(7),zeros(3,1),zeros(3,1),model);
end

function vector = outputVector(output)
vector = [output.dragIncrement_N;output.liftIncrement_N; ...
    output.sideForce_N;output.rollMoment_Nm; ...
    output.pitchMoment_Nm;output.yawMoment_Nm];
end

function expected = expectedDimensionalJacobian(longPlant,latPlant)
expected = zeros(6,10);
ld = longPlant.dimensionalDerivatives;
td = latPlant.dimensionalDerivatives;
expected(1,[1 2]) = [ld.D_u,ld.D_alpha];
expected(2,[1 2 5 7 8]) = [ ...
    ld.L_u,ld.L_alpha,ld.L_q,ld.L_alphaDot,ld.L_deltaE];
expected(5,[2 5 7 8]) = [ ...
    ld.M_alpha,ld.M_q,ld.M_alphaDot,ld.M_deltaE];
expected(3,[3 4 6 9 10]) = [ ...
    td.Y_beta,td.Y_p,td.Y_r,td.Y_deltaA,td.Y_deltaR];
expected(4,[3 4 6 9 10]) = [ ...
    td.L_beta,td.L_p,td.L_r,td.L_deltaA,td.L_deltaR];
expected(6,[3 4 6 9 10]) = [ ...
    td.N_beta,td.N_p,td.N_r,td.N_deltaA,td.N_deltaR];
end

function [A,B] = reconstructLongitudinalMatrices(J,data,model)
m = data.trim.mass_kg;
V = data.trim.V_mps;
g = data.constants.g;
Iy = data.inertia.Iy_kgm2;
D_u = J(1,1); D_alpha = J(1,2);
L_u = J(2,1); L_alpha = J(2,2); L_q = J(2,5);
L_alphaDot = J(2,7); L_deltaE = J(2,8);
M_alpha = J(5,2); M_q = J(5,5);
M_alphaDot = J(5,7); M_deltaE = J(5,8);
denominator = m*V+L_alphaDot;
aAlpha = [-L_u,-L_alpha,m*V-L_q,0]/denominator;
bAlpha = -L_deltaE/denominator;
A = zeros(4,4); B = zeros(4,1);
A(1,:) = [-D_u/m,g-D_alpha/m,0,-g];
A(2,:) = aAlpha;
A(3,:) = (M_alphaDot/Iy)*aAlpha;
A(3,2) = A(3,2)+M_alpha/Iy;
A(3,3) = A(3,3)+M_q/Iy;
A(4,3) = 1.0;
B(2) = bAlpha;
B(3) = M_deltaE/Iy+(M_alphaDot/Iy)*bAlpha;
if model.longitudinal.propulsion.dT_dV_N_per_mps ~= 0
    A(1,1) = A(1,1)+ ...
        model.longitudinal.propulsion.dT_dV_N_per_mps/m;
end
end

function [A,B] = reconstructLateralMatrices(J,data,~)
m = data.trim.mass_kg; V = data.trim.V_mps; g = data.constants.g;
inertia = [data.inertia.Ixx_kgm2,-data.inertia.Ixz_kgm2; ...
    -data.inertia.Ixz_kgm2,data.inertia.Izz_kgm2];
Y = J(3,[3 4 6 9 10]);
L = J(4,[3 4 6 9 10]);
N = J(6,[3 4 6 9 10]);
A = zeros(5,5); B = zeros(5,2);
A(1,1:3) = [Y(1),Y(2),Y(3)]/(m*V)+[0,0,-1];
A(1,4) = g/V;
B(1,:) = Y(4:5)/(m*V);
A(2:3,1:3) = inertia\[L(1:3);N(1:3)];
B(2:3,:) = inertia\[L(4:5);N(4:5)];
A(4,2) = 1.0; A(5,3) = 1.0;
end

function error = scaledMatrixError(actual,expected)
scale = max(1,abs(expected));
error = max(abs(actual(:)-expected(:))./scale(:));
end

function passed = expectError(action)
passed = false;
try
    action();
catch
    passed = true;
end
end
