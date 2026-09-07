%% C172S Lateral-Directional Plant V0.1 - open-loop validation

clc;
clear;
close all;

data = c172sLateralDirectionalData();
[sys,plant] = c172sLateralDirectionalPlant(data);

A = plant.A;
B = plant.B;

%% Structural and traceability gates

assert(isequal(size(A),[5,5]),'A must be 5-by-5.');
assert(isequal(size(B),[5,2]),'B must be 5-by-2.');
assert(all(isfinite(A(:))) && all(isfinite(B(:))), ...
    'Plant matrices must contain only finite values.');
assert(min(eig(plant.inertiaMatrix)) > 0, ...
    'The roll/yaw inertia matrix is not positive definite.');
assert(data.model.notForCertification, ...
    'The V0.1 research-status flag was unexpectedly removed.');

% Independent numeric baseline for the declared 4000-ft/110-KTAS case.
expectedA = [ ...
    -0.133360555603741, -0.001534632012328, -0.991289926416518, 0.173296387198115, 0; ...
   -19.190644699437110, -9.770892677695520,  1.995756802252702, 0,                 0; ...
     6.754771946606879, -0.300576771946759, -0.991903347424305, 0,                 0; ...
     0,                  1,                  0,                 0,                 0; ...
     0,                  0,                  1,                 0,                 0];

expectedB = [ ...
     0,                  -0.080446528702902; ...
    38.381289398874220,  -3.169690753727253; ...
     5.507737125694839,   6.827515644493413; ...
     0,                   0; ...
     0,                   0];

matrixTolerance = 5.0e-11;
maxAError = max(abs(A(:)-expectedA(:)));
maxBError = max(abs(B(:)-expectedB(:)));
assert(maxAError < matrixTolerance, ...
    'A-matrix baseline error %.6g exceeds %.6g.',maxAError,matrixTolerance);
assert(maxBError < matrixTolerance, ...
    'B-matrix baseline error %.6g exceeds %.6g.',maxBError,matrixTolerance);

if ~isempty(sys)
    assert(max(abs(sys.A(:)-A(:))) < eps, ...
        'The SS object A matrix differs from the matrix ledger.');
    assert(max(abs(sys.B(:)-B(:))) < eps, ...
        'The SS object B matrix differs from the matrix ledger.');
end

%% Sign-convention gates

% Positive sideslip must provide restoring side force/roll/yaw behavior.
betaDerivative = A*[1;0;0;0;0];
assert(betaDerivative(1) < 0, ...
    'CY_beta sign failed: positive beta must produce betaDot < 0.');
assert(betaDerivative(2) < 0, ...
    'Cl_beta sign failed: positive beta must produce pDot < 0.');
assert(betaDerivative(3) > 0, ...
    'Cn_beta sign failed: positive beta must produce rDot > 0.');

% Primary control directions under the project input convention.
assert(B(2,1) > 0, ...
    'Aileron sign failed: right-wing-down command must produce pDot > 0.');
assert(B(3,2) > 0, ...
    'Rudder sign failed: nose-right command must produce rDot > 0.');

%% Modal classification and engineering-plausibility gates

lambda = eig(A);
integratorIndex = find(abs(lambda) < 1.0e-10);
assert(numel(integratorIndex) == 1, ...
    'Expected exactly one heading integrator at the origin.');

dynamicLambda = lambda;
dynamicLambda(integratorIndex) = [];
assert(all(real(dynamicLambda) < 0), ...
    'Every non-heading lateral-directional mode must be stable.');

realModes = dynamicLambda(abs(imag(dynamicLambda)) < 1.0e-8);
complexModes = dynamicLambda(abs(imag(dynamicLambda)) >= 1.0e-8);
assert(numel(realModes) == 2 && numel(complexModes) == 2, ...
    'Expected two real modes and one complex Dutch-roll pair.');

[~,rollIndex] = min(real(realModes));
rollPole = real(realModes(rollIndex));
realModes(rollIndex) = [];
spiralPole = real(realModes(1));

dutchPole = complexModes(find(imag(complexModes) > 0,1,'first'));
dutchWn = abs(dutchPole);
dutchZeta = -real(dutchPole)/dutchWn;
dutchPeriod = 2*pi/abs(imag(dutchPole));
rollTimeConstant = -1/rollPole;
spiralTimeConstant = -1/spiralPole;

assert(rollTimeConstant > 0.05 && rollTimeConstant < 0.50, ...
    'Roll-subsidence time constant is outside the V0.1 plausibility gate.');
assert(spiralTimeConstant > 10.0, ...
    'Spiral-mode time constant is unexpectedly fast.');
assert(dutchWn > 0.5 && dutchWn < 5.0, ...
    'Dutch-roll natural frequency is outside the V0.1 plausibility gate.');
assert(dutchZeta > 0.05 && dutchZeta < 0.50, ...
    'Dutch-roll damping ratio is outside the V0.1 plausibility gate.');

%% Independent exact-ZOH pulse responses (no toolbox required)

dt = 0.01;
t = (0:dt:20).';
u = zeros(numel(t),2);
u(t < 1.0,1) = deg2rad(1.0);  % right-wing-down aileron pulse
u(t >= 5.0 & t < 6.0,2) = deg2rad(1.0); % nose-right rudder pulse

nState = size(A,1);
augmented = [A,B;zeros(2,nState+2)];
transition = expm(augmented*dt);
Ad = transition(1:nState,1:nState);
Bd = transition(1:nState,nState+1:end);

x = zeros(nState,1);
y = zeros(numel(t),nState);
for k = 1:numel(t)-1
    x = Ad*x + Bd*u(k,:).';
    y(k+1,:) = x.';
end

assert(max(y(t <= 1.0,4)) > 0, ...
    'Aileron pulse did not produce positive right-wing-down bank.');
assert(max(y(t >= 5.0 & t <= 6.0,3)) > 0, ...
    'Rudder pulse did not produce positive nose-right yaw rate.');

%% Report

fprintf('============================================================\n');
fprintf('  C172S LATERAL-DIRECTIONAL PLANT V0.1 - VALIDATION\n');
fprintf('============================================================\n\n');
fprintf('Status                              : %s\n',data.model.releaseStatus);
fprintf('Project trim                         : %.0f ft / %.1f KTAS / %.0f lb\n', ...
    data.trim.pressureAltitude_ft,data.trim.V_KTAS,data.trim.weight_lb);
fprintf('Primary published source             : PLOS ONE e0165017, Tables 1-2\n');
fprintf('Published source case                : %.0f m/s / %.0f m / %.1f kg\n', ...
    data.sourceCase.V_mps,data.sourceCase.altitude_m,data.sourceCase.mass_kg);
fprintf('Inertia mass scale                   : %.6f\n',data.inertia.massScale);
fprintf('Ixz                                  : %.6f kg*m^2 (source nominal)\n', ...
    data.inertia.Ixz_kgm2);
fprintf('Maximum A baseline error             : %.9g\n',maxAError);
fprintf('Maximum B baseline error             : %.9g\n\n',maxBError);
fprintf('Roll-subsidence pole                 : %.6f 1/s\n',rollPole);
fprintf('Roll-subsidence time constant        : %.4f s\n',rollTimeConstant);
fprintf('Spiral pole                          : %.8f 1/s\n',spiralPole);
fprintf('Spiral time constant                 : %.2f s\n',spiralTimeConstant);
fprintf('Dutch-roll poles                     : %.6f +/- %.6fi 1/s\n', ...
    real(dutchPole),abs(imag(dutchPole)));
fprintf('Dutch-roll natural frequency         : %.4f rad/s\n',dutchWn);
fprintf('Dutch-roll damping ratio             : %.4f\n',dutchZeta);
fprintf('Dutch-roll period                    : %.4f s\n\n',dutchPeriod);
fprintf('PASS: matrix construction and all sign conventions are correct.\n');
fprintf('PASS: roll, spiral and Dutch-roll modes satisfy V0.1 gates.\n');
fprintf('PASS: independent aileron and rudder pulse directions are correct.\n');
fprintf(['NOTE: this is a provisional research baseline; operating-point ', ...
    'adaptation and Ixz remain open validation items.\n']);

%% Diagnostic plots

figure('Name','C172S Lateral-Directional Plant V0.1','Color','w');
tiledlayout(2,2);

nexttile;
plot(t,rad2deg(y(:,4)),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('\phi (deg)');
title('Bank Angle');

nexttile;
plot(t,rad2deg(y(:,1)),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('\beta (deg)');
title('Sideslip');

nexttile;
plot(t,rad2deg(y(:,2)),'LineWidth',1.3);
hold on;
plot(t,rad2deg(y(:,3)),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Rate (deg/s)');
legend('p','r','Location','best');
title('Body Rates');

nexttile;
plot(real(dynamicLambda),imag(dynamicLambda),'x','MarkerSize',9,'LineWidth',1.5);
grid on;
xlabel('Real (1/s)');
ylabel('Imaginary (rad/s)');
title('Dynamic Lateral Poles');

sgtitle('C172S Lateral-Directional Plant V0.1');
