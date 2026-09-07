%% C172S Longitudinal Plant V1 - validation and sign check

clc;
clear;
close all;

data = c172sLongitudinalData();
[sys, model] = c172sLongitudinalPlant(data);

fprintf('============================================================\n');
fprintf('       C172S LONGITUDINAL PLANT V1 - BASELINE CHECK\n');
fprintf('============================================================\n\n');

fprintf('Trim condition\n');
fprintf('  Pressure altitude : %.0f ft ISA\n', data.trim.pressureAltitude_ft);
fprintf('  True airspeed     : %.1f KTAS (%.4f m/s)\n', ...
    data.trim.V_KTAS, data.trim.V_mps);
fprintf('  Weight            : %.0f lb\n', data.trim.weight_lb);
fprintf('  Density           : %.6f kg/m^3\n', data.trim.rho_kgpm3);
fprintf('  CL trim           : %.6f\n', data.trim.CL);
fprintf('  CD trim           : %.6f\n\n', data.trim.CD);

fprintf('Mass-properties and stability case\n');
fprintf('  CG                 : %.2f in aft of datum (%.3f %% MAC)\n', ...
    data.cg.selected_in, 100*data.cg.h);
fprintf('  Neutral point      : %.3f %% MAC\n', 100*data.cg.hNeutral);
fprintf('  Static margin      : %.3f %% MAC\n', 100*data.cg.staticMargin);
fprintf('  Cm_alpha           : %.6f rad^-1\n', data.aero.Cm_alpha);
fprintf('  Iy nominal         : %.3f kg*m^2\n\n', data.inertia.Iy_kgm2);

fprintf('A matrix, x = [u alpha q theta]^T\n');
disp(model.A);
fprintf('B matrix, deltaE positive trailing-edge down\n');
disp(model.B);

lambda = eig(model.A);
if any(real(lambda) >= 0)
    error('Plant check failed: at least one open-loop pole is unstable.');
end

% One pole from each conjugate pair is sufficient for mode reporting.
oscillatoryPoles = lambda(imag(lambda) > 1e-8);
[~, order] = sort(abs(imag(oscillatoryPoles)), 'descend');
oscillatoryPoles = oscillatoryPoles(order);

if numel(oscillatoryPoles) ~= 2
    error('Expected two oscillatory longitudinal modes; found %d.', ...
        numel(oscillatoryPoles));
end

modeNames = {'Short-period','Phugoid'};
fprintf('Open-loop modes\n');
for k = 1:2
    pole = oscillatoryPoles(k);
    wn = abs(pole);
    zeta = -real(pole)/wn;
    period = 2*pi/abs(imag(pole));
    fprintf(['  %-12s : %+.6f %+.6fi  wn = %.4f rad/s, ', ...
             'zeta = %.4f, period = %.3f s\n'], ...
        modeNames{k}, real(pole), imag(pole), wn, zeta, period);
end

shortPole = oscillatoryPoles(1);
phugoidPole = oscillatoryPoles(2);
shortWn = abs(shortPole);
shortZeta = -real(shortPole)/shortWn;
phugoidPeriod = 2*pi/abs(imag(phugoidPole));

assert(shortWn > 2.0 && shortWn < 10.0, ...
    'Short-period natural frequency failed its broad sanity range.');
assert(shortZeta > 0.2 && shortZeta < 1.0, ...
    'Short-period damping ratio failed its broad sanity range.');
assert(phugoidPeriod > 10.0 && phugoidPeriod < 100.0, ...
    'Phugoid period failed its broad sanity range.');

fprintf('\nPASS: both longitudinal mode pairs are stable and physically plausible.\n');

%% Pole map

figure('Name','C172S Longitudinal Open-Loop Poles','Color','w');
plot(real(lambda), imag(lambda), 'x', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Real axis (1/s)');
ylabel('Imaginary axis (rad/s)');
title('C172S Longitudinal Plant V1 - Open-Loop Poles');

%% Elevator sign check: one-degree trailing-edge-up command

if ~isempty(sys)
    t = (0:0.02:40).';
    deltaE = -deg2rad(1.0)*ones(size(t));
    y = lsim(sys, deltaE, t);

    figure('Name','C172S Elevator Sign Check','Color','w');
    tiledlayout(2,1);

    nexttile;
    plot(t, rad2deg(y(:,3)), 'LineWidth', 1.2);
    grid on;
    ylabel('q (deg/s)');
    title('Response to 1 deg Elevator Trailing-Edge Up');

    nexttile;
    plot(t, rad2deg(y(:,4)), 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('\theta (deg)');

    assert(y(2,3) > 0, ...
        'Elevator sign check failed: trailing-edge up should initially pitch up.');
    fprintf('PASS: elevator sign convention gives the expected initial pitch-up.\n');
end
