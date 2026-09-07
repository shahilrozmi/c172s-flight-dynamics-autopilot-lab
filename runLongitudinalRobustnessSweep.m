%% C172S Longitudinal Plant V1 - robustness sweep
%
% Sweeps the source-supported/engineering uncertainty set:
%   CG       = forward, nominal, aft MTOW limits/case
%   Iy       = lower bound, NASA-scaled nominal, intermediate, upper bound
%   CL_de    = provisional lower, nominal, upper values
%
% CG and Iy affect the open-loop A matrix and its modes. CL_de affects the
% B matrix and elevator authority but not the uncontrolled eigenvalues.

clc;
clear;
close all;

baselineData = c172sLongitudinalData();

cgValues_in = [ ...
    baselineData.cg.forwardLimit_in, ...
    baselineData.cg.nominal_in, ...
    baselineData.cg.aftLimit_in];

iyValues_kgm2 = unique([ ...
    baselineData.uncertainty.Iy_range_kgm2(1), ...
    baselineData.inertia.Iy_kgm2, ...
    2300.0, ...
    baselineData.uncertainty.Iy_range_kgm2(2)], 'stable');

clDeltaEValues = [ ...
    baselineData.uncertainty.CL_deltaE_range(1), ...
    baselineData.aero.CL_deltaE, ...
    baselineData.uncertainty.CL_deltaE_range(2)];

nCase = numel(cgValues_in)*numel(iyValues_kgm2)*numel(clDeltaEValues);

caseID             = (1:nCase).';
cgIn               = zeros(nCase,1);
cgPercentMAC       = zeros(nCase,1);
staticMarginPct    = zeros(nCase,1);
iyKgm2             = zeros(nCase,1);
clDeltaE           = zeros(nCase,1);
cmAlpha            = zeros(nCase,1);
isStable           = false(nCase,1);

spReal             = zeros(nCase,1);
spImag             = zeros(nCase,1);
spWn               = zeros(nCase,1);
spZeta             = zeros(nCase,1);
spPeriod_s         = zeros(nCase,1);

phReal             = zeros(nCase,1);
phImag             = zeros(nCase,1);
phWn               = zeros(nCase,1);
phZeta             = zeros(nCase,1);
phPeriod_s         = zeros(nCase,1);

alphaDot0_degps    = zeros(nCase,1);
qDot0_degps2       = zeros(nCase,1);

row = 0;
for iCG = 1:numel(cgValues_in)
    for iIy = 1:numel(iyValues_kgm2)
        for iCLde = 1:numel(clDeltaEValues)
            row = row + 1;

            data = c172sLongitudinalData(cgValues_in(iCG));
            data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
            data.aero.CL_deltaE = clDeltaEValues(iCLde);

            [~, plant] = c172sLongitudinalPlant(data);
            poles = eig(plant.A);

            positiveImaginaryPoles = poles(imag(poles) > 1.0e-8);
            [~, modeOrder] = sort(abs(imag(positiveImaginaryPoles)), ...
                'descend');
            positiveImaginaryPoles = positiveImaginaryPoles(modeOrder);

            if numel(positiveImaginaryPoles) ~= 2
                error('Case %d did not contain two oscillatory mode pairs.',row);
            end

            shortPeriodPole = positiveImaginaryPoles(1);
            phugoidPole = positiveImaginaryPoles(2);

            cgIn(row)            = data.cg.selected_in;
            cgPercentMAC(row)    = 100*data.cg.h;
            staticMarginPct(row) = 100*data.cg.staticMargin;
            iyKgm2(row)          = data.inertia.Iy_kgm2;
            clDeltaE(row)        = data.aero.CL_deltaE;
            cmAlpha(row)         = data.aero.Cm_alpha;
            isStable(row)        = all(real(poles) < 0);

            spReal(row)     = real(shortPeriodPole);
            spImag(row)     = imag(shortPeriodPole);
            spWn(row)       = abs(shortPeriodPole);
            spZeta(row)     = -real(shortPeriodPole)/abs(shortPeriodPole);
            spPeriod_s(row) = 2*pi/abs(imag(shortPeriodPole));

            phReal(row)     = real(phugoidPole);
            phImag(row)     = imag(phugoidPole);
            phWn(row)       = abs(phugoidPole);
            phZeta(row)     = -real(phugoidPole)/abs(phugoidPole);
            phPeriod_s(row) = 2*pi/abs(imag(phugoidPole));

            % Initial response to one degree trailing-edge up. At x=0,
            % xDot(0+) = B*deltaE. Positive qDot is pitch-up.
            deltaECheck_rad = -deg2rad(1.0);
            alphaDot0_degps(row) = rad2deg(plant.B(2)*deltaECheck_rad);
            qDot0_degps2(row) = rad2deg(plant.B(3)*deltaECheck_rad);
        end
    end
end

results = table( ...
    caseID,cgIn,cgPercentMAC,staticMarginPct,iyKgm2,clDeltaE,cmAlpha, ...
    isStable,spReal,spImag,spWn,spZeta,spPeriod_s, ...
    phReal,phImag,phWn,phZeta,phPeriod_s, ...
    alphaDot0_degps,qDot0_degps2);

%% Validation gates

assert(all(results.isStable), ...
    'At least one uncertainty case produced an unstable open-loop plant.');
assert(all(results.spWn > 2.0 & results.spWn < 10.0), ...
    'At least one short-period frequency failed the broad sanity range.');
assert(all(results.spZeta > 0.2 & results.spZeta < 1.0), ...
    'At least one short-period damping ratio failed the broad sanity range.');
assert(all(results.phZeta > 0), ...
    'At least one phugoid mode was not positively damped.');
assert(all(results.phPeriod_s > 10.0 & results.phPeriod_s < 100.0), ...
    'At least one phugoid period failed the broad sanity range.');
assert(all(results.qDot0_degps2 > 0), ...
    'At least one case failed the elevator pitch-sign check.');

% Confirm analytically expected structure: changing CL_de alone must not
% change A or the open-loop poles for fixed CG and Iy.
poleInvarianceTolerance = 1.0e-12;
for iCG = 1:numel(cgValues_in)
    for iIy = 1:numel(iyValues_kgm2)
        selection = results.cgIn == cgValues_in(iCG) ...
            & results.iyKgm2 == iyValues_kgm2(iIy);
        spValues = results.spWn(selection);
        phValues = results.phWn(selection);
        assert(max(spValues) - min(spValues) < poleInvarianceTolerance);
        assert(max(phValues) - min(phValues) < poleInvarianceTolerance);
    end
end

%% Console report and machine-readable ledger

fprintf('============================================================\n');
fprintf('     C172S LONGITUDINAL PLANT V1 - ROBUSTNESS SWEEP\n');
fprintf('============================================================\n\n');
fprintf('Total cases                       : %d\n',height(results));
fprintf('Stable cases                      : %d/%d\n', ...
    nnz(results.isStable),height(results));
fprintf('CG range                          : %.2f to %.2f in\n', ...
    min(results.cgIn),max(results.cgIn));
fprintf('Iy range                          : %.3f to %.3f kg*m^2\n', ...
    min(results.iyKgm2),max(results.iyKgm2));
fprintf('CL_deltaE range                   : %.3f to %.3f rad^-1\n\n', ...
    min(results.clDeltaE),max(results.clDeltaE));

fprintf('Short-period wn range             : %.4f to %.4f rad/s\n', ...
    min(results.spWn),max(results.spWn));
fprintf('Short-period damping range        : %.4f to %.4f\n', ...
    min(results.spZeta),max(results.spZeta));
fprintf('Phugoid period range              : %.3f to %.3f s\n', ...
    min(results.phPeriod_s),max(results.phPeriod_s));
fprintf('Phugoid damping range             : %.4f to %.4f\n', ...
    min(results.phZeta),max(results.phZeta));
fprintf('Initial pitch acceleration range  : %.3f to %.3f deg/s^2\n', ...
    min(results.qDot0_degps2),max(results.qDot0_degps2));
fprintf('Initial alpha-rate range          : %.3f to %.3f deg/s\n\n', ...
    min(results.alphaDot0_degps),max(results.alphaDot0_degps));

fprintf('PASS: all uncertainty cases remain open-loop stable.\n');
fprintf('PASS: all modal sanity and elevator-authority checks passed.\n');
fprintf(['PASS: CL_deltaE changes B/control authority without changing ', ...
    'the open-loop poles.\n\n']);

outputFile = fullfile(pwd,'longitudinal_robustness_results.csv');
writetable(results,outputFile);
fprintf('Saved case ledger: %s\n',outputFile);

%% Figure 1: robust pole map

figure('Name','C172S Robust Longitudinal Pole Map','Color','w');
hold on;
grid on;

cgColors = lines(numel(cgValues_in));
iyMarkers = {'o','s','d','^'};

for iCG = 1:numel(cgValues_in)
    for iIy = 1:numel(iyValues_kgm2)
        data = c172sLongitudinalData(cgValues_in(iCG));
        data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
        [~, plant] = c172sLongitudinalPlant(data);
        poles = eig(plant.A);

        plot(real(poles),imag(poles),iyMarkers{iIy}, ...
            'Color',cgColors(iCG,:), ...
            'MarkerSize',7, ...
            'LineWidth',1.3, ...
            'DisplayName',sprintf('CG %.2f in, Iy %.1f', ...
                cgValues_in(iCG),iyValues_kgm2(iIy)));
    end
end

xline(0,'k--','HandleVisibility','off');
xlabel('Real axis (1/s)');
ylabel('Imaginary axis (rad/s)');
title('C172S Longitudinal Pole Map Across CG and I_y');
legend('Location','eastoutside');

%% Figure 2: modal and control-authority trends

figure('Name','C172S Longitudinal Robustness Trends','Color','w');
tiledlayout(2,2);

nominalCLde = baselineData.aero.CL_deltaE;

nexttile;
hold on;
for iCG = 1:numel(cgValues_in)
    selection = results.cgIn == cgValues_in(iCG) ...
        & results.clDeltaE == nominalCLde;
    subset = sortrows(results(selection,:), 'iyKgm2');
    plot(subset.iyKgm2,subset.spWn,'-o','LineWidth',1.2, ...
        'DisplayName',sprintf('CG %.2f in',cgValues_in(iCG)));
end
grid on;
xlabel('I_y (kg m^2)');
ylabel('\omega_{n,SP} (rad/s)');
title('Short-Period Natural Frequency');
legend('Location','best');

nexttile;
hold on;
for iCG = 1:numel(cgValues_in)
    selection = results.cgIn == cgValues_in(iCG) ...
        & results.clDeltaE == nominalCLde;
    subset = sortrows(results(selection,:), 'iyKgm2');
    plot(subset.iyKgm2,subset.spZeta,'-o','LineWidth',1.2, ...
        'DisplayName',sprintf('CG %.2f in',cgValues_in(iCG)));
end
grid on;
xlabel('I_y (kg m^2)');
ylabel('\zeta_{SP}');
title('Short-Period Damping Ratio');

nexttile;
hold on;
for iCG = 1:numel(cgValues_in)
    selection = results.cgIn == cgValues_in(iCG) ...
        & results.clDeltaE == nominalCLde;
    subset = sortrows(results(selection,:), 'iyKgm2');
    plot(subset.iyKgm2,subset.phPeriod_s,'-o','LineWidth',1.2, ...
        'DisplayName',sprintf('CG %.2f in',cgValues_in(iCG)));
end
grid on;
xlabel('I_y (kg m^2)');
ylabel('Phugoid period (s)');
title('Phugoid Period');

nexttile;
hold on;
for iCLde = 1:numel(clDeltaEValues)
    selection = results.cgIn == baselineData.cg.nominal_in ...
        & results.clDeltaE == clDeltaEValues(iCLde);
    subset = sortrows(results(selection,:), 'iyKgm2');
    plot(subset.iyKgm2,subset.qDot0_degps2,'-o','LineWidth',1.2, ...
        'DisplayName',sprintf('CL_{\\deltae} = %.3f', ...
            clDeltaEValues(iCLde)));
end
grid on;
xlabel('I_y (kg m^2)');
ylabel('Initial q-dot (deg/s^2)');
title('Pitch Authority for 1 deg Elevator Up');
legend('Location','best');

sgtitle('C172S Longitudinal Plant V1 - Robustness Summary');