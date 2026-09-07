%% C172S Pitch-Rate Damper V1 - robust gain design
%
% Feedback law:
%     deltaE = deltaECommand + Kq*q
%
% Positive q commands positive (trailing-edge-down) elevator, producing a
% negative pitching moment and opposing the nose-up rate.

clc;
clear;
close all;

baselineData = c172sLongitudinalData();
damper = c172sPitchRateDamperData();

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

%% Search for the smallest robust qualifying pitch-rate gain

gainGrid_s = ( ...
    damper.design.gainSearchRange_s(1): ...
    damper.design.gainSearchIncrement_s: ...
    damper.design.gainSearchRange_s(2)).';

minimumSPZeta = nan(size(gainGrid_s));
maximumSPZeta = nan(size(gainGrid_s));
minimumSPWn = nan(size(gainGrid_s));
maximumSPWn = nan(size(gainGrid_s));
allCasesStable = false(size(gainGrid_s));
allCasesOscillatory = false(size(gainGrid_s));

for iGain = 1:numel(gainGrid_s)
    gain = gainGrid_s(iGain);
    caseSPZeta = [];
    caseSPWn = [];
    stableAtGain = true;
    oscillatoryAtGain = true;

    for iCG = 1:numel(cgValues_in)
        for iIy = 1:numel(iyValues_kgm2)
            for iCLde = 1:numel(clDeltaEValues)
                data = c172sLongitudinalData(cgValues_in(iCG));
                data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
                data.aero.CL_deltaE = clDeltaEValues(iCLde);
                [~, plant] = c172sLongitudinalPlant(data);

                Aclosed = plant.A + plant.B*[0,0,gain,0];
                poles = eig(Aclosed);

                if any(real(poles) >= 0)
                    stableAtGain = false;
                end

                positiveImaginaryPoles = poles(imag(poles) > 1.0e-8);
                if numel(positiveImaginaryPoles) ~= 2
                    oscillatoryAtGain = false;
                    continue;
                end

                [~, order] = sort(abs(imag(positiveImaginaryPoles)), ...
                    'descend');
                shortPeriodPole = positiveImaginaryPoles(order(1));

                caseSPWn(end+1,1) = abs(shortPeriodPole); %#ok<SAGROW>
                caseSPZeta(end+1,1) = ...
                    -real(shortPeriodPole)/abs(shortPeriodPole); %#ok<SAGROW>
            end
        end
    end

    allCasesStable(iGain) = stableAtGain;
    allCasesOscillatory(iGain) = oscillatoryAtGain;

    if stableAtGain && oscillatoryAtGain
        minimumSPZeta(iGain) = min(caseSPZeta);
        maximumSPZeta(iGain) = max(caseSPZeta);
        minimumSPWn(iGain) = min(caseSPWn);
        maximumSPWn(iGain) = max(caseSPWn);
    end
end

qualifying = allCasesStable & allCasesOscillatory ...
    & minimumSPZeta >= damper.requirements.minimumShortPeriodZeta ...
    & maximumSPZeta < damper.requirements.maximumShortPeriodZeta;

firstQualifyingIndex = find(qualifying,1,'first');
if isempty(firstQualifyingIndex)
    error('No gain in the search interval satisfies the damper requirements.');
end

minimumQualifyingGain_s = gainGrid_s(firstQualifyingIndex);
Kq_s = damper.Kq_s;

assert(Kq_s >= minimumQualifyingGain_s, ...
    'The selected Kq is below the minimum qualifying robust gain.');

%% Evaluate the selected gain across the full uncertainty grid

nCase = numel(cgValues_in)*numel(iyValues_kgm2)*numel(clDeltaEValues);

caseID = (1:nCase).';
cgIn = zeros(nCase,1);
iyKgm2 = zeros(nCase,1);
clDeltaE = zeros(nCase,1);
openSPWn = zeros(nCase,1);
openSPZeta = zeros(nCase,1);
closedSPReal = zeros(nCase,1);
closedSPImag = zeros(nCase,1);
closedSPWn = zeros(nCase,1);
closedSPZeta = zeros(nCase,1);
closedPHPeriod_s = zeros(nCase,1);
closedPHZeta = zeros(nCase,1);
isStable = false(nCase,1);
isOscillatory = false(nCase,1);

row = 0;
for iCG = 1:numel(cgValues_in)
    for iIy = 1:numel(iyValues_kgm2)
        for iCLde = 1:numel(clDeltaEValues)
            row = row + 1;

            data = c172sLongitudinalData(cgValues_in(iCG));
            data.inertia.Iy_kgm2 = iyValues_kgm2(iIy);
            data.aero.CL_deltaE = clDeltaEValues(iCLde);
            [~, plant] = c172sLongitudinalPlant(data);

            openPoles = eig(plant.A);
            Aclosed = plant.A + plant.B*damper.qFeedbackRow;
            closedPoles = eig(Aclosed);

            openPositive = openPoles(imag(openPoles) > 1.0e-8);
            [~, openOrder] = sort(abs(imag(openPositive)),'descend');
            openShortPeriodPole = openPositive(openOrder(1));

            closedPositive = closedPoles(imag(closedPoles) > 1.0e-8);
            [~, closedOrder] = sort(abs(imag(closedPositive)),'descend');

            cgIn(row) = data.cg.selected_in;
            iyKgm2(row) = data.inertia.Iy_kgm2;
            clDeltaE(row) = data.aero.CL_deltaE;
            openSPWn(row) = abs(openShortPeriodPole);
            openSPZeta(row) = ...
                -real(openShortPeriodPole)/abs(openShortPeriodPole);
            isStable(row) = all(real(closedPoles) < 0);
            isOscillatory(row) = numel(closedPositive) == 2;

            if ~isOscillatory(row)
                error('Selected gain produced a non-oscillatory case %d.',row);
            end

            closedShortPeriodPole = closedPositive(closedOrder(1));
            closedPhugoidPole = closedPositive(closedOrder(2));

            closedSPReal(row) = real(closedShortPeriodPole);
            closedSPImag(row) = imag(closedShortPeriodPole);
            closedSPWn(row) = abs(closedShortPeriodPole);
            closedSPZeta(row) = ...
                -real(closedShortPeriodPole)/abs(closedShortPeriodPole);
            closedPHPeriod_s(row) = ...
                2*pi/abs(imag(closedPhugoidPole));
            closedPHZeta(row) = ...
                -real(closedPhugoidPole)/abs(closedPhugoidPole);
        end
    end
end

results = table( ...
    caseID,cgIn,iyKgm2,clDeltaE,openSPWn,openSPZeta, ...
    closedSPReal,closedSPImag,closedSPWn,closedSPZeta, ...
    closedPHPeriod_s,closedPHZeta,isStable,isOscillatory);

%% Requirements and regression gates

assert(all(results.isStable), ...
    'At least one q-damper uncertainty case is unstable.');
assert(all(results.isOscillatory), ...
    'At least one q-damper case lost its oscillatory short-period pair.');
assert(all(results.closedSPZeta >= ...
    damper.requirements.minimumShortPeriodZeta), ...
    'At least one q-damper case failed the minimum damping requirement.');
assert(all(results.closedSPZeta < ...
    damper.requirements.maximumShortPeriodZeta), ...
    'At least one q-damper case became critically/over-damped.');
assert(all(results.closedPHZeta > 0), ...
    'At least one q-damper case destabilized the phugoid.');

%% Console report and result ledger

fprintf('============================================================\n');
fprintf('          C172S PITCH-RATE DAMPER V1 - DESIGN\n');
fprintf('============================================================\n\n');
fprintf('Feedback law                       : deltaE = deltaECmd + Kq*q\n');
fprintf('Minimum robust qualifying Kq       : %.4f s\n', ...
    minimumQualifyingGain_s);
fprintf('Selected Kq                        : %.4f s\n',Kq_s);
fprintf('Gain margin above minimum          : %.4f s\n\n', ...
    Kq_s - minimumQualifyingGain_s);

fprintf('Stable selected-gain cases         : %d/%d\n', ...
    nnz(results.isStable),height(results));
fprintf('Open-loop short-period zeta        : %.4f to %.4f\n', ...
    min(results.openSPZeta),max(results.openSPZeta));
fprintf('Damped short-period zeta           : %.4f to %.4f\n', ...
    min(results.closedSPZeta),max(results.closedSPZeta));
fprintf('Damped short-period wn             : %.4f to %.4f rad/s\n', ...
    min(results.closedSPWn),max(results.closedSPWn));
fprintf('Damped phugoid period              : %.3f to %.3f s\n', ...
    min(results.closedPHPeriod_s),max(results.closedPHPeriod_s));
fprintf('Damped phugoid zeta                : %.4f to %.4f\n\n', ...
    min(results.closedPHZeta),max(results.closedPHZeta));

fprintf('PASS: selected Kq satisfies all 36 robust damping cases.\n');
fprintf('PASS: the phugoid remains stable in all cases.\n');
fprintf('PASS: every short-period pair remains oscillatory.\n\n');

outputFile = fullfile(pwd,'pitch_rate_damper_robustness_results.csv');
writetable(results,outputFile);
fprintf('Saved gain ledger: %s\n',outputFile);

%% Figure 1: robust gain-selection envelope

figure('Name','C172S Pitch-Rate Damper Gain Selection','Color','w');
plot(gainGrid_s,minimumSPZeta,'LineWidth',1.4, ...
    'DisplayName','Worst-case \zeta_{SP}');
hold on;
plot(gainGrid_s,maximumSPZeta,'LineWidth',1.2, ...
    'DisplayName','Best-case \zeta_{SP}');
yline(damper.requirements.minimumShortPeriodZeta,'k--', ...
    'Minimum requirement','HandleVisibility','off');
xline(minimumQualifyingGain_s,':', ...
    'Minimum robust gain','HandleVisibility','off');
xline(Kq_s,'--','Selected K_q','HandleVisibility','off');
grid on;
xlabel('K_q (s)');
ylabel('Short-period damping ratio');
title('Robust Pitch-Rate Damper Gain Selection');
legend('Location','best');

%% Figure 2: open-loop and q-damped pole locations

figure('Name','C172S Pitch-Rate Damper Pole Comparison','Color','w');
hold on;
grid on;

plotOpenLegend = true;
plotClosedLegend = true;
for iCase = 1:height(results)
    data = c172sLongitudinalData(results.cgIn(iCase));
    data.inertia.Iy_kgm2 = results.iyKgm2(iCase);
    data.aero.CL_deltaE = results.clDeltaE(iCase);
    [~, plant] = c172sLongitudinalPlant(data);

    openPoles = eig(plant.A);
    closedPoles = eig(plant.A + plant.B*damper.qFeedbackRow);

    if plotOpenLegend
        openName = 'Open loop';
        plotOpenLegend = false;
    else
        openName = '';
    end

    if plotClosedLegend
        closedName = 'With q damper';
        plotClosedLegend = false;
    else
        closedName = '';
    end

    plot(real(openPoles),imag(openPoles),'x','Color',[0.55 0.55 0.55], ...
        'LineWidth',1.0,'HandleVisibility',onOff(openName), ...
        'DisplayName',openName);
    plot(real(closedPoles),imag(closedPoles),'o','Color',[0 0.447 0.741], ...
        'MarkerSize',4,'HandleVisibility',onOff(closedName), ...
        'DisplayName',closedName);
end

xline(0,'k--','HandleVisibility','off');
xlabel('Real axis (1/s)');
ylabel('Imaginary axis (rad/s)');
title(sprintf('Robust Pole Comparison, K_q = %.3f s',Kq_s));
legend('Location','best');

%% Figure 3: nominal response to an initial 5 deg/s pitch-rate disturbance

nominalData = c172sLongitudinalData();
[~, nominalPlant] = c172sLongitudinalPlant(nominalData);
nominalAClosed = nominalPlant.A + nominalPlant.B*damper.qFeedbackRow;

t = (0:0.01:12).';
x0 = [0;0;deg2rad(5);0];

openSystem = ss(nominalPlant.A,zeros(4,1),eye(4),zeros(4,1));
closedSystem = ss(nominalAClosed,zeros(4,1),eye(4),zeros(4,1));

yOpen = initial(openSystem,x0,t);
yClosed = initial(closedSystem,x0,t);
deltaEFeedback_deg = rad2deg(Kq_s*yClosed(:,3));

figure('Name','C172S Pitch-Rate Damper Disturbance Response','Color','w');
tiledlayout(2,1);

nexttile;
plot(t,rad2deg(yOpen(:,3)),'LineWidth',1.2,'DisplayName','Open loop');
hold on;
plot(t,rad2deg(yClosed(:,3)),'LineWidth',1.3, ...
    'DisplayName','With q damper');
grid on;
ylabel('q (deg/s)');
title('Response to Initial q = 5 deg/s');
legend('Location','best');

nexttile;
plot(t,deltaEFeedback_deg,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Damper elevator (deg)');
title('Elevator Feedback Command');

%% Local compatibility helper

function value = onOff(name)
if isempty(name)
    value = 'off';
else
    value = 'on';
end
end
