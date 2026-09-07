function figureHandle = plotC172sFlightControlBackendComparison( ...
    matlabResult,simulinkResult,scenario,comparison)
%PLOTC172SFLIGHTCONTROLBACKENDCOMPARISON Show public backend equivalence.

t = matlabResult.time_s(:);
reference = c172sNonlinearAutopilotResultMatrix(matlabResult);
executable = c172sNonlinearAutopilotResultMatrix(simulinkResult);
errorBySample = max(abs(reference-executable),[],2);

figureHandle = figure('Name', ...
    ['C172S Flight-Control Lab - MATLAB versus Simulink - ' scenario.name], ...
    'Color','w','NumberTitle','off');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t,mod(rad2deg(matlabResult.euler_rad(:,3)),360), ...
    'LineWidth',1.4);
hold on;
plot(t,mod(rad2deg(simulinkResult.euler_rad(:,3)),360),'--', ...
    'LineWidth',1.1);
grid on; xlabel('Time (s)'); ylabel('Heading (deg)');
title('Heading equivalence'); legend('MATLAB','Simulink','Location','best');

nexttile;
plot(t,matlabResult.altitude_ft,'LineWidth',1.4);
hold on;
plot(t,simulinkResult.altitude_ft,'--','LineWidth',1.1);
grid on; xlabel('Time (s)'); ylabel('Altitude (ft)');
title('Altitude equivalence'); legend('MATLAB','Simulink','Location','best');

nexttile;
plot(t,rad2deg(matlabResult.control(:,1:3)),'LineWidth',1.2);
hold on;
set(gca,'ColorOrderIndex',1);
plot(t,rad2deg(simulinkResult.control(:,1:3)),'--','LineWidth',0.9);
grid on; xlabel('Time (s)'); ylabel('Surface (deg)');
title('Automatic-surface equivalence');
legend('Elevator MATLAB','Aileron MATLAB','Rudder MATLAB', ...
    'Elevator Simulink','Aileron Simulink','Rudder Simulink', ...
    'Location','best');

nexttile;
semilogy(t,max(errorBySample,eps),'LineWidth',1.25);
hold on;
yline(comparison.tolerance,'--','Tolerance','LineWidth',1.0);
grid on; xlabel('Time (s)'); ylabel('Maximum absolute difference');
title('Maximum of 58 signals at each sample');

status = 'PASS'; if ~comparison.pass, status = 'REVIEW'; end
sgtitle(sprintf('%s | MATLAB versus Simulink | max %.3g | %s', ...
    scenario.name,comparison.maximumAbsoluteError,status));
end
