%% C172S Altitude Hold V1 - Simulink integration check

clc;
clear;
close all;

fprintf('\n*** RUNNING ALTITUDE HOLD V1 CHECKER REVISION R2 ***\n\n');

try
    load_system('simulink');
catch ME
    error('runAltitudeHoldSimulinkCheck:SimulinkUnavailable', ...
        'Unable to load Simulink. MATLAB reported:\n%s',ME.message);
end

data = c172sLongitudinalData();
[~,plant] = c172sLongitudinalPlant(data);
pitchController = c172sPitchAttitudeControllerData();
actuator = c172sElevatorActuatorData();
altitude = c172sAltitudeHoldControllerData();

Kh = altitude.altitudeGain_per_s;
Kv = altitude.verticalSpeedGain_radPerMps;
Vtrim = data.trim.V_mps;
tolerance = 1.0e-5;

% Rebuild from a clean diagram so a prior interrupted builder run cannot
% leave a partial or unsaved model in memory.
modelName = buildC172sAltitudeHoldSimulink(true);

%% Ten-foot small-signal command

smallCommand_m = altitude.validation.smallSignalCommand_m;
smallInput = Simulink.SimulationInput(modelName);
smallInput = smallInput.setVariable('hCommand_ah1_m',smallCommand_m);
smallInput = smallInput.setVariable('antiWindupEnable_ah1',1.0);
smallInput = smallInput.setVariable('x0_ah1',zeros(4,1));
smallInput = smallInput.setVariable('xi0_ah1',0.0);
smallInput = smallInput.setVariable('deltaE0_ah1',0.0);
smallInput = smallInput.setVariable('h0_ah1_m',0.0);
smallInput = smallInput.setModelParameter( ...
    'StopTime',num2str(altitude.validation.stopTime_s));
smallOut = sim(smallInput);
[tSmall,ySmall] = extractArrayOutput(smallOut,18);

z0 = zeros(9,1);
ySmallReference = rk4Reference(tSmall,z0,smallCommand_m,true, ...
    plant,pitchController,actuator,altitude,Vtrim);
smallDifference = ySmall - ySmallReference;
smallMaxError = max(abs(smallDifference),[],'all');
smallRmsError = sqrt(mean(smallDifference(:).^2));
assert(smallMaxError < tolerance, ...
    'Small-signal Simulink/reference mismatch %.6g exceeds %.6g.', ...
    smallMaxError,tolerance);

%% Protected 100-foot operational command

operationalCommand_m = altitude.validation.operationalCommand_m;
operationalInput = Simulink.SimulationInput(modelName);
operationalInput = operationalInput.setVariable( ...
    'hCommand_ah1_m',operationalCommand_m);
operationalInput = operationalInput.setVariable('antiWindupEnable_ah1',1.0);
operationalInput = operationalInput.setVariable('x0_ah1',zeros(4,1));
operationalInput = operationalInput.setVariable('xi0_ah1',0.0);
operationalInput = operationalInput.setVariable('deltaE0_ah1',0.0);
operationalInput = operationalInput.setVariable('h0_ah1_m',0.0);
operationalInput = operationalInput.setModelParameter( ...
    'StopTime',num2str(altitude.validation.stopTime_s));
operationalOut = sim(operationalInput);
[tOperational,yOperational] = extractArrayOutput(operationalOut,18);

yOperationalReference = rk4Reference(tOperational,z0, ...
    operationalCommand_m,true,plant,pitchController,actuator,altitude,Vtrim);
operationalDifference = yOperational - yOperationalReference;
operationalMaxError = max(abs(operationalDifference),[],'all');
operationalRmsError = sqrt(mean(operationalDifference(:).^2));
assert(operationalMaxError < tolerance, ...
    'Operational Simulink/reference mismatch %.6g exceeds %.6g.', ...
    operationalMaxError,tolerance);

fixedStep_s = median(diff(tOperational));
assert(abs(fixedStep_s-altitude.validation.timeStep_s) < 1.0e-12, ...
    'The Simulink fixed step does not match the validated altitude design.');

%% Signal-law reconstruction and feedback directions

% Columns:
% [u alpha q theta xi thetaFiltered deltaE h hFiltered hDot ...
%  hDotRaw hDotCommand thetaRaw thetaCommand deltaEUnsat ...
%  deltaEPosition deltaERate antiWindupCorrection]
law = reconstructSignals(yOperational,Kh,Kv,pitchController, ...
    actuator,altitude);

kinematicError = max(abs(yOperational(:,10) - ...
    Vtrim*(yOperational(:,4)-yOperational(:,2))));
verticalSpeedRawError = max(abs(yOperational(:,11) - ...
    Kh*(yOperational(:,9)-yOperational(:,8))));
verticalSpeedCommandError = max(abs(yOperational(:,12)-law.hDotCommand));
pitchRawError = max(abs(yOperational(:,13) - ...
    Kv*(yOperational(:,12)-yOperational(:,10))));
pitchCommandError = max(abs(yOperational(:,14)-law.thetaCommand));
controllerError = max(abs(yOperational(:,15)-law.deltaEUnsat));
positionCommandError = max(abs(yOperational(:,16)-law.deltaEPosition));
actuatorRateError = max(abs(yOperational(:,17)-law.limitedRate));
antiWindupError = max(abs(yOperational(:,18)-law.antiWindupCorrection));

reconstructionTolerance = 1.0e-11;
assert(max([kinematicError,verticalSpeedRawError, ...
    verticalSpeedCommandError,pitchRawError,pitchCommandError, ...
    controllerError,positionCommandError,actuatorRateError, ...
    antiWindupError]) < reconstructionTolerance, ...
    'One or more Simulink signal laws were reconstructed incorrectly.');

positiveAltitudeError = find(yOperational(:,9)-yOperational(:,8) > 0.01, ...
    1,'first');
assert(~isempty(positiveAltitudeError) && ...
    yOperational(positiveAltitudeError,11) > 0, ...
    'Positive altitude error did not command a climb.');
positiveVerticalSpeedError = find( ...
    yOperational(:,12)-yOperational(:,10) > 0.01,1,'first');
assert(~isempty(positiveVerticalSpeedError) && ...
    yOperational(positiveVerticalSpeedError,14) > 0, ...
    'Positive vertical-speed error did not command positive pitch.');
positiveAttitudeError = find( ...
    yOperational(:,6)-yOperational(:,4) > deg2rad(0.01),1,'first');
assert(~isempty(positiveAttitudeError) && ...
    yOperational(positiveAttitudeError,15) < 0, ...
    'Positive pitch-attitude error did not command trailing-edge-up elevator.');

%% Response metrics and requirements

smallMetrics = captureMetrics(tSmall,ySmall,smallCommand_m, ...
    pitchController,actuator,altitude,Vtrim);
operationalMetrics = captureMetrics(tOperational,yOperational, ...
    operationalCommand_m,pitchController,actuator,altitude,Vtrim);

req = altitude.requirements;
assert(smallMetrics.overshoot_pct <= ...
    req.maximumSmallSignalOvershoot_pct, ...
    'The 10-ft response exceeded the overshoot requirement.');
assert(smallMetrics.riseTime_s >= req.minimumSmallSignalRiseTime_s && ...
    smallMetrics.riseTime_s <= req.maximumSmallSignalRiseTime_s, ...
    'The 10-ft response failed the rise-time requirement.');
assert(smallMetrics.settlingTime_s <= ...
    req.maximumSmallSignalSettlingTime_s, ...
    'The 10-ft response exceeded the settling-time requirement.');
assert(smallMetrics.finalError_ft <= ...
    req.maximumSmallSignalFinalError_ft, ...
    'The 10-ft response failed final altitude tracking.');
assert(smallMetrics.peakPitchCommand_deg <= ...
    req.maximumSmallSignalPitchCommand_deg, ...
    'The 10-ft response used excessive pitch command.');
assert(smallMetrics.peakVerticalSpeed_fpm <= ...
    req.maximumSmallSignalVerticalSpeed_fpm, ...
    'The 10-ft response used excessive vertical speed.');
assert(~smallMetrics.verticalSpeedLimitActive && ...
    ~smallMetrics.pitchCommandLimitActive && ...
    ~smallMetrics.elevatorPositionLimitActive && ...
    ~smallMetrics.elevatorRateLimitActive, ...
    'A protection limit unexpectedly activated during the 10-ft response.');

assert(operationalMetrics.overshoot_pct <= ...
    req.maximumOperationalOvershoot_pct, ...
    'The 100-ft response exceeded the overshoot requirement.');
assert(operationalMetrics.riseTime_s >= ...
    req.minimumOperationalRiseTime_s && ...
    operationalMetrics.riseTime_s <= ...
    req.maximumOperationalRiseTime_s, ...
    'The 100-ft response failed the rise-time requirement.');
assert(operationalMetrics.settlingTime_s <= ...
    req.maximumOperationalSettlingTime_s, ...
    'The 100-ft response exceeded the settling-time requirement.');
assert(operationalMetrics.finalError_ft <= ...
    req.maximumOperationalFinalError_ft, ...
    'The 100-ft response failed final altitude tracking.');
assert(operationalMetrics.peakPitchCommand_deg <= ...
    req.maximumOperationalPitchCommand_deg+1.0e-10, ...
    'The 100-ft response exceeded pitch-command authority.');
assert(operationalMetrics.peakPitch_deg <= ...
    req.maximumOperationalPitch_deg, ...
    'The 100-ft response exceeded the pitch-attitude gate.');
assert(operationalMetrics.peakVerticalSpeed_fpm <= ...
    req.maximumOperationalVerticalSpeed_fpm, ...
    'The 100-ft response exceeded the vertical-speed gate.');
assert(operationalMetrics.peakElevator_deg <= ...
    req.maximumOperationalElevator_deg, ...
    'The 100-ft response exceeded the elevator gate.');
assert(operationalMetrics.peakActuatorRate_degps <= ...
    req.maximumOperationalActuatorRate_degps, ...
    'The 100-ft response exceeded the actuator-rate gate.');
assert(operationalMetrics.peakSpeedFraction <= ...
    req.maximumSpeedPerturbationFraction, ...
    'The 100-ft response left the speed-perturbation envelope.');
assert(operationalMetrics.peakAlpha_deg <= ...
    req.maximumAlphaPerturbation_deg, ...
    'The 100-ft response left the alpha envelope.');
assert(operationalMetrics.verticalSpeedLimitActive, ...
    'The 100-ft response did not exercise the intended vertical-speed cap.');
assert(~operationalMetrics.pitchCommandLimitActive, ...
    'Pitch-command limiting unexpectedly activated during the 100-ft response.');
assert(~operationalMetrics.elevatorPositionLimitActive, ...
    'Elevator authority unexpectedly activated during the 100-ft response.');
assert(~operationalMetrics.elevatorRateLimitActive, ...
    'Actuator rate limiting unexpectedly activated during the 100-ft response.');

%% Nominal linear stability check

Aclosed = nominalClosedLoopMatrix(plant,pitchController,actuator, ...
    altitude,Vtrim);
closedLoopPoles = eig(Aclosed);
assert(all(real(closedLoopPoles) < 0), ...
    'The nominal linear Altitude Hold V1 model is unstable.');
slowestDecayRate_per_s = min(abs(real(closedLoopPoles)));

%% Console report

fprintf('============================================================\n');
fprintf('       C172S ALTITUDE HOLD V1 - SIMULINK CHECK\n');
fprintf('============================================================\n\n');
fprintf('Model                               : %s\n',modelName);
fprintf('Frozen inner loop                   : Pitch-Attitude Hold V2\n');
fprintf('Altitude kinematics                 : hDot = Vtrim*(theta - alpha)\n');
fprintf('Altitude gain Kh                    : %.4f 1/s\n',Kh);
fprintf('Vertical-speed gain Kv              : %.4f rad/(m/s)\n',Kv);
fprintf('Altitude command-filter time const. : %.2f s\n', ...
    altitude.commandFilterTimeConstant_s);
fprintf('Vertical-speed command authority    : +/- %.0f ft/min\n', ...
    altitude.verticalSpeedCommandLimit_fpm);
fprintf('Pitch-command authority             : +/- %.1f deg\n', ...
    altitude.pitchCommandLimit_deg);
fprintf('Solver / fixed step                 : ode4 / %.4f s\n\n',fixedStep_s);

fprintf('10-ft maximum signal error          : %.9g\n',smallMaxError);
fprintf('10-ft RMS signal error              : %.9g\n',smallRmsError);
fprintf('100-ft maximum signal error         : %.9g\n',operationalMaxError);
fprintf('100-ft RMS signal error             : %.9g\n',operationalRmsError);
fprintf('Allowed error                       : %.3g\n',tolerance);
fprintf('Slowest nominal decay rate          : %.5f 1/s\n\n', ...
    slowestDecayRate_per_s);

printCaptureMetrics('Small-signal command',smallMetrics);
printCaptureMetrics('Protected operational command',operationalMetrics);

fprintf('PASS: Simulink matches independent RK4 references for both captures.\n');
fprintf('PASS: the 10-ft and protected 100-ft responses satisfy every gate.\n');
fprintf('PASS: only the intended vertical-speed cap activates at 100 ft.\n');
fprintf('PASS: outer-loop kinematics and all feedback directions are correct.\n');
fprintf('PASS: frozen V2 elevator-position, rate and anti-windup logic is preserved.\n');

%% Figures

figure('Name','C172S Altitude Hold Simulink Equivalence','Color','w');
tiledlayout(2,2);
nexttile;
plot(tSmall,ySmall(:,8)/0.3048,'LineWidth',1.4, ...
    'DisplayName','Simulink');
hold on;
plot(tSmall,ySmallReference(:,8)/0.3048,'--','LineWidth',1.1, ...
    'DisplayName','Independent RK4');
yline(altitude.validation.smallSignalCommand_ft,'k:','Command', ...
    'HandleVisibility','off');
grid on;
xlabel('Time (s)');
ylabel('Altitude (ft)');
title('10-ft Small-Signal Capture');
legend('Location','best');
nexttile;
plot(tSmall,ySmall(:,10)*196.850393700787,'LineWidth',1.4, ...
    'DisplayName','Simulink');
hold on;
plot(tSmall,ySmallReference(:,10)*196.850393700787,'--', ...
    'LineWidth',1.1,'DisplayName','Independent RK4');
grid on;
xlabel('Time (s)');
ylabel('Vertical speed (ft/min)');
title('10-ft Vertical Speed');
nexttile;
plot(tOperational,yOperational(:,8)/0.3048,'LineWidth',1.4, ...
    'DisplayName','Simulink');
hold on;
plot(tOperational,yOperationalReference(:,8)/0.3048,'--', ...
    'LineWidth',1.1,'DisplayName','Independent RK4');
yline(altitude.validation.operationalCommand_ft,'k:','Command', ...
    'HandleVisibility','off');
grid on;
xlabel('Time (s)');
ylabel('Altitude (ft)');
title('Protected 100-ft Capture');
nexttile;
plot(tOperational,yOperational(:,10)*196.850393700787, ...
    'LineWidth',1.4,'DisplayName','Actual');
hold on;
plot(tOperational,yOperational(:,12)*196.850393700787,'--', ...
    'LineWidth',1.2,'DisplayName','Command');
yline(altitude.verticalSpeedCommandLimit_fpm,'k:','Command cap', ...
    'HandleVisibility','off');
yline(-altitude.verticalSpeedCommandLimit_fpm,'k:', ...
    'HandleVisibility','off');
grid on;
xlabel('Time (s)');
ylabel('Vertical speed (ft/min)');
title('Protected Vertical-Speed Command');
legend('Location','best');
sgtitle('Altitude Hold V1: Simulink and Independent RK4 Equivalence');

figure('Name','C172S Altitude Hold Controller Signals','Color','w');
tiledlayout(3,1);
nexttile;
plot(tOperational,rad2deg(yOperational(:,14)),'--', ...
    'DisplayName','Pitch command');
hold on;
plot(tOperational,rad2deg(yOperational(:,4)),'LineWidth',1.4, ...
    'DisplayName','Aircraft pitch');
grid on;
ylabel('Angle (deg)');
title('Pitch Command and Response');
legend('Location','best');
nexttile;
plot(tOperational,rad2deg(yOperational(:,15)), ...
    'DisplayName','Unsaturated demand');
hold on;
plot(tOperational,rad2deg(yOperational(:,16)),'--', ...
    'DisplayName','Position command');
plot(tOperational,rad2deg(yOperational(:,7)),'LineWidth',1.4, ...
    'DisplayName','Actual elevator');
grid on;
ylabel('Elevator (deg)');
title('Frozen V2 Elevator Path');
legend('Location','best');
nexttile;
plot(tOperational,rad2deg(yOperational(:,17)),'LineWidth',1.4);
hold on;
yline(actuator.dynamics.rateLimit_degps,'k--','Rate limit', ...
    'HandleVisibility','off');
yline(-actuator.dynamics.rateLimit_degps,'k--', ...
    'HandleVisibility','off');
grid on;
xlabel('Time (s)');
ylabel('Elevator rate (deg/s)');
title('Actuator Rate');

%% Refresh the saved model after all simulations and plotting

modelFile = get_param(modelName,'FileName');
close_system(modelName,0);
load_system(modelFile);
open_system(modelName);
fprintf('Reloaded generated model to clear transient editor diagnostics.\n');

%% Local helpers

function [t,y] = extractArrayOutput(simOut,nSignal)
t = simOut.tout;
y = simOut.yout;
if isa(y,'timeseries')
    y = y.Data;
elseif isa(y,'Simulink.SimulationData.Dataset')
    y = y.getElement(1).Values.Data;
end
y = squeeze(y);
if size(y,1) ~= numel(t) && size(y,2) == numel(t)
    y = y.';
end
assert(size(y,2) == nSignal, ...
    'Expected %d logged signals but received %d.',nSignal,size(y,2));
end

function y = rk4Reference(t,z0,heightCommand,antiWindupEnabled, ...
    plant,pitchController,actuator,altitude,Vtrim)
dt = median(diff(t));
assert(max(abs(diff(t)-dt)) < 1.0e-10, ...
    'Reference integration requires uniform sample time.');
z = z0;
y = zeros(numel(t),18);
for k = 1:numel(t)
    [~,diagnostic] = altitudeDerivative(z,heightCommand, ...
        antiWindupEnabled,plant,pitchController,actuator,altitude,Vtrim);
    y(k,:) = outputVector(z,diagnostic).';
    if k < numel(t)
        k1 = altitudeDerivative(z,heightCommand,antiWindupEnabled, ...
            plant,pitchController,actuator,altitude,Vtrim);
        k2 = altitudeDerivative(z+0.5*dt*k1,heightCommand, ...
            antiWindupEnabled,plant,pitchController,actuator,altitude,Vtrim);
        k3 = altitudeDerivative(z+0.5*dt*k2,heightCommand, ...
            antiWindupEnabled,plant,pitchController,actuator,altitude,Vtrim);
        k4 = altitudeDerivative(z+dt*k3,heightCommand,antiWindupEnabled, ...
            plant,pitchController,actuator,altitude,Vtrim);
        z = z+(dt/6)*(k1+2*k2+2*k3+k4);
    end
end
end

function [zDot,d] = altitudeDerivative(z,heightCommand, ...
    antiWindupEnabled,plant,pitchController,actuator,altitude,Vtrim)
d = reconstructStateSignals(z,heightCommand,antiWindupEnabled, ...
    pitchController,actuator,altitude,Vtrim);
zDot = zeros(9,1);
zDot(1:4) = plant.A*z(1:4)+plant.B*z(7);
zDot(5) = d.eTheta+d.antiWindupCorrection;
zDot(6) = (d.thetaCommand-z(6))/ ...
    pitchController.commandFilterTimeConstant_s;
zDot(7) = d.limitedRate;
zDot(8) = d.verticalSpeed;
zDot(9) = (heightCommand-z(9))/altitude.commandFilterTimeConstant_s;
end

function d = reconstructStateSignals(z,~,antiWindupEnabled, ...
    pitchController,actuator,altitude,Vtrim)
Kh = altitude.altitudeGain_per_s;
Kv = altitude.verticalSpeedGain_radPerMps;
Kp = pitchController.Kp_radPerRad;
Ki = pitchController.Ki_radPerRadPerS;
Kq = actuator.integration.Kq_s;
tauActuator = actuator.dynamics.timeConstant_s;
Tt = actuator.antiWindup.trackingTimeConstant_s;
rateLimit = deg2rad(actuator.dynamics.rateLimit_degps);

d.verticalSpeed = Vtrim*(z(4)-z(2));
d.hDotCommandRaw = Kh*(z(9)-z(8));
d.hDotCommand = clamp(d.hDotCommandRaw, ...
    -altitude.verticalSpeedCommandLimit_mps, ...
    altitude.verticalSpeedCommandLimit_mps);
d.thetaCommandRaw = Kv*(d.hDotCommand-d.verticalSpeed);
d.thetaCommand = clamp(d.thetaCommandRaw, ...
    -altitude.pitchCommandLimit_rad,altitude.pitchCommandLimit_rad);
d.eTheta = z(6)-z(4);
d.deltaEUnsat = -Kp*d.eTheta-Ki*z(5)+Kq*z(3);
d.deltaEPosition = clamp(d.deltaEUnsat, ...
    actuator.authority.lowerLimit_rad,actuator.authority.upperLimit_rad);
d.rawRate = (d.deltaEPosition-z(7))/tauActuator;
d.limitedRate = clamp(d.rawRate,-rateLimit,rateLimit);
d.deltaETrack = z(7)+tauActuator*d.limitedRate;
d.antiWindupCorrection = antiWindupEnabled* ...
    (d.deltaEUnsat-d.deltaETrack)/(Ki*Tt);
end

function y = outputVector(z,d)
y = [z(1:4);z(5);z(6);z(7);z(8);z(9);d.verticalSpeed; ...
    d.hDotCommandRaw;d.hDotCommand;d.thetaCommandRaw;d.thetaCommand; ...
    d.deltaEUnsat;d.deltaEPosition;d.limitedRate; ...
    d.antiWindupCorrection];
end

function law = reconstructSignals(y,Kh,Kv,pitchController, ...
    actuator,altitude)
law.hDotCommand = clamp(Kh*(y(:,9)-y(:,8)), ...
    -altitude.verticalSpeedCommandLimit_mps, ...
    altitude.verticalSpeedCommandLimit_mps);
law.thetaCommand = clamp(Kv*(law.hDotCommand-y(:,10)), ...
    -altitude.pitchCommandLimit_rad,altitude.pitchCommandLimit_rad);
eTheta = y(:,6)-y(:,4);
law.deltaEUnsat = -pitchController.Kp_radPerRad*eTheta ...
    -pitchController.Ki_radPerRadPerS*y(:,5) ...
    +actuator.integration.Kq_s*y(:,3);
law.deltaEPosition = clamp(law.deltaEUnsat, ...
    actuator.authority.lowerLimit_rad,actuator.authority.upperLimit_rad);
law.rawRate = (law.deltaEPosition-y(:,7))/ ...
    actuator.dynamics.timeConstant_s;
rateLimit = deg2rad(actuator.dynamics.rateLimit_degps);
law.limitedRate = clamp(law.rawRate,-rateLimit,rateLimit);
deltaETrack = y(:,7)+actuator.dynamics.timeConstant_s*law.limitedRate;
law.antiWindupCorrection = (law.deltaEUnsat-deltaETrack)/ ...
    (pitchController.Ki_radPerRadPerS* ...
    actuator.antiWindup.trackingTimeConstant_s);
end

function metrics = captureMetrics(t,y,command,pitchController, ...
    actuator,altitude,Vtrim)
[metrics.riseTime_s,metrics.settlingTime_s,metrics.overshoot_pct] = ...
    stepMetrics(t,y(:,8),command);
metrics.command_ft = command/0.3048;
metrics.finalError_ft = abs(command-y(end,8))/0.3048;
metrics.peakPitchCommand_deg = max(abs(rad2deg(y(:,14))));
metrics.peakPitch_deg = max(abs(rad2deg(y(:,4))));
metrics.peakVerticalSpeed_fpm = ...
    max(abs(y(:,10)))*196.850393700787;
metrics.peakElevator_deg = max(abs(rad2deg(y(:,7))));
metrics.peakActuatorRate_degps = max(abs(rad2deg(y(:,17))));
metrics.peakSpeedFraction = max(abs(y(:,1)))/Vtrim;
metrics.peakAlpha_deg = max(abs(rad2deg(y(:,2))));
metrics.verticalSpeedLimitActive = any(abs(y(:,11)-y(:,12)) > 1.0e-10);
metrics.pitchCommandLimitActive = any(abs(y(:,13)-y(:,14)) > 1.0e-10);
metrics.elevatorPositionLimitActive = any(abs(y(:,15)-y(:,16)) > 1.0e-10);
rawRate = (y(:,16)-y(:,7))/actuator.dynamics.timeConstant_s;
metrics.elevatorRateLimitActive = any(abs(rawRate-y(:,17)) > 1.0e-8);

% Keep the pitch-controller argument explicit in the interface so this
% metric function documents the complete frozen-loop dependency.
assert(pitchController.Ki_radPerRadPerS > 0 && ...
    altitude.altitudeGain_per_s > 0,'Controller gains must be positive.');
end

function Aclosed = nominalClosedLoopMatrix(plant,pitchController, ...
    actuator,altitude,Vtrim)
Kp = pitchController.Kp_radPerRad;
Ki = pitchController.Ki_radPerRadPerS;
Kq = actuator.integration.Kq_s;
tauTheta = pitchController.commandFilterTimeConstant_s;
tauActuator = actuator.dynamics.timeConstant_s;
Kh = altitude.altitudeGain_per_s;
Kv = altitude.verticalSpeedGain_radPerMps;
tauAltitude = altitude.commandFilterTimeConstant_s;

% State order: [u alpha q theta xi thetaFiltered deltaE h hFiltered].
Aattitude = zeros(7,7);
Aattitude(1:4,1:4) = plant.A;
Aattitude(1:4,7) = plant.B;
Aattitude(5,4) = -1;
Aattitude(5,6) = 1;
Aattitude(6,6) = -1/tauTheta;
Aattitude(7,3) = Kq/tauActuator;
Aattitude(7,4) = Kp/tauActuator;
Aattitude(7,5) = -Ki/tauActuator;
Aattitude(7,6) = -Kp/tauActuator;
Aattitude(7,7) = -1/tauActuator;
Btheta = zeros(7,1);
Btheta(6) = 1/tauTheta;
CverticalSpeed = [0,-Vtrim,0,Vtrim,0,0,0];

Aclosed = zeros(9,9);
Aclosed(1:7,1:7) = Aattitude-Btheta*(Kv*CverticalSpeed);
Aclosed(1:7,8) = -Btheta*Kv*Kh;
Aclosed(1:7,9) = Btheta*Kv*Kh;
Aclosed(8,1:7) = CverticalSpeed;
Aclosed(9,9) = -1/tauAltitude;
end

function printCaptureMetrics(label,metrics)
fprintf('%s             : %.1f ft\n',label,metrics.command_ft);
fprintf('Overshoot                           : %.4f %%\n',metrics.overshoot_pct);
fprintf('10-90%% rise time                    : %.3f s\n',metrics.riseTime_s);
fprintf('2%% settling time                    : %.3f s\n',metrics.settlingTime_s);
fprintf('Final altitude error                : %.5f ft\n',metrics.finalError_ft);
fprintf('Peak pitch command                  : %.4f deg\n', ...
    metrics.peakPitchCommand_deg);
fprintf('Peak aircraft pitch                 : %.4f deg\n',metrics.peakPitch_deg);
fprintf('Peak vertical speed                 : %.3f ft/min\n', ...
    metrics.peakVerticalSpeed_fpm);
fprintf('Peak actual elevator                : %.4f deg\n',metrics.peakElevator_deg);
fprintf('Peak actuator rate                  : %.4f deg/s\n', ...
    metrics.peakActuatorRate_degps);
fprintf('Peak |u|/Vtrim                      : %.3f %%\n', ...
    100*metrics.peakSpeedFraction);
fprintf('Peak alpha perturbation             : %.4f deg\n',metrics.peakAlpha_deg);
fprintf('Limits active (hDot/theta/pos/rate) : %d / %d / %d / %d\n\n', ...
    metrics.verticalSpeedLimitActive,metrics.pitchCommandLimitActive, ...
    metrics.elevatorPositionLimitActive,metrics.elevatorRateLimitActive);
end

function y = clamp(x,lowerLimit,upperLimit)
y = min(max(x,lowerLimit),upperLimit);
end

function [riseTime,settlingTime,overshoot] = ...
    stepMetrics(t,response,command)
rise10Index = find(response >= 0.10*command,1,'first');
rise90Index = find(response >= 0.90*command,1,'first');
if isempty(rise10Index) || isempty(rise90Index)
    riseTime = inf;
else
    riseTime = t(rise90Index)-t(rise10Index);
end
outside = find(abs(response-command) > 0.02*abs(command));
if isempty(outside)
    settlingTime = 0;
elseif outside(end) == numel(t)
    settlingTime = inf;
else
    settlingTime = t(outside(end)+1);
end
overshoot = max(0,100*(max(response)-command)/abs(command));
end
