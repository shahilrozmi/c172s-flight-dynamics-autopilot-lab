function modelName = buildC172sIntegratedAutopilotSimulink(forceRebuild)
%BUILDC172SINTEGRATEDAUTOPILOTSIMULINK Build Integrated Autopilot V1.
%
% The generated model copies the already validated executable supervisor,
% Altitude Hold V1 and Heading Hold V1 diagrams into a new integration
% harness. Only their validation sources are replaced by external mode-
% manager routing. The frozen source models and controller files are not
% modified.

arguments
    forceRebuild (1,1) logical = false
end

modelName = 'C172S_IntegratedAutopilot_V1';
modelFile = fullfile(pwd,[modelName '.slx']);
integration = c172sIntegratedAutopilotData();
manager = c172sAutopilotModeManagerData();
assertDependencies(integration,manager);

sourceModels = { ...
    'C172S_AutopilotModeManager_V1', ...
    'C172S_AltitudeHold_V1', ...
    'C172S_HeadingHold_V1'};
sourceBuilders = { ...
    'buildC172sAutopilotModeManagerSimulink', ...
    'buildC172sAltitudeHoldSimulink', ...
    'buildC172sHeadingHoldSimulink'};
for k = 1:numel(sourceModels)
    if ~isfile(fullfile(pwd,[sourceModels{k} '.slx']))
        feval(sourceBuilders{k},true);
    end
    load_system(sourceModels{k});
end

assignin('base','ia1InputSignal',defaultInputSignal(manager));
sourceInit = cellfun(@(name)get_param(name,'InitFcn'),sourceModels, ...
    'UniformOutput',false);
if bdIsLoaded(modelName), close_system(modelName,0); end
if isfile(modelFile) && ~forceRebuild
    load_system(modelFile);
    open_system(modelName);
    fprintf('Opened existing model: %s\n',modelFile);
    return;
end
if isfile(modelFile), delete(modelFile); end
load_system('simulink');
new_system(modelName);
open_system(modelName);
set_param(modelName, ...
    'SolverType','Fixed-step','Solver','ode4','FixedStep','0.01', ...
    'StartTime','0','StopTime','180','SaveTime','on','TimeSaveName','tout', ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Array');

% The copied supervisor chart contains literal audited constants. Its source
% model callback is deliberately omitted because that callback requires the
% standalone amm1InputSignal source that has been replaced here.
set_param(modelName,'InitFcn',[sourceInit{2} newline sourceInit{3} newline ...
    'ia1Integration = c172sIntegratedAutopilotData();' newline ...
    'if ~strcmp(ia1Integration.artifactRevision,''INTEGRATED-AUTOPILOT-V1-INTERFACE-AUDIT-MATLAB-R1''), error(''Integrated Autopilot V1 audit mismatch.''); end' newline ...
    'if ~exist(''ia1InputSignal'',''var''), error(''ia1InputSignal is required.''); end' newline ...
    'ia1TrimAltitude_ft = 4000.0;']);

%% Copy the frozen executable contents into generated subsystems.

supervisorPath = [modelName '/Frozen Executable Mode Manager V1'];
longitudinalPath = [modelName '/Frozen Longitudinal Stack'];
lateralPath = [modelName '/Frozen Lateral Stack'];
add_block('simulink/Ports & Subsystems/Subsystem',supervisorPath, ...
    'Position',[510 80 770 250]);
add_block('simulink/Ports & Subsystems/Subsystem',longitudinalPath, ...
    'Position',[980 320 1240 520]);
add_block('simulink/Ports & Subsystems/Subsystem',lateralPath, ...
    'Position',[980 580 1240 780]);
clearDefaultSubsystem(supervisorPath);
clearDefaultSubsystem(longitudinalPath);
clearDefaultSubsystem(lateralPath);
Simulink.BlockDiagram.copyContentsToSubsystem(sourceModels{1},supervisorPath);
Simulink.BlockDiagram.copyContentsToSubsystem(sourceModels{2},longitudinalPath);
Simulink.BlockDiagram.copyContentsToSubsystem(sourceModels{3},lateralPath);

externalizeSupervisor(supervisorPath);
configureSupervisorSampleTime(supervisorPath, ...
    integration.execution.supervisorRate_s);
externalizeLongitudinal(longitudinalPath);
externalizeLateral(lateralPath);

%% Timed requests and live-state substitution into the 19-input supervisor.

add_block('simulink/Sources/From Workspace',[modelName '/Timed Requests'], ...
    'VariableName','ia1InputSignal','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','SampleTime','0.01', ...
    'Position',[35 80 205 120]);
add_block('simulink/Signal Routing/Demux',[modelName '/Request Demux'], ...
    'Outputs','19','Position',[255 20 260 470]);
add_block('simulink/Signal Routing/Mux',[modelName '/Live Supervisor Input Mux'], ...
    'Inputs','19','Position',[430 20 435 470]);
add_line(modelName,'Timed Requests/1','Request Demux/1','autorouting','on');

% Live state conversions. The longitudinal altitude state is a perturbation
% about the 4000-ft integration trim; the supervisor uses absolute altitude.
add_block('simulink/Signal Routing/Demux',[modelName '/Longitudinal Demux'], ...
    'Outputs','18','Position',[1320 300 1325 550]);
add_block('simulink/Signal Routing/Demux',[modelName '/Lateral Demux'], ...
    'Outputs','52','Position',[1320 565 1325 815]);
add_block('simulink/Math Operations/Gain',[modelName '/Pitch rad to deg'], ...
    'Gain','180/pi','Position',[1390 355 1490 385]);
add_block('simulink/Math Operations/Gain',[modelName '/Altitude m to ft'], ...
    'Gain','1/0.3048','Position',[1390 410 1490 440]);
add_block('simulink/Math Operations/Bias',[modelName '/Trim Altitude'], ...
    'Bias','4000','Position',[1530 410 1625 440]);
add_block('simulink/Math Operations/Gain',[modelName '/Bank rad to deg'], ...
    'Gain','180/pi','Position',[1390 625 1490 655]);
add_block('simulink/Math Operations/Gain',[modelName '/Heading rad to deg'], ...
    'Gain','180/pi','Position',[1390 680 1490 710]);

add_line(modelName,'Frozen Longitudinal Stack/1','Longitudinal Demux/1', ...
    'autorouting','on');
add_line(modelName,'Frozen Lateral Stack/1','Lateral Demux/1','autorouting','on');
add_line(modelName,'Longitudinal Demux/4','Pitch rad to deg/1','autorouting','on');
add_line(modelName,'Longitudinal Demux/8','Altitude m to ft/1','autorouting','on');
add_line(modelName,'Altitude m to ft/1','Trim Altitude/1','autorouting','on');
add_line(modelName,'Lateral Demux/4','Bank rad to deg/1','autorouting','on');
add_line(modelName,'Lateral Demux/5','Heading rad to deg/1','autorouting','on');

for k = 1:19
    if ~ismember(k,14:17)
        add_line(modelName,sprintf('Request Demux/%d',k), ...
            sprintf('Live Supervisor Input Mux/%d',k),'autorouting','on');
    end
end
add_line(modelName,'Bank rad to deg/1','Live Supervisor Input Mux/14', ...
    'autorouting','on');
add_line(modelName,'Pitch rad to deg/1','Live Supervisor Input Mux/15', ...
    'autorouting','on');
add_line(modelName,'Heading rad to deg/1','Live Supervisor Input Mux/16', ...
    'autorouting','on');
add_line(modelName,'Trim Altitude/1','Live Supervisor Input Mux/17', ...
    'autorouting','on');
add_line(modelName,'Live Supervisor Input Mux/1', ...
    'Frozen Executable Mode Manager V1/1','autorouting','on');

%% Route the 20 supervisor outputs into both frozen stacks.

add_block('simulink/Signal Routing/Demux',[modelName '/Supervisor Demux'], ...
    'Outputs','20','Position',[835 40 840 285]);
add_line(modelName,'Frozen Executable Mode Manager V1/1', ...
    'Supervisor Demux/1','autorouting','on');

% Longitudinal ports: altitude command ft, pitch command deg,
% altitude-mode enable, engaged.
add_line(modelName,'Supervisor Demux/11','Frozen Longitudinal Stack/1', ...
    'autorouting','on');
add_line(modelName,'Supervisor Demux/9','Frozen Longitudinal Stack/2', ...
    'autorouting','on');
add_line(modelName,'Supervisor Demux/7','Frozen Longitudinal Stack/3', ...
    'autorouting','on');
add_line(modelName,'Supervisor Demux/1','Frozen Longitudinal Stack/4', ...
    'autorouting','on');

% Lateral ports: heading command deg, bank command deg,
% heading-mode enable, engaged.
add_line(modelName,'Supervisor Demux/10','Frozen Lateral Stack/1', ...
    'autorouting','on');
add_line(modelName,'Supervisor Demux/8','Frozen Lateral Stack/2', ...
    'autorouting','on');
add_line(modelName,'Supervisor Demux/5','Frozen Lateral Stack/3', ...
    'autorouting','on');
add_line(modelName,'Supervisor Demux/1','Frozen Lateral Stack/4', ...
    'autorouting','on');

%% Fixed 90-signal verification ledger.

add_block('simulink/Signal Routing/Mux',[modelName '/Integrated Signal Mux'], ...
    'Inputs','3','DisplayOption','bar','Position',[1710 330 1715 670]);
add_block('simulink/Sinks/Out1',[modelName '/Integrated Autopilot Signals'], ...
    'Port','1','Position',[1810 490 1840 510]);
add_line(modelName,'Frozen Executable Mode Manager V1/1', ...
    'Integrated Signal Mux/1','autorouting','on');
add_line(modelName,'Frozen Longitudinal Stack/1', ...
    'Integrated Signal Mux/2','autorouting','on');
add_line(modelName,'Frozen Lateral Stack/1', ...
    'Integrated Signal Mux/3','autorouting','on');
add_line(modelName,'Integrated Signal Mux/1', ...
    'Integrated Autopilot Signals/1','autorouting','on');

try
    note = Simulink.Annotation(modelName,sprintf([ ...
        'INTEGRATED AUTOPILOT V1\n' ...
        '20 supervisor + 18 longitudinal + 52 lateral = 90 verification signals\n' ...
        'Simultaneous single-trim plants; no cross-axis aerodynamic coupling']));
    note.Position = [35 505 700 590];
catch
end

set_param(modelName,'SimulationCommand','update');
save_system(modelName,modelFile);
for k = 1:numel(sourceModels), close_system(sourceModels{k},0); end
close_system(modelName,0);
load_system(modelFile);
open_system(modelName);
fprintf('Created Simulink model: %s\n',modelFile);
fprintf('Run runIntegratedAutopilotSimulinkCheck to validate Integrated Autopilot V1.\n');
end

function externalizeSupervisor(path)
safeDeleteLine(path,'Timed Supervisor Inputs/1','Input Demux/1');
delete_block([path '/Timed Supervisor Inputs']);
add_block('simulink/Ports & Subsystems/In1',[path '/Supervisor Inputs'], ...
    'Port','1','Position',[75 205 105 225]);
add_line(path,'Supervisor Inputs/1','Input Demux/1','autorouting','on');
end

function configureSupervisorSampleTime(path,sampleTime_s)
% The standalone Mode Manager model is discrete-only, so its MATLAB
% Function chart originally inherited a discrete rate. Once copied into the
% mixed continuous/discrete integration model, inheritance would make the
% chart continuous and invalidate its persistent state. Freeze the audited
% supervisor execution rate explicitly.
block = [path '/Independent Executable Mode Manager'];
configuration = get_param(block,'MATLABFunctionConfiguration');
configuration.UpdateMethod = 'Discrete';
configuration.SampleTime = num2str(sampleTime_s,17);
end

function externalizeLongitudinal(path)
safeDeleteLine(path,'Altitude Command/1','Altitude Command Filter/1');
delete_block([path '/Altitude Command']);
addIn(path,'Altitude Command ft',1,[35 65 65 85]);
addIn(path,'Pitch Command deg',2,[900 250 930 270]);
addIn(path,'Altitude Mode Enable',3,[900 285 930 305]);
addIn(path,'Autopilot Engaged',4,[1690 360 1720 380]);
add_block('simulink/Math Operations/Bias',[path '/Remove Trim Altitude'], ...
    'Bias','-4000','Position',[100 55 210 85]);
add_block('simulink/Math Operations/Gain',[path '/Altitude ft to m'], ...
    'Gain','0.3048','Position',[235 55 330 85]);
add_line(path,'Altitude Command ft/1','Remove Trim Altitude/1','autorouting','on');
add_line(path,'Remove Trim Altitude/1','Altitude ft to m/1','autorouting','on');
add_line(path,'Altitude ft to m/1','Altitude Command Filter/1','autorouting','on');

safeDeleteLine(path,'Pitch-Command Limit/1','Pitch Command Filter/1');
add_block('simulink/Math Operations/Gain',[path '/Pitch deg to rad'], ...
    'Gain','pi/180','Position',[960 245 1060 275]);
add_block('simulink/Signal Routing/Switch',[path '/Vertical Mode Select'], ...
    'Criteria','u2 >= Threshold','Threshold','0.5', ...
    'Position',[1100 95 1140 165]);
add_line(path,'Pitch Command deg/1','Pitch deg to rad/1','autorouting','on');
add_line(path,'Pitch-Command Limit/1','Vertical Mode Select/1','autorouting','on');
add_line(path,'Altitude Mode Enable/1','Vertical Mode Select/2','autorouting','on');
add_line(path,'Pitch deg to rad/1','Vertical Mode Select/3','autorouting','on');
add_line(path,'Vertical Mode Select/1','Pitch Command Filter/1','autorouting','on');

safeDeleteLine(path,'Unsaturated Total Command/1','Actuator and Anti-Windup/1');
add_line(path,'Unsaturated Total Command/1','Actuator and Anti-Windup/1', ...
    'autorouting','on');
gateElevatorActuator([path '/Actuator and Anti-Windup']);
add_line(path,'Autopilot Engaged/1','Actuator and Anti-Windup/2', ...
    'autorouting','on');
end

function externalizeLateral(path)
safeDeleteLine(path,'Heading Command/1','Heading Sensor and Outer Loop/1');
delete_block([path '/Heading Command']);
addIn(path,'Heading Command deg',1,[35 55 65 75]);
addIn(path,'Bank Command deg',2,[330 235 360 255]);
addIn(path,'Heading Mode Enable',3,[330 270 360 290]);
addIn(path,'Autopilot Engaged',4,[520 690 550 710]);
add_block('simulink/Math Operations/Gain',[path '/Heading deg to rad'], ...
    'Gain','pi/180','Position',[80 45 175 75]);
add_line(path,'Heading Command deg/1','Heading deg to rad/1','autorouting','on');
add_line(path,'Heading deg to rad/1','Heading Sensor and Outer Loop/1', ...
    'autorouting','on');

safeDeleteLine(path,'Heading Sensor and Outer Loop/1','Command Filter/1');
add_block('simulink/Math Operations/Gain',[path '/Bank deg to rad'], ...
    'Gain','pi/180','Position',[375 225 465 255]);
add_block('simulink/Signal Routing/Switch',[path '/Lateral Mode Select'], ...
    'Criteria','u2 >= Threshold','Threshold','0.5', ...
    'Position',[500 100 540 170]);
add_line(path,'Bank Command deg/1','Bank deg to rad/1','autorouting','on');
add_line(path,'Heading Sensor and Outer Loop/1','Lateral Mode Select/1', ...
    'autorouting','on');
add_line(path,'Heading Mode Enable/1','Lateral Mode Select/2','autorouting','on');
add_line(path,'Bank deg to rad/1','Lateral Mode Select/3','autorouting','on');
add_line(path,'Lateral Mode Select/1','Command Filter/1','autorouting','on');

gateProtectedActuator([path '/Frozen Protected Roll-Attitude Hold V1'], ...
    'Unsaturated Aileron Command','Automatic Aileron Position Limit', ...
    'Aileron Engagement Gate',4);
add_line(path,'Autopilot Engaged/1', ...
    'Frozen Protected Roll-Attitude Hold V1/4','autorouting','on');
gateProtectedActuator([path '/Frozen Protected Yaw Damper V2'], ...
    'Automatic Rudder Gain','Automatic Rudder Position Limit', ...
    'Rudder Engagement Gate',2);
add_line(path,'Autopilot Engaged/1', ...
    'Frozen Protected Yaw Damper V2/2','autorouting','on');
end

function gateProtectedActuator(path,sourceBlock,limitBlock,gateName,port)
safeDeleteLine(path,[sourceBlock '/1'],[limitBlock '/1']);
add_block('simulink/Ports & Subsystems/In1',[path '/Autopilot Engaged'], ...
    'Port',num2str(port),'Position',[20 675 50 695]);
add_block('simulink/Sources/Constant',[path '/Released Command'], ...
    'Value','0','Position',[340 680 390 700]);
add_block('simulink/Signal Routing/Switch',[path '/' gateName], ...
    'Criteria','u2 >= Threshold','Threshold','0.5', ...
    'Position',[430 535 470 605]);
add_line(path,[sourceBlock '/1'],[gateName '/1'],'autorouting','on');
add_line(path,'Autopilot Engaged/1',[gateName '/2'],'autorouting','on');
add_line(path,'Released Command/1',[gateName '/3'],'autorouting','on');
add_line(path,[gateName '/1'],[limitBlock '/1'],'autorouting','on');
end

function gateElevatorActuator(path)
safeDeleteLine(path,'Position Authority/1','Position Error/1');
add_block('simulink/Ports & Subsystems/In1',[path '/Autopilot Engaged'], ...
    'Port','2','Position',[20 425 50 445]);
add_block('simulink/Sources/Constant',[path '/Released Position Command'], ...
    'Value','0','Position',[105 425 160 445]);
add_block('simulink/Signal Routing/Switch',[path '/Elevator Engagement Gate'], ...
    'Criteria','u2 >= Threshold','Threshold','0.5', ...
    'Position',[210 55 250 125]);
add_line(path,'Position Authority/1','Elevator Engagement Gate/1', ...
    'autorouting','on');
add_line(path,'Autopilot Engaged/1','Elevator Engagement Gate/2', ...
    'autorouting','on');
add_line(path,'Released Position Command/1','Elevator Engagement Gate/3', ...
    'autorouting','on');
add_line(path,'Elevator Engagement Gate/1','Position Error/1','autorouting','on');
end

function addIn(path,name,port,position)
add_block('simulink/Ports & Subsystems/In1',[path '/' name], ...
    'Port',num2str(port),'Position',position);
end

function clearDefaultSubsystem(path)
% Library-created Subsystem blocks can retain a default signal line even
% after their named In1/Out1 blocks are deleted. The dedicated API removes
% every block, line and annotation and satisfies copyContentsToSubsystem's
% strict empty-destination precondition in R2026a.
Simulink.SubSystem.deleteContents(path);
end

function safeDeleteLine(path,source,destination)
try
    delete_line(path,source,destination);
catch ME
    error('buildC172sIntegratedAutopilotSimulink:MissingFrozenConnection', ...
        'Expected frozen connection %s -> %s was not found in %s. %s', ...
        source,destination,path,ME.message);
end
end

function signal = defaultInputSignal(manager)
row = zeros(1,19);
row(4:7) = 1.0;
row(10) = double(manager.codes.lateral.OFF);
row(11) = double(manager.codes.vertical.OFF);
row(17) = 4000.0;
row(19) = 4000.0;
signal = [0 row;1 row];
end

function assertDependencies(integration,manager)
altitude = c172sAltitudeHoldControllerData();
heading = c172sHeadingHoldControllerData();
assert(strcmp(manager.artifactRevision,integration.required.modeManager));
assert(abs(integration.execution.fixedStep_s-manager.validation.sampleTime_s) < eps);
assert(strcmp(altitude.version,integration.required.altitudeLoop));
assert(strcmp(heading.artifactRevision,integration.required.headingController));
end
