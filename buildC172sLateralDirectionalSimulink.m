function modelName = buildC172sLateralDirectionalSimulink(forceRebuild)
%BUILDC172SLATERALDIRECTIONALSIMULINK Build the Plant V0.1 Simulink model.
%
%   MODELNAME = BUILDC172SLATERALDIRECTIONALSIMULINK() creates or opens
%   C172S_LateralDirectionalPlant_V01.slx in the current folder.
%
%   MODELNAME = BUILDC172SLATERALDIRECTIONALSIMULINK(true) overwrites the
%   generated model. Use false to preserve an existing model.
%
%   State order: [beta; p; r; phi; psi]
%   Input order: [deltaA; deltaR]

arguments
    forceRebuild (1,1) logical = false
end

modelName = 'C172S_LateralDirectionalPlant_V01';
modelFile = fullfile(pwd,[modelName '.slx']);

%% Initialize variables used by the generated diagram

ldpData = c172sLateralDirectionalData();
[~,ldpPlant] = c172sLateralDirectionalPlant(ldpData);
assignModelVariables(ldpData,ldpPlant);

% Close any in-memory copy first so interrupted builder runs cannot leave a
% partial diagram loaded.
if bdIsLoaded(modelName)
    close_system(modelName,0);
end

if isfile(modelFile) && ~forceRebuild
    load_system(modelFile);
    open_system(modelName);
    fprintf('Opened existing model: %s\n',modelFile);
    return;
end

if isfile(modelFile) && forceRebuild
    delete(modelFile);
end

%% Create and configure the model

new_system(modelName);
open_system(modelName);

set_param(modelName, ...
    'SolverType','Fixed-step', ...
    'Solver','ode4', ...
    'FixedStep','0.01', ...
    'StartTime','0', ...
    'StopTime','20', ...
    'SaveTime','on', ...
    'TimeSaveName','tout', ...
    'SaveOutput','on', ...
    'OutputSaveName','yout', ...
    'SaveFormat','Array');

% Reconstruct the nominal plant before every run, while preserving command
% and initial-condition overrides supplied by the equivalence checker.
initCode = [ ...
    'ldpData = c172sLateralDirectionalData();' newline ...
    '[~,ldpPlant] = c172sLateralDirectionalPlant(ldpData);' newline ...
    'A_ldp = ldpPlant.A; B_ldp = ldpPlant.B;' newline ...
    'C_ldp = ldpPlant.C; D_ldp = ldpPlant.D;' newline ...
    'if ~exist(''deltaACommand_ldp_rad'',''var''), deltaACommand_ldp_rad = deg2rad(1.0); end' newline ...
    'if ~exist(''deltaRCommand_ldp_rad'',''var''), deltaRCommand_ldp_rad = deg2rad(1.0); end' newline ...
    'if ~exist(''x0_ldp'',''var''), x0_ldp = zeros(5,1); end'];
set_param(modelName,'InitFcn',initCode);

%% One-second aileron and rudder pulses

add_block('simulink/Sources/Step',[modelName '/Aileron Pulse Start'], ...
    'Time','0','Before','0','After','deltaACommand_ldp_rad', ...
    'SampleTime','0','Position',[40 130 110 160]);
add_block('simulink/Sources/Step',[modelName '/Aileron Pulse End'], ...
    'Time','1','Before','0','After','-deltaACommand_ldp_rad', ...
    'SampleTime','0','Position',[40 185 110 215]);
add_block('simulink/Math Operations/Sum',[modelName '/Aileron Command'], ...
    'Inputs','++','Position',[160 145 190 195]);

add_block('simulink/Sources/Step',[modelName '/Rudder Pulse Start'], ...
    'Time','5','Before','0','After','deltaRCommand_ldp_rad', ...
    'SampleTime','0','Position',[40 290 110 320]);
add_block('simulink/Sources/Step',[modelName '/Rudder Pulse End'], ...
    'Time','6','Before','0','After','-deltaRCommand_ldp_rad', ...
    'SampleTime','0','Position',[40 345 110 375]);
add_block('simulink/Math Operations/Sum',[modelName '/Rudder Command'], ...
    'Inputs','++','Position',[160 305 190 355]);

add_block('simulink/Signal Routing/Mux',[modelName '/Control Mux'], ...
    'Inputs','2','Position',[255 185 260 315]);

add_block('simulink/Continuous/State-Space', ...
    [modelName '/C172S Lateral-Directional Plant'], ...
    'A','A_ldp','B','B_ldp','C','C_ldp','D','D_ldp','X0','x0_ldp', ...
    'Position',[330 205 535 295]);

add_block('simulink/Signal Routing/Demux',[modelName '/State Demux'], ...
    'Outputs','5','Position',[600 145 605 355]);

%% Global signal tags

tagDefinitions = { ...
    'Aileron Tag','ldp_deltaA',[225 115 315 135]; ...
    'Rudder Tag','ldp_deltaR',[225 355 315 375]; ...
    'Beta Tag','ldp_beta',[665 135 735 155]; ...
    'p Tag','ldp_p',[665 180 735 200]; ...
    'r Tag','ldp_r',[665 225 735 245]; ...
    'Phi Tag','ldp_phi',[665 270 735 290]; ...
    'Psi Tag','ldp_psi',[665 315 735 335]};

for k = 1:size(tagDefinitions,1)
    addGlobalGoto(modelName,tagDefinitions{k,1},tagDefinitions{k,2}, ...
        tagDefinitions{k,3});
end

%% Verification subsystem

verificationPath = [modelName '/Verification Signals'];
add_block('simulink/Ports & Subsystems/Subsystem',verificationPath, ...
    'Position',[805 170 955 240]);
clearDefaultSubsystem(verificationPath);

% Output order: [beta, p, r, phi, psi, deltaA, deltaR].
verificationTags = { ...
    'ldp_beta','ldp_p','ldp_r','ldp_phi','ldp_psi', ...
    'ldp_deltaA','ldp_deltaR'};
verificationNames = { ...
    'Beta','p','r','Phi','Psi','Aileron Command','Rudder Command'};

for k = 1:numel(verificationTags)
    yPosition = 25 + 45*(k - 1);
    addFrom(verificationPath,verificationNames{k},verificationTags{k}, ...
        [25 yPosition 145 yPosition + 20]);
end

add_block('simulink/Signal Routing/Mux', ...
    [verificationPath '/Signal Mux'], ...
    'Inputs',num2str(numel(verificationTags)), ...
    'Position',[215 15 220 345]);
add_block('simulink/Sinks/Out1',[verificationPath '/Signals'], ...
    'Port','1','Position',[280 170 310 190]);

for k = 1:numel(verificationTags)
    add_line(verificationPath,sprintf('%s/1',verificationNames{k}), ...
        sprintf('Signal Mux/%d',k),'autorouting','on');
end
add_line(verificationPath,'Signal Mux/1','Signals/1','autorouting','on');

add_block('simulink/Sinks/Out1',[modelName '/Plant Signals'], ...
    'Port','1','Position',[1015 195 1045 215]);
add_line(modelName,'Verification Signals/1','Plant Signals/1', ...
    'autorouting','on');

%% Display subsystem

displayPath = [modelName '/Display Scopes'];
add_block('simulink/Ports & Subsystems/Subsystem',displayPath, ...
    'Position',[805 315 955 385]);
clearDefaultSubsystem(displayPath);
buildDisplaySubsystem(displayPath);

%% Connect and label the executable path

add_line(modelName,'Aileron Pulse Start/1','Aileron Command/1', ...
    'autorouting','on');
add_line(modelName,'Aileron Pulse End/1','Aileron Command/2', ...
    'autorouting','on');
add_line(modelName,'Rudder Pulse Start/1','Rudder Command/1', ...
    'autorouting','on');
add_line(modelName,'Rudder Pulse End/1','Rudder Command/2', ...
    'autorouting','on');

connectNamed(modelName,'Aileron Command/1','Control Mux/1', ...
    'deltaA rad, right-wing-down command positive');
connectNamed(modelName,'Rudder Command/1','Control Mux/2', ...
    'deltaR rad, nose-right command positive');
connectNamed(modelName,'Control Mux/1', ...
    'C172S Lateral-Directional Plant/1','u = [deltaA deltaR]');
connectNamed(modelName,'C172S Lateral-Directional Plant/1', ...
    'State Demux/1','x = [beta p r phi psi]');

% Feed every global tag from a short local branch. All corresponding From
% blocks already exist before the model is saved and reloaded.
branchTo(modelName,'Aileron Command/1','Aileron Tag/1');
branchTo(modelName,'Rudder Command/1','Rudder Tag/1');
stateTagNames = {'Beta Tag','p Tag','r Tag','Phi Tag','Psi Tag'};
for k = 1:5
    branchTo(modelName,sprintf('State Demux/%d',k), ...
        [stateTagNames{k} '/1']);
end

%% Diagram note and clean reload

try
    note = Simulink.Annotation(modelName, ...
        ['Lateral-Directional Plant V0.1: provisional research baseline' newline ...
         'x = [beta p r phi psi], u = [deltaA deltaR], all angles in radians' newline ...
         'deltaA positive right-wing-down; deltaR positive nose-right' newline ...
         'Fixed-step ode4 at 0.01 s; source/adaptation assumptions remain explicit in data file']);
    note.Position = [35 25 900 95];
catch
    % Annotation appearance is nonessential to the executable model.
end

save_system(modelName,modelFile);
close_system(modelName,0);
load_system(modelFile);
open_system(modelName);

fprintf('Created Simulink model: %s\n',modelFile);
fprintf('Run runLateralDirectionalSimulinkCheck to validate Plant V0.1.\n');

end

function assignModelVariables(data,plant)
assignin('base','ldpData',data);
assignin('base','ldpPlant',plant);
assignin('base','A_ldp',plant.A);
assignin('base','B_ldp',plant.B);
assignin('base','C_ldp',plant.C);
assignin('base','D_ldp',plant.D);
assignin('base','deltaACommand_ldp_rad',deg2rad(1.0));
assignin('base','deltaRCommand_ldp_rad',deg2rad(1.0));
assignin('base','x0_ldp',zeros(5,1));
end

function buildDisplaySubsystem(path)
tags = {'ldp_beta','ldp_p','ldp_r','ldp_phi','ldp_psi', ...
    'ldp_deltaA','ldp_deltaR'};
names = {'Beta','p','r','Phi','Psi','Aileron','Rudder'};

for k = 1:numel(tags)
    yPosition = 25 + 45*(k - 1);
    addFrom(path,names{k},tags{k},[25 yPosition 135 yPosition + 20]);
    add_block('simulink/Math Operations/Gain', ...
        [path '/' names{k} ' rad-to-deg'], ...
        'Gain','180/pi','Position',[175 yPosition - 5 260 yPosition + 25]);
    add_line(path,[names{k} '/1'],[names{k} ' rad-to-deg/1'], ...
        'autorouting','on');
end

add_block('simulink/Signal Routing/Mux',[path '/State Mux'], ...
    'Inputs','5','Position',[320 15 325 245]);
add_block('simulink/Sinks/Scope',[path '/State Scope'], ...
    'Position',[385 110 430 145]);
for k = 1:5
    add_line(path,[names{k} ' rad-to-deg/1'],sprintf('State Mux/%d',k), ...
        'autorouting','on');
end
add_line(path,'State Mux/1','State Scope/1','autorouting','on');

add_block('simulink/Signal Routing/Mux',[path '/Control Mux'], ...
    'Inputs','2','Position',[320 255 325 345]);
add_block('simulink/Sinks/Scope',[path '/Control Scope'], ...
    'Position',[385 290 430 325]);
for k = 6:7
    add_line(path,[names{k} ' rad-to-deg/1'], ...
        sprintf('Control Mux/%d',k - 5),'autorouting','on');
end
add_line(path,'Control Mux/1','Control Scope/1','autorouting','on');
end

function clearDefaultSubsystem(path)
lines = find_system(path,'FindAll','on','SearchDepth',1,'Type','line');
if ~isempty(lines)
    delete_line(lines);
end
blocks = find_system(path,'SearchDepth',1,'Type','Block');
for k = 2:numel(blocks)
    delete_block(blocks{k});
end
end

function addGlobalGoto(systemName,blockName,tagName,position)
add_block('simulink/Signal Routing/Goto',[systemName '/' blockName], ...
    'GotoTag',tagName,'TagVisibility','global','Position',position);
end

function addFrom(systemName,blockName,tagName,position)
add_block('simulink/Signal Routing/From',[systemName '/' blockName], ...
    'GotoTag',tagName,'Position',position);
end

function connectNamed(systemName,source,destination,name)
h = add_line(systemName,source,destination,'autorouting','on');
set_param(h,'Name',name);
end

function branchTo(systemName,source,destination)
add_line(systemName,source,destination,'autorouting','on');
end
