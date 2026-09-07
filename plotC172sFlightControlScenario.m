function figures = plotC172sFlightControlScenario(result,scenario,metrics)
%PLOTC172SFLIGHTCONTROLSCENARIO Plot flight, controls and mode evidence.

if nargin < 3 || isempty(metrics)
    metrics = analyzeC172sFlightControlScenario(result,scenario);
end
t = result.time_s(:);
heading_deg = mod(rad2deg(result.euler_rad(:,3)),360);
attitude_deg = rad2deg(result.euler_rad(:,1:2));
rates_degps = rad2deg(result.plantState(:,4:6));
surface_deg = rad2deg(result.control(:,1:3));
c = scenario.configuration;

figures = gobjects(2,1);
figures(1) = figure('Name', ...
    ['C172S Flight-Control Lab - ' scenario.name], ...
    'Color','w','NumberTitle','off');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax = nexttile;
if strcmp(c.lateralMode,'ROLL')
    plot(t,attitude_deg(:,1),'LineWidth',1.35);
    hold on;
    plot(t,result.bankCommand_deg,'--','LineWidth',1.05);
    ylabel('Bank (deg)'); title('Bank capture');
    referenceLabel = 'Captured bank';
else
    plot(t,heading_deg,'LineWidth',1.35);
    hold on;
    plot(t,result.headingCommand_deg,'--','LineWidth',1.05);
    ylabel('Heading (deg)'); title('Heading');
    referenceLabel = 'Selected heading';
end
grid on;
xlabel('Time (s)');
legend('Aircraft',referenceLabel,'Location','best');
addTimelineMarkers(ax,scenario,true);

ax = nexttile;
if strcmp(c.verticalMode,'PITCH')
    plot(t,attitude_deg(:,2),'LineWidth',1.35);
    hold on;
    plot(t,result.pitchCommand_deg,'--','LineWidth',1.05);
    ylabel('Pitch (deg)'); title('Pitch capture');
    referenceLabel = 'Captured pitch';
else
    plot(t,result.altitude_ft,'LineWidth',1.35);
    hold on;
    plot(t,result.altitudeCommand_ft,'--','LineWidth',1.05);
    ylabel('Altitude (ft)'); title('Altitude');
    referenceLabel = 'Selected altitude';
end
grid on;
xlabel('Time (s)');
legend('Aircraft',referenceLabel,'Location','best');
addTimelineMarkers(ax,scenario,false);

ax = nexttile;
plot(t,attitude_deg,'LineWidth',1.20);
grid on;
xlabel('Time (s)'); ylabel('Attitude (deg)'); title('Aircraft attitude');
legend('Bank','Pitch','Location','best');
addTimelineMarkers(ax,scenario,false);

ax = nexttile;
plot(t,rates_degps,'LineWidth',1.10);
grid on;
xlabel('Time (s)'); ylabel('Rate (deg/s)'); title('Body rates');
legend('p','q','r','Location','best');
addTimelineMarkers(ax,scenario,false);

figures(2) = figure('Name', ...
    ['C172S Flight-Control Lab - controls and health - ' scenario.name], ...
    'Color','w','NumberTitle','off');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax = nexttile;
plot(t,surface_deg,'LineWidth',1.15);
grid on;
xlabel('Time (s)'); ylabel('Surface (deg)'); title('Automatic surfaces');
legend('Elevator','Aileron','Rudder','Location','best');
addTimelineMarkers(ax,scenario,true);

ax = nexttile;
plot(t,rad2deg(result.alpha_rad),'LineWidth',1.15);
hold on;
plot(t,rad2deg(result.beta_rad),'LineWidth',1.15);
grid on;
xlabel('Time (s)'); ylabel('Angle (deg)'); title('Air-relative angles');
legend('Alpha','Beta','Location','best');
addTimelineMarkers(ax,scenario,false);

ax = nexttile;
stairs(t,double(result.lateralMode),'LineWidth',1.20);
hold on;
stairs(t,double(result.verticalMode),'LineWidth',1.20);
stairs(t,2.5*double(result.engaged),':','LineWidth',1.05);
grid on;
xlabel('Time (s)'); ylabel('Mode code'); title('Autopilot modes');
legend('Lateral','Vertical','Engaged x 2.5','Location','best');
addTimelineMarkers(ax,scenario,false);

ax = nexttile;
plot(t,result.TAS_mps/0.514444444444444,'LineWidth',1.15);
hold on;
violation = ~result.withinEnvelope;
if any(violation)
    plot(t(violation),result.TAS_mps(violation)/0.514444444444444, ...
        'rx','LineWidth',1.2);
end
grid on;
xlabel('Time (s)'); ylabel('TAS (KTAS)'); title('Air-data health');
if any(violation)
    legend('TAS','Envelope violation','Location','best');
else
    legend('TAS','Location','best');
end
addTimelineMarkers(ax,scenario,false);

sgtitle(sprintf('%s | %s | overall %s',scenario.name, ...
    finalModeLabel(metrics),passLabel(metrics.overallPass)));
end

function text = finalModeLabel(metrics)
if metrics.final.engaged
    text = sprintf('%s/%s',metrics.final.lateralMode, ...
        metrics.final.verticalMode);
else
    text = 'AUTOPILOT OFF';
end
end

function text = passLabel(status)
if status
    text = 'PASS';
else
    text = 'REVIEW';
end
end

function addTimelineMarkers(ax,scenario,showLabels)
c = scenario.configuration;
times = zeros(0,1);
labels = cell(0,1);
colors = zeros(0,3);
if c.engage
    add(c.engageTime_s,'Engage',[0.20,0.60,0.20]);
end
if c.headwind_mps ~= 0 || c.crosswindFromRight_mps ~= 0
    add(c.windStart_s,'Wind',[0.10,0.65,0.80]);
end
if c.upwardGust_mps > 0
    add(c.gustStart_s,'Gust start',[0.65,0.25,0.75]);
    add(c.gustStart_s+c.gustDuration_s,'Gust end',[0.65,0.25,0.75]);
end
if ~strcmp(c.eventType,'NONE')
    add(c.eventTime_s,strrep(c.eventType,'-',' '),[0.85,0.20,0.20]);
end
for k = 1:numel(times)
    if showLabels
        xline(ax,times(k),':',labels{k},'Color',colors(k,:), ...
            'LineWidth',0.85,'LabelVerticalAlignment','bottom', ...
            'HandleVisibility','off');
    else
        xline(ax,times(k),':','Color',colors(k,:), ...
            'LineWidth',0.85,'HandleVisibility','off');
    end
end

    function add(time,label,color)
        match = find(abs(times-time) <= 1e-9,1,'first');
        if isempty(match)
            times(end+1,1) = time; %#ok<AGROW>
            labels{end+1,1} = label; %#ok<AGROW>
            colors(end+1,:) = color; %#ok<AGROW>
        else
            labels{match} = [labels{match} ' + ' label];
        end
    end
end
