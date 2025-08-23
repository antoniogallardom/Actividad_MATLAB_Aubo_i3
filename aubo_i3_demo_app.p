%% AUBO i3 – realtime control (knobs + numeric spinners, colored links via patches)
% Immediate update with *soft* rate-limited motion, synced UI, single camlight.
% Put next to: aubo_i3.urdf and link0.stl … link6.stl

clear; clc;

%% 1) Load URDF
thisDir  = pwd;
urdfFile = fullfile(thisDir,'aubo_i3.urdf');
if ~isfile(urdfFile)
    [f,p] = uigetfile({'*.urdf','URDF files (*.urdf)'}, 'Select AUBO i3 URDF');
    if isequal(f,0), error('URDF not selected.'); end
    urdfFile = fullfile(p,f);
end
fprintf('Loading URDF: %s\n', urdfFile);
robot = importrobot(urdfFile);
robot.DataFormat = 'row';
robot.Gravity    = [0 0 -9.81];

%% 2) Joint vector + names
q = reshape(homeConfiguration(robot),1,[]);
n = numel(q);

oldFmt = robot.DataFormat;
robot.DataFormat = 'struct';
cfgS   = homeConfiguration(robot);
jNames = strings(1,n);
for i = 1:n, jNames(i) = string(cfgS(i).JointName); end
robot.DataFormat = oldFmt;

%% 3) Joint limits (radians) with robust defaults
lims = zeros(n,2);
for i = 1:n
    ji = findJointObject(robot, jNames(i));   % helper at end
    if strcmp(ji.Type,'prismatic'), lims(i,:) = [-0.10 0.10];
    else,                             lims(i,:) = [-pi pi];
    end
    if isprop(ji,'PositionLimits')
        L = ji.PositionLimits;
        if isnumeric(L) && numel(L)==2 && all(isfinite(L)) && L(1)<L(2)
            lims(i,:) = L(:).';
        end
    end
end

%% 4) Windows
% --- Visualization (classic axes so show() works)
visFig = figure('Name','AUBO i3 – View','NumberTitle','off');
ax = axes('Parent',visFig); hold(ax,'on');
axis(ax,'equal'); grid(ax,'on'); box(ax,'on'); view(ax,135,20);
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z'); title(ax,'AUBO i3');
show(robot, q, 'Parent', ax, 'Frames','off', 'PreservePlot',false);
lighting gouraud; material dull;
hLight = camlight(ax,'headlight');               % single reusable light
recolorAuboPatches(ax);                          % color links after draw

% --- Controls (spinner + knob per joint)
ctrlFig = uifigure('Name','AUBO i3 – Controls');
g = uigridlayout(ctrlFig,[2 1]); g.RowHeight = {'1x',48}; g.Padding = [8 8 8 8]; g.RowSpacing = 6;

rows = max(1, ceil(n/3));
pg = uigridlayout(g,[rows,3]); pg.Layout.Row = 1;
pg.RowHeight = repmat({'1x'},1,rows); pg.ColumnWidth = {'1x','1x','1x'};

% Shared state
S.q      = q;                 % current joints [rad]
S.target = q;                 % target joints [rad]
S.ax     = ax;                % classic axes handle
S.robot  = robot;
S.knobs  = gobjects(1,n);     % knob handles
S.spin   = gobjects(1,n);     % spinner handles
S.light  = hLight;
S.lims   = lims;

% --- softness / animation params ---
S.vmaxDegPS  = 1000;            % max joint speed (deg/s) during motion
S.settleTime = 0.4;           % seconds to finish remaining motion after release
S.lastNow    = now*ones(1,n); % per-joint timestamp for rate limiting

% --- homing params (run-to-completion) ---
S.homeTolDeg  = 0.05;         % stop when every joint within this tolerance (deg)
S.maxHomeTime = 100;            % safety timeout (s)

% Workspace & world axes settings
S.wsHandle  = gobjects(1,1);
S.wsVisible = false;
S.wsRadius  = 0.66;           % your tuned radius (approx reachable)
S.wsCenter  = [];             % estimated later from model
S.wsZBump   = 0.33;           % small upward offset if desired
S.pad       = 0.12;           % margin around sphere for axes limits
S.worldAxes = struct('x',gobjects(1,1),'y',gobjects(1,1),'z',gobjects(1,1));

ctrlFig.UserData = S;

% Per-joint UI (vertical: spinner over knob)
for i = 1:n
    idx = i;
    slot = uigridlayout(pg,[2 1]); slot.RowHeight = {'fit','1x'}; slot.ColumnWidth = {'1x'};

    % Spinner (degrees)
    e = uispinner(slot, 'Limits',rad2deg(lims(idx,:)), 'Step',1, 'Value',rad2deg(q(idx)));
    e.Tooltip = sprintf('Joint %d (%s) – degrees', idx, jNames(idx));
    e.ValueChangingFcn = @(src,evt) softSet(ctrlFig, idx, deg2rad(evt.Value), false);
    e.ValueChangedFcn  = @(src,evt) softSet(ctrlFig, idx, deg2rad(src.Value),  true);

    % Knob (degrees)
    k = uiknob(slot,'continuous');
    k.Limits = rad2deg(lims(idx,:));
    k.Value  = rad2deg(q(idx));
    k.MajorTicks = round(linspace(k.Limits(1),k.Limits(2),5),-1);
    k.Tooltip = sprintf('Joint %d (%s)', idx, jNames(idx));
    k.ValueChangingFcn = @(src,evt) softSet(ctrlFig, idx, deg2rad(evt.Value), false);
    k.ValueChangedFcn  = @(src,evt) softSet(ctrlFig, idx, deg2rad(src.Value),  true);

    S = ctrlFig.UserData; S.knobs(idx) = k; S.spin(idx) = e; ctrlFig.UserData = S;
end

% Buttons row
btns = uigridlayout(g,[1 3]); btns.Layout.Row = 2; btns.ColumnWidth = {'1x','1x','1x'};
uibutton(btns,'Text','Home',           'ButtonPushedFcn', @(btn,~) kHome(ctrlFig, btn));
uibutton(btns,'Text','Reset Camera',   'ButtonPushedFcn', @(~,~) kCam(ctrlFig));
uibutton(btns,'Text','Show Workspace', 'ButtonPushedFcn', @(src,~) toggleWorkspace(ctrlFig,src));

% Initial axis limits & world axes centered at base
S = ctrlFig.UserData;
S.wsCenter = estimateBaseCenter(S.robot) + [0 0 S.wsZBump];
ctrlFig.UserData = S;
adjustAxesToWorkspace(ctrlFig);   % sets X/Y/Z limits and draws origin axes

% Close both windows together
ctrlFig.CloseRequestFcn = @(src,evt) closeBoth(ctrlFig, visFig);
visFig.CloseRequestFcn  = @(src,evt) closeBoth(ctrlFig, visFig);

%% ===== Local functions ====================================================
function softSet(ctrlFig, idx, newTarget, settleAfter)
S = ctrlFig.UserData;

% clamp to limits & set target
L = S.lims(idx,:); newTarget = max(min(newTarget, L(2)), L(1));
S.target(idx) = newTarget;

% one rate-limited step (deg/s -> rad/s)
tn = now; dt = max((tn - S.lastNow(idx))*86400, 1/120); % seconds; >= ~8.3 ms
S.lastNow(idx) = tn;
vmax = deg2rad(S.vmaxDegPS);
dq   = S.target(idx) - S.q(idx);
step = max(min(dq, vmax*dt), -vmax*dt);
S.q(idx) = S.q(idx) + step;

% draw
show(S.robot, S.q, 'Parent', S.ax, 'Frames','off', 'PreservePlot',false);
recolorAuboPatches(S.ax);
drawnow limitrate;

% sync widgets to actual
if isvalid(S.knobs(idx)), S.knobs(idx).Value = rad2deg(S.q(idx)); end
if isvalid(S.spin(idx)),  S.spin(idx).Value  = rad2deg(S.q(idx)); end

ctrlFig.UserData = S;

% On release, finish smoothly to the final target
if settleAfter
    settleJoint(ctrlFig, idx);
end
end

function settleJoint(ctrlFig, idx)
S = ctrlFig.UserData;
t0 = tic;
vmax = deg2rad(S.vmaxDegPS);
while toc(t0) < S.settleTime
    dq = S.target(idx) - S.q(idx);
    if abs(dq) < deg2rad(0.2), break; end
    dt = 1/90;                                 % ~90 Hz settle loop
    step = max(min(dq, vmax*dt), -vmax*dt);
    S.q(idx) = S.q(idx) + step;

    show(S.robot, S.q, 'Parent', S.ax, 'Frames','off', 'PreservePlot',false);
    recolorAuboPatches(S.ax);
    drawnow limitrate;

    if isvalid(S.knobs(idx)), S.knobs(idx).Value = rad2deg(S.q(idx)); end
    if isvalid(S.spin(idx)),  S.spin(idx).Value  = rad2deg(S.q(idx)); end
end
ctrlFig.UserData = S;
end

function kHome(ctrlFig, btn)
S = ctrlFig.UserData;

% UI busy state
if nargin>1 && isvalid(btn), btn.Enable = 'off'; btn.Text = 'Homing…'; end
if isvalid(ctrlFig), ctrlFig.Pointer = 'watch'; drawnow; end

% target is nominal home
S.target = reshape(homeConfiguration(S.robot),1,[]);
S = settleAll(ctrlFig, S, true);   % run-to-completion, then snap

% UI restore
if nargin>1 && isvalid(btn), btn.Text = 'Home'; btn.Enable = 'on'; end
if isvalid(ctrlFig), ctrlFig.Pointer = 'arrow'; end

ctrlFig.UserData = S;
syncUI(ctrlFig);
end

function S = settleAll(ctrlFig, S, snapAtEnd)
% Drive S.q -> S.target with speed cap until within tolerance or timeout
vmax = deg2rad(S.vmaxDegPS);        % rad/s
tol  = deg2rad(S.homeTolDeg);       % rad
dt   = 1/120;                       % internal settle step (~120 Hz)
t0   = tic;

while true
    dq = S.target - S.q;
    if all(abs(dq) <= tol), break; end
    if toc(t0) > S.maxHomeTime, break; end

    step = max(min(dq, vmax*dt), -vmax*dt);
    S.q  = S.q + step;

    show(S.robot, S.q, 'Parent', S.ax, 'Frames','off', 'PreservePlot',false);
    recolorAuboPatches(S.ax);
    drawnow limitrate;
end

if snapAtEnd
    S.q = S.target;
    show(S.robot, S.q, 'Parent', S.ax, 'Frames','off', 'PreservePlot',false);
    recolorAuboPatches(S.ax);
    drawnow;
end
end

function kCam(ctrlFig)
S = ctrlFig.UserData;
view(S.ax,135,20);
if isfield(S,'light') && isvalid(S.light)
    camlight(S.light,'headlight');
else
    S.light = camlight(S.ax,'headlight');
end
ctrlFig.UserData = S;

% Keep full workspace visible after camera reset
adjustAxesToWorkspace(ctrlFig);
end

function toggleWorkspace(ctrlFig, btn)
S = ctrlFig.UserData;
if isempty(S.wsCenter), S.wsCenter = estimateBaseCenter(S.robot) + [0 0 S.wsZBump]; end
if ~isgraphics(S.wsHandle)
    S.wsHandle = drawWorkspaceSphere(S.ax, S.wsRadius, S.wsCenter);
    S.wsVisible = true; btn.Text = 'Hide Workspace';
else
    if strcmp(S.wsHandle.Visible,'on')
        S.wsHandle.Visible = 'off'; S.wsVisible = false; btn.Text = 'Show Workspace';
    else
        S.wsHandle.Visible = 'on';  S.wsVisible = true;  btn.Text = 'Hide Workspace';
    end
end
ctrlFig.UserData = S;

% Make sure limits cover the sphere on first show
adjustAxesToWorkspace(ctrlFig);
end

function adjustAxesToWorkspace(ctrlFig)
% Fix axes so the *entire* reachable zone is always visible;
% also draw world X/Y/Z axes starting at the origin.
S = ctrlFig.UserData;
if isempty(S.wsCenter), S.wsCenter = estimateBaseCenter(S.robot) + [0 0 S.wsZBump]; end
C = S.wsCenter; R = S.wsRadius; p = S.pad;

xlim(S.ax,[C(1)-R-p, C(1)+R+p]);
ylim(S.ax,[C(2)-R-p, C(2)+R+p]);
zmin = min(0, C(3)-R-p);                % include any region below base if needed
zmax = C(3)+R+p;
zlim(S.ax,[zmin, zmax]);

% draw/update world axes from origin
L = R + p;
S.worldAxes = drawWorldAxes(S.ax, S.worldAxes, L);
ctrlFig.UserData = S;
end

function A = drawWorldAxes(ax, A, L)
% Draw (or update) full-length world X/Y/Z axes from origin to +L
if ~isgraphics(A.x) || ~ishandle(A.x)
    A.x = plot3(ax,[0 L],[0 0],[0 0],'r-','LineWidth',1.2,'Tag','worldX');
else
    set(A.x,'XData',[0 L],'YData',[0 0],'ZData',[0 0]);
end
if ~isgraphics(A.y) || ~ishandle(A.y)
    A.y = plot3(ax,[0 0],[0 L],[0 0],'g-','LineWidth',1.2,'Tag','worldY');
else
    set(A.y,'XData',[0 0],'YData',[0 L],'ZData',[0 0]);
end
if ~isgraphics(A.z) || ~ishandle(A.z)
    A.z = plot3(ax,[0 0],[0 0],[0 L],'b-','LineWidth',1.2,'Tag','worldZ');
else
    set(A.z,'XData',[0 0],'YData',[0 0],'ZData',[0 L]);
end
uistack([A.x A.y A.z],'bottom');   % keep under robot graphics
end

function syncUI(ctrlFig)
S = ctrlFig.UserData;
for ii = 1:numel(S.q)
    if isvalid(S.knobs(ii)), S.knobs(ii).Value = rad2deg(S.q(ii)); end
    if isvalid(S.spin(ii)),  S.spin(ii).Value  = rad2deg(S.q(ii)); end
end
ctrlFig.UserData = S;
end

function closeBoth(ctrlFig, visFig)
if isvalid(visFig),  delete(visFig);  end
if isvalid(ctrlFig), delete(ctrlFig); end
end

function j = findJointObject(robot, jointName)
jn = char(jointName);
for ii = 1:robot.NumBodies
    if strcmp(robot.Bodies{ii}.Joint.Name, jn)
        j = robot.Bodies{ii}.Joint; return;
    end
end
error('Joint "%s" not found.', jn);
end

function recolorAuboPatches(ax)
orange = [0.96 0.45 0.00];
black  = [0.10 0.10 0.10];
blackNames = lower(["link0","base","flange","link6","tool","ee","tcp","wrist3_link","wrist3"]);
patches = findobj(ax,'Type','Patch');
for h = reshape(patches,1,[])
    if ~ishandle(h), continue; end
    name = "";
    try name = string(get(h,'DisplayName')); catch, end
    if strlength(strtrim(name))==0, try name = string(get(h,'Tag')); catch, end, end
    if strlength(strtrim(name))==0
        try ud = get(h,'UserData');
            if isstruct(ud) && isfield(ud,'BodyName'), name = string(ud.BodyName); end
        catch, end
    end
    lname = lower(strtrim(name));
    useBlack = any(contains(lname, blackNames));
    set(h,'FaceColor', ternary(useBlack, black, orange), 'EdgeColor','none','FaceAlpha',1);
end
end

function out = ternary(cond, a, b), if cond, out = a; else, out = b; end, end

%% ===== Workspace helpers ==================================================
function c = estimateBaseCenter(robot)
% Center at first actuated joint using the parent-side transform.
idx = [];
for ii = 1:robot.NumBodies
    if robot.Bodies{ii}.Joint.Type ~= "fixed"
        idx = ii; break;
    end
end
if isempty(idx), c = [0 0 0]; return, end
j   = robot.Bodies{idx}.Joint;
P2J = invSE3(j.JointToParentTransform);  % parent->joint
c   = P2J(1:3,4).';
end

function Tinv = invSE3(T)
R = T(1:3,1:3); p = T(1:3,4);
Tinv = eye(4); Tinv(1:3,1:3) = R.'; Tinv(1:3,4) = -R.'*p;
end

function h = drawWorkspaceSphere(ax, R, C)
[X,Y,Z] = sphere(48);
h = surf(ax, C(1)+R*X, C(2)+R*Y, C(3)+R*Z, ...
    'FaceColor','none', 'EdgeColor',[0.75 0.75 0.75], 'EdgeAlpha',0.35, 'Tag','workspace');
uistack(h,'bottom');
end

