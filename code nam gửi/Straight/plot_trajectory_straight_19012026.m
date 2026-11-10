load('data.mat');
size(X_all)

%% --- IEEE COLOR PALETTE ---
ieee_colors = [
    0,    0.4470, 0.7410;   % Blue
    0.8500, 0.3250, 0.0980; % Red-Orange
    0.9290, 0.6940, 0.1250; % Yellow
    0.4940, 0.1840, 0.5560; % Purple
];

leader_color = [0 0 0];  % black
num_followers = num_robots - 1;

%% --- Main figure ---
figure;
set(gcf,'units','centimeters','position',[0 0 22 18]); 
hold on; grid on;

%% --- Plot trajectories (NO MARKERS) ---
for j = 1:num_robots

    if j == 1
        c  = leader_color;
        ls = '-';
        lw = 1.8;
    else
        c  = ieee_colors(j-1,:);
        ls = '--';
        lw = 1.5;
    end

    plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
        'Color', c, ...
        'LineWidth', lw, ...
        'LineStyle', ls);
end

%% --- Draw robots on main plot ---
for i = 1:2000:num_steps
    for j = 1:num_robots
        if j == 1
            c = leader_color;
        else
            c = ieee_colors(j-1,:);
        end
        draw_arrow_and_box(X_all(:,j,i), 6, 3, c);
    end
end

legend({'Leader','Follower 1','Follower 2','Follower 3','Follower 4'}, ...
       'Interpreter','latex','Location','northeast');

xlabel('$x$ (m)','Interpreter','latex');
ylabel('$y$ (m)','Interpreter','latex');
title('Leader--Follower Trajectory','Interpreter','latex');
axis equal;

%% ===================== TIME ANNOTATIONS (CUSTOMIZABLE) =====================
% Cấu hình từng mũi tên thời gian
time_cfg = [
    struct('t',0.0,  'dx',-0.08,'dy',-0.04,'ls','--','lw',1.2)
    struct('t',5.4,  'dx',-0.08,'dy',-0.04,'ls','--','lw',1.2)
    struct('t',11.4, 'dx',-0.08,'dy',-0.04,'ls','--','lw',1.2)
    struct('t',17.4, 'dx',-0.08,'dy',-0.04,'ls','--','lw',1.2)
    struct('t',23.4, 'dx',-0.08,'dy',-0.04,'ls','--','lw',1.2)
    struct('t',56.4, 'dx',-0.08,'dy',-0.04,'ls','--','lw',1.2)
];

ax = gca;
xL = ax.XLim;
yL = ax.YLim;

for k = 1:length(time_cfg)

    % Index gần nhất với thời gian mong muốn
    [~, idx] = min(abs(sim_time - time_cfg(k).t));

    % Vị trí leader
    x = X_all(1,1,idx);
    y = X_all(2,1,idx);

    % Data -> figure normalized
    xn = (x - xL(1)) / (xL(2) - xL(1));
    yn = (y - yL(1)) / (yL(2) - yL(1));

    % Vị trí text
    x_text = xn + time_cfg(k).dx;
    y_text = yn + time_cfg(k).dy;

    annotation('textarrow', ...
        [x_text xn], ...
        [y_text yn], ...
        'String', sprintf('$t = %.1f\\,\\mathrm{s}$', time_cfg(k).t), ...
        'Interpreter','latex', ...
        'FontSize',10, ...
        'LineStyle',time_cfg(k).ls, ...
        'LineWidth',time_cfg(k).lw, ...
        'Color','k', ...
        'HeadLength',7, ...
        'HeadWidth',7);
end


%% ===================== ZOOM REGIONS =====================
x_zoom  = [-5 10];   y_zoom  = [270 330];
x_zoom2 = [-5 45];   y_zoom2 = [-50 10];

rectangle('Position',[x_zoom(1), y_zoom(1), diff(x_zoom), diff(y_zoom)], ...
    'LineStyle','--','LineWidth',1.4,'EdgeColor','k');

rectangle('Position',[x_zoom2(1), y_zoom2(1), diff(x_zoom2), diff(y_zoom2)], ...
    'LineStyle','--','LineWidth',1.4,'EdgeColor','k');

%% ===================== INSET 1 =====================
ax_inset = axes('Position',[0.20 0.35 0.20 0.40]);
box on; hold on; grid on;

for j = 1:num_robots
    if j == 1
        c = leader_color; ls='-'; lw=1.4;
    else
        c = ieee_colors(j-1,:); ls='--'; lw=1.3;
    end

    plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
        'Color', c, 'Parent', ax_inset, ...
        'LineWidth', lw, 'LineStyle', ls);
end

for i = 1:2000:num_steps
    for j = 1:num_robots
        if j == 1
            c = leader_color;
        else
            c = ieee_colors(j-1,:);
        end
        draw_arrow_and_box(X_all(:,j,i), 4, 2, c, ax_inset);
    end
end

xlim(ax_inset, x_zoom);
ylim(ax_inset, y_zoom);

%% ===================== INSET 2 =====================
ax_inset2 = axes('Position',[0.60 0.25 0.25 0.30]);
box on; hold on; grid on;

for j = 1:num_robots
    if j == 1
        c = leader_color; ls='-'; lw=1.4;
    else
        c = ieee_colors(j-1,:); ls='--'; lw=1.3;
    end

    plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
        'Color', c, 'Parent', ax_inset2, ...
        'LineWidth', lw, 'LineStyle', ls);
end

for i = 1:2000:num_steps
    for j = 1:num_robots
        if j == 1
            c = leader_color;
        else
            c = ieee_colors(j-1,:);
        end
        draw_arrow_and_box(X_all(:,j,i), 4, 2, c, ax_inset2);
    end
end

xlim(ax_inset2, x_zoom2);
ylim(ax_inset2, y_zoom2);

%% ===================== ARROWS TO INSETS =====================
annotation('textarrow',[0.48 0.4],[0.52 0.58], ...
    'LineStyle','--','LineWidth',1.2,'Color','k');

annotation('textarrow',[0.52 0.6],[0.18 0.38], ...
    'LineStyle','--','LineWidth',1.2,'Color','k');

%exportgraphics(gcf,'Trajectory_straight_v3.pdf','ContentType','vector','BackgroundColor','none');

%% ===================== FUNCTIONS =====================
function draw_arrow_and_box(pos, L, W, color, ax)
    if nargin < 5, ax = gca; end
    x = pos(1); y = pos(2); theta = pos(3);

    quiver(x, y, 5*cos(theta), 5*sin(theta), 0, ...
        'Color', color, 'LineWidth',1.2, ...
        'MaxHeadSize',0.5, 'Parent', ax);

    plot(x, y, '.', 'MarkerSize', 15, ...
        'Color', color, 'Parent', ax);

    draw_rectangle([x; y], L, W, theta, color, ax);
end

function draw_rectangle(center, length, width, angle, color, ax)
    if nargin < 6, ax = gca; end

    p = [-length/2, -width/2;
          length/2, -width/2;
          length/2,  width/2;
         -length/2,  width/2]';

    R = [cos(angle), -sin(angle);
         sin(angle),  cos(angle)];

    p_rot = R * p + center(1:2);

    fill(p_rot(1,:), p_rot(2,:), color, ...
        'FaceAlpha', 0.3, ...
        'EdgeColor', color, ...
        'LineWidth', 1.4, ...
        'Parent', ax);
end
