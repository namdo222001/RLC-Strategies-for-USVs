% clear all; 
% clc;
% close;
%save('data.mat', 'X_all', 'ctrl_mat', 'ctrl_all', 'sim_time', 'a_input');
load('data.mat');
size(X_all)
%%  Vẽ quỹ đạo + hình robot
Ts = 0.01; 
Tsim = 70; 
N = 20;
sim_time = 0:Ts:Tsim;

num_steps = numel(sim_time);
num_robots = size(X_all, 2);  % 1 leader + n follower
num_followers = num_robots - 1;
colors = {'b','r','g','m','c','y','k'};

figure;
set(gcf,'units','centimeters','position',[0 0 22 18]); hold on; grid on;
for j = 1:num_robots
    plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), 'Color', colors{j}, 'LineWidth', 1.5);
end

for i = 1:2000:num_steps
    for j = 1:num_robots
        draw_arrow_and_box(X_all(:,j,i), 6, 3, colors{j});
    end
end

close all; clc;

figure('Position',[100 100 800 600]);

% =============== Subplot 1: Linear velocities ==================
subplot(2,1,1); hold on; grid on;
labels = cell(1,num_robots-1);

for j = 1:num_robots-1
    plot(sim_time, squeeze(ctrl_all(j,1,:)), 'LineWidth', 1.2);
    labels{j} = sprintf('Follower %d', j);
end

legend(labels,'Interpreter','latex');
xlabel('Time (s)','Interpreter','latex');
ylabel('$v$ (m/s)','Interpreter','latex');
title('Linear velocities of Followers','Interpreter','latex');

% Inset cho subplot 1
axInset1 = axes('Position',[0.55 0.65 0.3 0.2]); % [x y w h]
box on; hold on; grid on;
for j = 1:num_robots-1
    plot(sim_time, squeeze(ctrl_all(j,1,:)), 'LineWidth', 1.0);
end
xlim([10 20]);
ylim([4 10]);

annotation('rectangle',[0.25 0.77 0.25 0.15],'Color','k'); % toạ độ tương đối figure
% Mũi tên nối vùng zoom (tọa độ figure-based)
annotation('textarrow',[0.5 0.63],[0.9 0.75],...
    'String','','Interpreter','latex');


% =============== Subplot 2: Angular velocities ==================
subplot(2,1,2); hold on; grid on;
labels = cell(1,num_robots-1);

for j = 1:num_robots-1
    plot(sim_time, squeeze(ctrl_all(j,2,:)), 'LineWidth', 1.2);
    labels{j} = sprintf('Follower %d', j);
end

legend(labels,'Interpreter','latex');
xlabel('Time (s)','Interpreter','latex');
ylabel('$\omega$ (rad/s)','Interpreter','latex');
title('Angular velocities of Followers','Interpreter','latex');

% Inset cho subplot 2
axInset2 = axes('Position',[0.55 0.20 0.3 0.2]); % [x y w h]
box on; hold on; grid on;
for j = 1:num_robots-1
    plot(sim_time, squeeze(ctrl_all(j,2,:)), 'LineWidth', 1.0);
end
xlim([10 20]);
ylim([0 3]);

annotation('rectangle',[0.2 0.12 0.25 0.15],'Color','k'); % toạ độ tương đối figure
% Mũi tên nối vùng zoom (tọa độ figure-based)
annotation('textarrow',[0.45 0.63],[0.27 0.4],...
    'String','','Interpreter','latex');
 %% Ràng buộc đầu vào cho mỗi follower
figure;
hold on; grid on;

for j = 1:num_followers
    constraint_violation = vecnorm([squeeze(ctrl_all(j,1,:))'/a_input; ...
                                    squeeze(ctrl_all(j,2,:))'/a_input],1);
    plot(sim_time, constraint_violation, 'Color', colors{j+1}, 'LineWidth',1.2);
end

% yline(lambda_tube, 'k--', 'Limit');
legend(arrayfun(@(j) sprintf('Follower %d', j), 1:num_followers, 'UniformOutput', false), ...
       'Location','northeast');
title('Constraint Violation: ||v||/a + ||\omega||/b');
xlabel('Time (s)'); ylabel('Violation (norm₁)');

%% === Zoom (Inset Axes) ===
% Tạo axes nhỏ
ax_inset = axes('Position',[0.55 0.25 0.3 0.3]); % [x y width height] (normalized)
box on; hold on; grid on;

% Vẽ lại dữ liệu trong vùng zoom
x_zoom = [10 25];  % khoảng thời gian cần zoom
y_zoom = [0 0.6];  % scale trục y mong muốn

for j = 1:num_followers
    constraint_violation = vecnorm([squeeze(ctrl_all(j,1,:))'/a_input; ...
                                    squeeze(ctrl_all(j,2,:))'/a_input],1);
    plot(sim_time, constraint_violation, 'Color', colors{j+1}, 'LineWidth',1.0);
end

xlim(x_zoom);
ylim(y_zoom);
title('');

% === Vẽ mũi tên nối vùng zoom ===
annotation('rectangle',[0.23 0.45 0.22 0.4],'Color','k'); % vùng trên plot chính
annotation('textarrow',[0.45 0.55],[0.5 0.25],'Color','k');     % nối lên inset
%exportgraphics(gcf,'myfigure.pdf','ContentType','vector','BackgroundColor','none');
%% --- IEEE COLOR PALETTE ---
ieee_colors = [
    0,    0.4470, 0.7410;   % Blue
    0.8500, 0.3250, 0.0980; % Red-Orange
    0.9290, 0.6940, 0.1250; % Yellow
    0.4940, 0.1840, 0.5560; % Purple
];

leader_color = [0 0 0];  % black
num_followers = num_robots - 1;

%% --- Vẽ quỹ đạo + hình robot ---
figure;
set(gcf,'units','centimeters','position',[0 0 22 18]); 
hold on; grid on;

for j = 1:num_robots

    if j == 1
        % Leader: nét liền + đen
        plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
             'Color', leader_color, 'LineWidth', 1.8, 'LineStyle', '-');
    else
        % Followers: nét đứt + màu IEEE
        plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
             'Color', ieee_colors(j-1,:), 'LineWidth', 1.5, 'LineStyle', '--');
    end

end

% Vẽ robot trên hình chính
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

legend({'Leader', 'Follower 1', 'Follower 2', 'Follower 3', 'Follower 4'}, ...
       'Interpreter','latex', 'Location','northeast');

xlabel('$x (m)$','Interpreter','latex');
ylabel('$y (m)$','Interpreter','latex');
title('Leader - Follower Trajectory','Interpreter','latex');
axis equal;

%% === Zoom inset ===
x_zoom = [-5 10];
y_zoom = [70 130];

ax_inset = axes('Position',[0.2 0.35 0.2 0.4]);
box on; hold on; grid on;

% Vẽ lại quỹ đạo trong inset
for j = 1:num_robots
    if j == 1
        plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
             'Color', leader_color, 'Parent', ax_inset, ...
             'LineWidth', 1.4, 'LineStyle', '-');
    else
        plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
             'Color', ieee_colors(j-1,:), 'Parent', ax_inset, ...
             'LineWidth', 1.3, 'LineStyle', '--');
    end
end

% Vẽ robot trong inset
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
title(ax_inset,' ','FontSize',8);

%% === Zoom2 inset ===
x_zoom2 = [-5 45];
y_zoom2 = [-50 10];
ax_inset2 = axes('Position',[0.6 0.25 0.25 0.3]);  % inset thứ 2
box on; hold on; grid on;
for j = 1:num_robots
    if j == 1
        plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
             'Color', leader_color, 'Parent', ax_inset2, ...
             'LineWidth', 1.4, 'LineStyle', '-');
    else
        plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
             'Color', ieee_colors(j-1,:), 'Parent', ax_inset2, ...
             'LineWidth', 1.3, 'LineStyle', '--');
    end
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
title(ax_inset2,' ','FontSize',8);

%exportgraphics(gcf,'Trajectory_straight_v3.pdf','ContentType','vector','BackgroundColor','none');
function draw_arrow_and_box(pos, L, W, color, ax)
    if nargin < 5
        ax = gca; % mặc định vẽ vào axes hiện tại
    end
    x = pos(1); y = pos(2); theta = pos(3);
    quiver(x, y, 5*cos(theta), 5*sin(theta), 0, ...
        'Color', color, 'LineWidth',1.2, 'MaxHeadSize',0.5, 'Parent', ax);
    plot(x, y, '.', 'MarkerSize', 15, 'Color', color, 'Parent', ax);
    draw_rectangle([x; y], L, W, theta, color, ax);
end

function draw_rectangle(center, length, width, angle, color, ax)
    if nargin < 6
        ax = gca;
    end
    p = [-length/2, -width/2;
          length/2, -width/2;
          length/2,  width/2;
         -length/2,  width/2]';
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    p_rotated = R * p + center(1:2);
    fill(p_rotated(1,:), p_rotated(2,:), color, ...
        'FaceAlpha', 0.3, 'EdgeColor', color, 'LineWidth', 1.5, 'Parent', ax);
end
