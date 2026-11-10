save('data.mat', 'X_all', 'ctrl_mat', 'ctrl_all', 'sim_time');
load('data.mat');
% size(X_all)
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
% figure;
% for j = 1:num_robots-1
%     subplot(num_robots-1,1,j);
%     plot(sim_time, squeeze(ctrl_all(j,1,:)), 'LineWidth', 1.2);
%     legend('v'); 
%     title(sprintf('Linear velocity - Follower %d', j));
%     grid on;
% end
% 
% % ----- Hình 2: vẽ vận tốc góc omega -----
% figure;
% for j = 1:num_robots-1
%     subplot(num_robots-1,1,j);
%     plot(sim_time, squeeze(ctrl_all(j,2,:)), 'LineWidth', 1.2);
%     legend('\omega'); 
%     title(sprintf('Angular velocity - Follower %d', j));
%     grid on;
% end
% legend(labels,'Interpreter','latex');
% xlabel('$x (m)$','Interpreter','latex');
% ylabel('$y (m)$','Interpreter','latex');
% title('Leader - Follower Trajectory','Interpreter','latex');
% axis equal;
% --- Subplot 1: Linear velocities ---
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

% --- Subplot 2: Angular velocities ---
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
%% Đo góc
function draw_arrow_and_box(pos, L, W, color)
    x = pos(1); y = pos(2); theta = pos(3);
    quiver(x, y, 5*cos(theta), 5*sin(theta), 0, 'Color', color, 'LineWidth',1.2, 'MaxHeadSize',0.5);
    plot(x, y, '.', 'MarkerSize', 15, 'Color', color);
    draw_rectangle([x; y], L, W, theta, color);
end

function pd = calc_desired_pose(p, Ld, d, psid)
    pd_x = p(1) - d*cos(p(3)) - Ld*sin(psid + p(3));
    pd_y = p(2) - d*sin(p(3)) + Ld*cos(psid + p(3));
    pd = [pd_x; pd_y; p(3)];
end

function draw_rectangle(center, length, width, angle, color)
    p = [-length/2, -width/2;
          length/2, -width/2;
          length/2,  width/2;
         -length/2,  width/2]';
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    p_rotated = R * p + center(1:2);
    fill(p_rotated(1,:), p_rotated(2,:), color, ...
        'FaceAlpha', 0.3, 'EdgeColor', color, 'LineWidth', 1.5);
end

