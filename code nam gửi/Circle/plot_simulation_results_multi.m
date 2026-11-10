function plot_simulation_results_multi(sim_time, X_all, ctrl_all, Ld, d, psid, a_input, lambda_tube)
% X_all: [3 x num_robots x time]
% ctrl_all: [num_robots-1 x 2 x time]
% Ld: vector khoảng cách của mỗi follower
num_steps = numel(sim_time);
num_robots = size(X_all, 2);  % 1 leader + n follower
num_followers = num_robots - 1;

colors = {'b','r','g','m','c','y','k'};
labels = ["Leader", arrayfun(@(i) sprintf("Follower %d", i), 1:num_robots-1, 'UniformOutput', false)];

%%  Vẽ quỹ đạo + hình robot
figure;
set(gcf,'units','centimeters','position',[0 0 22 18]); hold on; grid on;
for j = 1:num_robots
    plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), 'Color', colors{j}, 'LineWidth', 1.5);
end

for i = 1:100:num_steps
    for j = 1:num_robots
        draw_arrow_and_box(X_all(:,j,i), 6, 3, colors{j});
    end
end
legend(labels,'Interpreter','latex');
xlabel('$x (m)$','Interpreter','latex');
ylabel('$y (m)$','Interpreter','latex');
title('Leader - Follower Trajectory','Interpreter','latex');
axis equal;

%% Tín hiệu điều khiển
% figure;
% for j = 1:num_robots-1
%     subplot(num_robots-1,1,j);
%     plot(sim_time, squeeze(ctrl_all(j,1,:)), sim_time, squeeze(ctrl_all(j,2,:)), 'LineWidth',1.2);
%     legend('v','\omega'); title(sprintf('Tín hiệu điều khiển - Follower %d', j)); grid on;
% end
% ----- Hình 1: vẽ vận tốc v -----
figure;
for j = 1:num_robots-1
    subplot(num_robots-1,1,j);
    plot(sim_time, squeeze(ctrl_all(j,1,:)), 'LineWidth', 1.2);
    legend('v'); 
    title(sprintf('Linear velocity - Follower %d', j));
    grid on;
end

% ----- Hình 2: vẽ vận tốc góc omega -----
figure;
for j = 1:num_robots-1
    subplot(num_robots-1,1,j);
    plot(sim_time, squeeze(ctrl_all(j,2,:)), 'LineWidth', 1.2);
    legend('\omega'); 
    title(sprintf('Angular velocity - Follower %d', j));
    grid on;
end
 %% Ràng buộc đầu vào cho mỗi follower
figure;
    hold on; grid on;
    for j = 1:num_followers
        constraint_violation = vecnorm([squeeze(ctrl_all(j,1,:))'/a_input; squeeze(ctrl_all(j,2,:))'/a_input],1);
        plot(sim_time, constraint_violation, 'Color', colors{j+1}, 'LineWidth',1.2);
    end
    % yline(lambda_tube, 'k--', 'Limit');
    legend(arrayfun(@(j) sprintf('Follower %d', j), 1:num_followers, 'UniformOutput', false));
    title('Constraint Violation: ||v||/a + ||\omega||/b');
    xlabel('Time (s)'); ylabel('Violation (norm₁)');
%%  Lỗi khoảng cách & định hướng
% figure;
for j = 2:num_robots
    dist_err = zeros(1, num_steps); orient_err = zeros(1, num_steps);
    for i = 1:num_steps
        pd = calc_desired_pose(X_all(:,j-1,i), Ld(j-1), d, psid); % robot j theo robot j-1
        err = X_all(:,j,i) - pd;
        dist_err(i) = norm(err(1:2)) - Ld(j-1);
        orient_err(i) = err(3);
    end
    % subplot(num_robots-1, 2, 2*(j-1)-1);
    % plot(sim_time, dist_err, 'Color', colors{j}); ylabel('Dist Err'); title(sprintf('Follower %d',j-1)); grid on;
    % 
    % subplot(num_robots-1, 2, 2*(j-1));
    % plot(sim_time, orient_err, 'Color', colors{j}); ylabel('Orient Err'); grid on;
end
xlabel('Time (s)');
end

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
