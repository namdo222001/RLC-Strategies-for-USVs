clear; clc; close all;

%% Tham số mô phỏng
Ts = 0.01; 
Tsim = 70; 
N = 20;
sim_time = 0:Ts:Tsim;
K = numel(sim_time);
% vi_func = @(t) (t<15)*5 + (t>=15)*10;
% wi_func = @(t) (t<15)*0.1 + (t>=15)*(-0.1);
% vi_func=5;
% wi_func=0;
vi_func = @(t) (t<15)*5 + (t>=15)*10;
wi_func=0;
psid = 4*pi/3; 
d = 1;

% Khoảng cách desired:
%Ld = [1.5, 6.0, 10.0, 14.0];
%% Circle
%Ld = [6.0, 12.0, 18.0, 24.0];
%% Straight
Ld = [20.0, 25.0, 28.0, 31.0];
k2 = 0.35;

a_input = 20; lambda_tube = 0.41;

%% Khởi tạo vị trí
X = zeros(3, 1+4, K);  % X(:,:,1)=leader, then followers 1–4
X(:,1,1) = [0;0;pi/2];
X(:,2,1) = [10;-10;pi/2];
X(:,3,1) = [20;-20;pi/2];
X(:,4,1) = [30;-30;pi/2];
X(:,5,1) = [40;-40;pi/2];

ctrl = cell(1,4);

err = zeros(3,4,K);

%%  Thiết lập MPC cho từng follower
import yalmip.*; yalmip('clear');
Q = diag([2,2,1]); R = diag([0.05,0.05]);
for j = 1:4
    ctrl{j} = create_mpc_controller(N, Ts, Q, R, d, k2, Ld(j), a_input, lambda_tube);
end

%% Mô phỏng
for i = 1:K-1
    t = sim_time(i);
    fprintf('Sim time: %.1f s\n', sim_time(i));  % ⬅ Thêm dòng này
    % vi = vi_func(t); wi = wi_func(t);
    vi = vi_func(t); wi = wi_func;
    % Cập nhật leader
    pl = X(:,1,i);
    X(:,1,i+1) = pl + Ts * [vi*cos(pl(3)); vi*sin(pl(3)); wi];

    % Mỗi follower chạy theo robot trước nó
    for j = 1:4
        pk = X(:,j+1,i);
        pt = X(:,j,i);
        [pk_new, uk, ek] = stepFollower(pk, pt, vi, wi, ctrl{j}, psid, k2, d, Ld(j), Ts);
        X(:,j+1,i+1) = pk_new;
        err(:,j,i+1) = ek;
        ctrl_mat(j,:,i+1) = uk;
    end
end

%% Assign ra workspace
assignin('base','X_all',X);
assignin('base','err_all',err);
assignin('base','ctrl_all',ctrl_mat);

%% Vẽ quỹ đạo và robot
% figure('Position',[100 100 900 700]); hold on; axis equal; grid on;
% colors = {'b','r','g','m','c'};
% labels = {'Leader','F1','F2','F3','F4'};
% for j = 1:5
%     plot(squeeze(X(1,j,:)), squeeze(X(2,j,:)), 'Color',colors{j}, 'LineWidth',1.5);
%     draw_robot_rect(X(:,j,end), 3, 1.2, colors{j});
% end
% legend(labels,'Location','best');
% title('Trajectory: Leader → F1 → F2 → F3 → F4');
plot_simulation_results_multi(sim_time, X_all, ctrl_all, Ld, d, psid, a_input, lambda_tube);

%% Tín hiệu điều khiển
% figure;
% for j = 1:4
%     subplot(4,1,j);
%     plot(sim_time, squeeze(ctrl_mat(j,1,:)), sim_time, squeeze(ctrl_mat(j,2,:)));
%     legends = {'v','w'};
%     legend(legends); title(sprintf('Follower %d Control',j)); grid on;
% end

%% Lỗi theo dõi
% figure;
% for j = 1:4
%     subplot(4,1,j);
%     plot(sim_time, squeeze(err(1,j,:)), sim_time, squeeze(err(2,j,:)), sim_time, squeeze(err(3,j,:)));
%     legend('x_error','y_error','theta_error');
%     title(sprintf('Follower %d Tracking Errors',j)); grid on; xlabel('Time (s)');
% end

%% Hàm tạo MPC
% function ctrl = create_mpc_controller(N, Ts, Q, R, d, k2, Ld, a_input, lambda)
%     x = sdpvar(3,N+1); u = sdpvar(2,N);
%     r = sdpvar(3,1); ct = sdpvar(1); st = sdpvar(1);
%     k1 = 0.25; k3 = 0.4;
%     con = [x(:,1)==r]; obj = 0;
%     for k = 1:N
%         dx = [x(3,k)*x(2,k) - u(1,k) + 5*ct - Ld*0.1*st;
%               -x(3,k)*x(1,k) - d*u(2,k) + 5*st + Ld*0.1*ct;
%               (5*st + Ld*0.1*ct + 2*k2*x(2,k))/d - u(2,k)];
%         con = [con, x(:,k+1)==x(:,k)+Ts*dx, norm(u(:,k)/a_input,1)<=lambda];
%         obj = obj + x(:,k)'*Q*x(:,k) + u(:,k)'*R*u(:,k);
%     end
%     obj = obj + 10*x(1,N+1)^2 + 10*x(2,N+1)^2 + 5*x(3,N+1)^2;
%     con = [con, -0.1 <= k1*x(1,N+1)+k2*x(2,N+1)+k3*x(3,N+1) <= 0.1];
%     ctrl = optimizer(con, obj, sdpsettings('solver','quadprog','verbose',0), {r,ct,st}, u(:,1));
% end
%% Hàm MPC với safety constraint (mặc định safety luôn đúng)
function ctrl = create_mpc_controller(N, Ts, Q, R, d, k2, Ld, a_input, lambda)
    x = sdpvar(3,N+1); u = sdpvar(2,N);
    r = sdpvar(3,1); ct = sdpvar(1); st = sdpvar(1);
    enable_safety = false;
    po = sdpvar(2,1);
    k1 = 0.25; k3 = 0.4;
    con = [x(:,1)==r]; obj = 0;eta=0;
    for k = 1:N
        dx = [x(3,k)*x(2,k)-u(1,k)+5*ct-Ld*0.1*st;
              -x(3,k)*x(1,k)-d*u(2,k)+5*st+Ld*0.1*ct;
              (5*st+Ld*0.1*ct+2*k2*x(2,k))/d - u(2,k)];
        con = [con, x(:,k+1)==x(:,k)+Ts*dx];
        con = [con, norm(u(:,k)/a_input,1)<=lambda];
        if enable_safety
            con = [con, (x(1,k)-po(1))^2 + (x(2,k)-po(2))^2 >= 1^2];
        else
            con = [con, eta>=0];
        end
        obj = obj + x(:,k)'*Q*x(:,k) + u(:,k)'*R*u(:,k);
    end
    obj = obj + 10*x(1,N+1)^2 + 10*x(2,N+1)^2 + 5*x(3,N+1)^2;
    con = [con, -0.1 <= k1*x(1,N+1)+k2*x(2,N+1)+k3*x(3,N+1) <= 0.1];
    if enable_safety
        ctrl = optimizer(con,obj,sdpsettings('solver','fmincon'),{r,ct,st,po},u(:,1));
    else
        ctrl = optimizer(con,obj,sdpsettings('solver','quadprog'),{r,ct,st},u(:,1));
    end
end

%% Step follower chung
function [pk_new, uk, ek] = stepFollower(pk, pt, vi, wi, ctrl, psid, k2, d, Ld, Ts)
    pe = pt - pk;
    Rk = [cos(pk(3)), sin(pk(3)), 0; -sin(pk(3)), cos(pk(3)), 0; 0,0,1];
    ek = Rk * pe;
    ct = cos(pt(3)-pk(3)); st = sin(pt(3)-pk(3));
    try
        uopt = ctrl{{ek, ct, st}};
        vk = vi*ct - Ld*wi*sin(psid + (pt(3)-pk(3))) - uopt(1);
        wk = (vi*st + Ld*wi*cos(psid + (pt(3)-pk(3))) + 2*k2*ek(2))/d - uopt(2);
    catch
        vk = 0; wk = 0;
    end
    uk = [vk;wk];
    pk_new = pk + Ts * [vk*cos(pk(3)); vk*sin(pk(3)); wk];
end

%% Vẽ robot chữ nhật
function draw_robot_rect(pos, L, W, color)
    x = pos(1); y = pos(2); theta = pos(3);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    corners = 0.5*[-L -W; -L W; L W; L -W]';
    rot = R * corners;
    patch(x + rot(1,:), y + rot(2,:), color, 'FaceAlpha', 0.3, 'EdgeColor', 'k');
end
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

for i = 1:2000:num_steps
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
figure;
for j = 1:num_robots-1
    subplot(num_robots-1,1,j);
    plot(sim_time, squeeze(ctrl_all(j,1,:)), sim_time, squeeze(ctrl_all(j,2,:)), 'LineWidth',1.2);
    legend('v','\omega'); title(sprintf('Tín hiệu điều khiển - Follower %d', j)); grid on;
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
