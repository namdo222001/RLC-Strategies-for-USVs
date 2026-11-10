%%  Vẽ quỹ đạo + hình robot
figure;
set(gcf,'units','centimeters','position',[0 0 22 18]); 
hold on; grid on;

for j = 1:num_robots
    plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
         'Color', colors{j}, 'LineWidth', 1.5);
end

% Vẽ robot trên hình chính
for i = 1:2000:num_steps
    for j = 1:num_robots
        draw_arrow_and_box(X_all(:,j,i), 6, 3, colors{j}, gca);
    end
end

legend(labels,'Interpreter','latex','Location','best');
xlabel('$x (m)$','Interpreter','latex');
ylabel('$y (m)$','Interpreter','latex');
title('Leader - Follower Trajectory','Interpreter','latex');
axis equal;

%% === Zoom inset ===
x_zoom = [20 40];
y_zoom = [10 30];

ax_inset = axes('Position',[0.55 0.55 0.3 0.3]); 
box on; hold on; grid on;

% Vẽ lại quỹ đạo trong inset
for j = 1:num_robots
    plot(squeeze(X_all(1,j,:)), squeeze(X_all(2,j,:)), ...
         'Color', colors{j}, 'LineWidth', 1.2, 'Parent', ax_inset);
end

% Vẽ lại robot trong inset
for i = 1:2000:num_steps
    for j = 1:num_robots
        xj = X_all(1,j,i);
        yj = X_all(2,j,i);
        if xj >= x_zoom(1) && xj <= x_zoom(2) && ...
           yj >= y_zoom(1) && yj <= y_zoom(2)
            draw_arrow_and_box(X_all(:,j,i), 6, 3, colors{j}, ax_inset);
        end
    end
end

xlim(ax_inset, x_zoom);
ylim(ax_inset, y_zoom);
title(ax_inset,'Zoom','FontSize',8);

%% === Khung zoom trên hình chính ===
rectangle('Position',[x_zoom(1), y_zoom(1), ...
           diff(x_zoom), diff(y_zoom)], ...
           'EdgeColor','k','LineStyle','--','LineWidth',1.2);

%% --- HÀM VẼ ROBOT ---
function draw_arrow_and_box(pos, L, W, color, ax)
    if nargin < 5, ax = gca; end
    x = pos(1); y = pos(2); theta = pos(3);
    quiver(x, y, 5*cos(theta), 5*sin(theta), 0, ...
        'Color', color, 'LineWidth',1.2, 'MaxHeadSize',0.5, 'Parent', ax);
    plot(x, y, '.', 'MarkerSize', 15, 'Color', color, 'Parent', ax);
    draw_rectangle([x; y], L, W, theta, color, ax);
end

function draw_rectangle(center, length, width, angle, color, ax)
    if nargin < 6, ax = gca; end
    p = [-length/2, -width/2;
          length/2, -width/2;
          length/2,  width/2;
         -length/2,  width/2]';
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    p_rotated = R * p + center(1:2);
    fill(p_rotated(1,:), p_rotated(2,:), color, ...
        'FaceAlpha', 0.3, 'EdgeColor', color, 'LineWidth', 1.5, 'Parent', ax);
end
