clc; clear;

% Load data
load("psi_values_circle_sinh.mat")
load("psi_values_str_nam.mat")

N = 700;
% L_circle = L_values_circle(1:N);
% L_nam    = L_values_nam(1:N);
% Lsid     = 10 * ones(N,1);
psi1 = psi_values_circle(1:N);
psi_nam = psi_values_circle_new(1:N);
% Giá trị mong muốn
psid = (4*pi/3) * ones(N,1);
t        = (1:N)';
%% ===== MARKER SETUP =====
idx_mark = 1:25:N;          % khoảng cách marker
markers  = {'v','>'};       % marker cho psi1, psi_nam
hLegend  = gobjects(3,1);   % handle legend
%% ===== AXES CHÍNH =====
%figure('Units','centimeters','Position',[3 3 8.8 6])
figure;
axMain = axes;
hold(axMain,'on'); grid on; box on;

% ===== psi1 (đỏ – liền – có marker) =====
plot(t, psi1,'r-','LineWidth',1.5)
plot(t(idx_mark), psi1(idx_mark), ...
    'LineStyle','none','Marker',markers{1}, ...
    'MarkerEdgeColor','r','MarkerFaceColor','r', ...
    'MarkerSize',5)

hLegend(1) = plot(nan,nan,'r-', ...
    'LineWidth',1.5,'Marker',markers{1},'MarkerSize',5);

% ===== psi_nam (xanh – liền – có marker) =====
plot(t, psi_nam,'b-','LineWidth',1.5)
plot(t(idx_mark), psi_nam(idx_mark), ...
    'LineStyle','none','Marker',markers{2}, ...
    'MarkerEdgeColor','b','MarkerFaceColor','b', ...
    'MarkerSize',5)

hLegend(2) = plot(nan,nan,'b-', ...
    'LineWidth',1.5,'Marker',markers{2},'MarkerSize',5);

% ===== psid (đen – liền – KHÔNG marker) =====
plot(t, psid,'k-','LineWidth',1.5)

hLegend(3) = plot(nan,nan,'k-', ...
    'LineWidth',1.5,'Marker','none');

xlabel('Time (s)', 'Interpreter','latex');
ylabel('Bearing (rad)', 'Interpreter','latex');
title('Separation', 'Interpreter','latex');
%legend('Interpreter','latex','Location','best');

legend(axMain, hLegend, ...
    {'${\psi}_{\mathrm{Reference[13]}}$', ...
     '${\psi}_{\mathrm{P.Method}}$', ...
     '${\psi}_{lk}^d$'}, ...
    'Interpreter','latex','FontSize',8,'Location','best')

set(gca,'FontName','Times New Roman','FontSize',8,'LineWidth',1.2)

%% ===== THÔNG SỐ ZOOM =====
xZoom = [120 170];
yZoom = [4.19 4.21];

%% ===== AXES INSET =====
axInset = axes('Position',[0.52 0.4 0.32 0.25]); % [x y w h]
hold(axInset,'on'); grid on; box on;

plot(t, psi1,'r-','LineWidth',1.0)
plot(t(idx_mark), psi1(idx_mark), ...
    'LineStyle','none','Marker',markers{1}, ...
    'MarkerEdgeColor','r','MarkerFaceColor','r', ...
    'MarkerSize',4)

plot(t, psi_nam,'b-','LineWidth',1.0)
plot(t(idx_mark), psi_nam(idx_mark), ...
    'LineStyle','none','Marker',markers{2}, ...
    'MarkerEdgeColor','b','MarkerFaceColor','b', ...
    'MarkerSize',4)

plot(t, psid,'k-','LineWidth',1.0)   

xlim(xZoom)
ylim(yZoom)

set(axInset,'FontName','Times New Roman', ...
             'FontSize',7, ...
             'LineWidth',0.6)

%% ===== ĐÁNH DẤU VÙNG ZOOM TRÊN AXES CHÍNH =====
xlim([120 170]);
ylim([4.18 4.21]);

annotation('rectangle',[0.23 0.15 0.1 0.07],'Color','k'); % toạ độ tương đối figure
% Mũi tên nối vùng zoom (tọa độ figure-based)
% Mũi tên nét đứt nối vùng zoom (figure-based coordinates)
annotation('textarrow',[0.3 0.63],[0.22 0.4], ...
    'String','', ...
    'Interpreter','latex', ...
    'LineStyle','--', ...
    'LineWidth',1.2);
exportgraphics(gcf,'plot_psivalues_str.pdf','ContentType','vector','BackgroundColor','none');

