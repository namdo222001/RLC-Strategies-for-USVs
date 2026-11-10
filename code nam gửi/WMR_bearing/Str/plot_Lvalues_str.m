clc; clear;

% Load data
load("L_values_circle_sinh.mat")
load("L_values_str_nam.mat")

N = 700;
L_circle = L_values_circle(1:N);
L_nam    = L_values_circle_new(1:N);
Lsid     = 10 * ones(N,1);
t        = (1:N)';
%% ===== MARKER SETUP =====
idx_mark = 1:25:N;          % khoảng cách marker
markers  = {'v','>'};      % marker cho 2 đường L
hLegend  = gobjects(3,1);   % handle legend
%% ===== AXES CHÍNH =====
%figure('Units','centimeters','Position',[3 3 8.8 6])
figure;
axMain = axes;
hold(axMain,'on'); grid on; box on;

% ===== L_circle (đỏ – liền – có marker) =====
plot(t, L_circle,'r-','LineWidth',1.5)
plot(t(idx_mark), L_circle(idx_mark), ...
    'LineStyle','none','Marker',markers{1}, ...
    'MarkerEdgeColor','r','MarkerFaceColor','r', ...
    'MarkerSize',5)

hLegend(1) = plot(nan,nan,'r-', ...
    'LineWidth',1.5,'Marker',markers{1},'MarkerSize',5);

% ===== L_nam (xanh – liền – có marker) =====
plot(t, L_nam,'b-','LineWidth',1.5)
plot(t(idx_mark), L_nam(idx_mark), ...
    'LineStyle','none','Marker',markers{2}, ...
    'MarkerEdgeColor','b','MarkerFaceColor','b', ...
    'MarkerSize',5)

hLegend(2) = plot(nan,nan,'b-', ...
    'LineWidth',1.5,'Marker',markers{2},'MarkerSize',5);

% ===== Lsid (đen – liền – KHÔNG marker) =====
plot(t, Lsid,'k-','LineWidth',1.5)

hLegend(3) = plot(nan,nan,'k-', ...
    'LineWidth',1.5,'Marker','none');

xlabel('Time (s)', 'Interpreter','latex');
ylabel('Separation (m)', 'Interpreter','latex');
title('Separation', 'Interpreter','latex');
%legend('Interpreter','latex','Location','best');

legend(axMain, hLegend, ...
    {'$P_{\mathrm{Reference[13]}}$', ...
     '$P_{\mathrm{P.Method}}$', ...
     '$P_{lk}^d$'}, ...
    'Interpreter','latex','FontSize',8,'Location','best')

set(gca,'FontName','Times New Roman','FontSize',8,'LineWidth',1.2)

%% ===== THÔNG SỐ ZOOM =====
xZoom = [100 150];
yZoom = [9.8 10.2];

%% ===== AXES INSET =====
axInset = axes('Position',[0.52 0.4 0.32 0.25]); % [x y w h]
hold(axInset,'on'); grid on; box on;

plot(t, L_circle,'r-','LineWidth',1.0)
plot(t(idx_mark), L_circle(idx_mark), ...
    'LineStyle','none','Marker',markers{1}, ...
    'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',4)

plot(t, L_nam,'b-','LineWidth',1.0)
plot(t(idx_mark), L_nam(idx_mark), ...
    'LineStyle','none','Marker',markers{2}, ...
    'MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',4)

plot(t, Lsid,'k-','LineWidth',1.0)   %  KHÔNG marker

xlim(xZoom)
ylim(yZoom)

set(axInset,'FontName','Times New Roman', ...
             'FontSize',7, ...
             'LineWidth',0.6)

%% ===== ĐÁNH DẤU VÙNG ZOOM TRÊN AXES CHÍNH =====
xlim([50 100]);
ylim([9 11]);

annotation('rectangle',[0.15 0.17 0.15 0.1],'Color','k'); % toạ độ tương đối figure
% Mũi tên nối vùng zoom (tọa độ figure-based)
annotation('textarrow',[0.3 0.63],[0.22 0.4], ...
    'String','', ...
    'Interpreter','latex', ...
    'LineStyle','--', ...
    'LineWidth',1.2);
exportgraphics(gcf,'plot_Lvalues_str.pdf','ContentType','vector','BackgroundColor','none');

