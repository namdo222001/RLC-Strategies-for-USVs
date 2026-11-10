% clc; 

% Load data
%load("L_values_circle_v2.mat")
load("L_values_nam_circle.mat")
% Number of samples
N = numel(L_values_nam);

% Time
dt = 70/709;
t  = 70;

% Desired signal
Lsid = (10) * ones(N,1);
% Error (1D signal)
e_l = abs(L_values_nam - Lsid)';
% Performance indices
IAE2  = trapz(t, e_l);
ITAE2 = trapz(t, t .* e_l);
RMSE2 = sqrt(mean((L_values_nam - Lsid).^2));

% Display
IAE2
ITAE2
RMSE2
