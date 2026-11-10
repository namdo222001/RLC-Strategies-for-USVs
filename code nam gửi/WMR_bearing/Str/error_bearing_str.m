% clc; 

% Load data
%load("psi_values_circle_sinh.mat")      % psi_values
load("psi_values_str_nam.mat")
% Number of samples
%N = numel(psi_values_circle);
N = numel(psi_values_circle_new);

% Time
dt = 70/709;
t  = 70;

% Desired signal
psid = (4*pi/3) * ones(N,1);

% Error (1D signal)
e = abs(psi_values_circle_new - psid)';

% Performance indices
IAE1  = trapz(t, e);
ITAE1 = trapz(t, t .* e);
% RMSE1 = sqrt(mean((psi_values - psid).^2));
RMSE1 = sqrt(mean((psi_values_circle_new - psid).^2));

% Display
IAE1
ITAE1
RMSE1
