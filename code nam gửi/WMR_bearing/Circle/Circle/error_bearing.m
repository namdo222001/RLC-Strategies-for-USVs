% clc; 

% Load data
%load("psi_values.mat")      % psi_values
load("psi_values_nam.mat")
% Number of samples
% N = numel(psi_values);
N = numel(psi_nam_v2);

% Time
dt = 70/709;
t  = 70;

% Desired signal
psid = (4*pi/3) * ones(N,1);

% Error (1D signal)
e = abs(psi_nam_v2 - psid)';

% Performance indices
IAE1  = trapz(t, e);
ITAE1 = trapz(t, t .* e);
% RMSE1 = sqrt(mean((psi_values - psid).^2));
RMSE1 = sqrt(mean((psi_nam_v2 - psid).^2));

% Display
IAE1
ITAE1
RMSE1
