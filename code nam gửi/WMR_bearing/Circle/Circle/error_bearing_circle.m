clc;
t= 70;
Step = 70 / 709;
e1_mat = psi_value_v2 - psid;
e_norm = sqrt(sum(e1_mat.^2, 1));
dt = Step;
IAE1 = trapz(t, e_norm)
IAE1 = trapz(t, t .* e_norm)
RMSE1 = sqrt(mean(e_norm.^2))

% e2_mat = L_value_v2 - y;
% e_norm2 = sqrt(sum(e2_mat.^2, 1));
% dt = Step;
% IAE2 = trapz(t, e_norm2)
% ITAE2 = trapz(t, t .* e_norm2)
% RMSE2 = sqrt(mean(e_norm2.^2))
