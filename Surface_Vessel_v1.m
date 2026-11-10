clear; close all; clc;
%% Time interval and simulation time
Step = 0.001;T_end = 20;
t = 0:Step:T_end;
%% Variables
eta = cell(1,size(t,2));
eta_d = cell(1,size(t,2));
eta_d_dot = cell(1,size(t,2));
eta_d_dot_dot = cell(1,size(t,2));
eta_dot = cell(1,size(t,2));
upsilon_dot = cell(1,size(t,2));
f = cell(1,size(t,2));
d = cell(1,size(t,2));
tau = cell(1,size(t,2));

e1 = cell(1,size(t,2));
e2 = cell(1,size(t,2));
W_c = cell(1,size(t,2));
W_a = cell(1,size(t,2));
GAMMA = cell(1,size(t,2));
upsilon = cell(1,size(t,2));
temp = cell(1,size(t,2));

% switching input
sigma = cell(1,size(t,2));
u_s = cell(1,size(t,2));
v = cell(1,size(t,2));

%RISE
epsilon = cell(1,size(t,2));
rho = cell(1,size(t,2));

%DO
e2_hat = cell(1,size(t,2));
e_phi_hat = cell(1,size(t,2));
Delta_hat = cell(1,size(t,2));
lambda0 = cell(1,size(t,2));
nu1 = cell(1,size(t,2));
compensator = cell(1,size(t,2));

%% Parameters
n = 3;
N = 12; % number of NN nodes

%% Important parameters
eta_c = 2; eta_c2 = 2;
eta_a1 = 0.0;eta_a2 = 50;
nuy = 0.01;

R = 0.5*eye(n);
Q_11 = [20 2 4;2 20 6;4 6 20];
Q_12 = -[2 1 2;1 1 1;2 1 4];
Q_22 = diag([2,2,2]);
Q = [Q_11 Q_12;Q_12' Q_22];

K = [13.4 -9.7 -4.4;-8.4 18.8 3.6;-4.2 0.9 3.8];

% RISE
k_s = 100;
beta2 = 60;
beta1 = 5;

% DO
c = 0.1;
amp_delta1 = 30;
amp_delta2 = 30;
amp_delta3 = 30;
epsi1 = 10; epsi2 = 20;
M_compensator = 40;
delta = [amp_delta1 0 0;0 amp_delta2 0;0 0 amp_delta3];

% switching input
omega_bar = 1;

%% Initial conditions
eta{1} = [0.5;-5;0.3];
W_c{1} = 4*rand(N,1);
W_a{1} = 4*rand(N,1);
upsilon{1} = [0;0;0];

% eta{1} = 1*rand(3,1);
% W_c{1} = 4*rand(N,1);
% W_a{1} = 4*rand(N,1);
% upsilon{1} = 1*rand(3,1);

% eta{1} = [0.9669;0.4182;0.7174];
% W_c{1} = [0.3124;0.2181;0.1044;0.2337;0.3414;0.2429;0.8206;0.9416;0.9652;0.5765;0.8697;0.9161];
% W_a{1} = [0.6170;0.2612;0.5157;0.1243;0.0248;0.8606;0.0368;0.2575;0.8730;0.9489;0.0194;0.4064];

%RISE
rho{1} = [0;0;0];
GAMMA{1} = diag([300,300,300,300,300,100,20,300,20,10,10,10]);
% GAMMA{1} = 100*eye(N);

%DO
lambda0{1} = [0;0;0];
nu1{1} = [0;0;0];
e2_hat{1} = [0;0;0];
compensator{1} = [0;0;0];
Delta_hat{1} = [0;0;0];
e_phi_hat{1} = [0;0;0];

v{1} = [0;0;0];

temp_to = 10;
to1_stack = [-0.1;0;0.1]*temp_to;
to2_stack = [-0.1;0;0.1]*temp_to;
to3_stack = [-0.1;0;0.1]*temp_to;
to4_stack = [-0.1;0;0.1]*temp_to;
to5_stack = [-0.1;0;0.1]*temp_to;
to6_stack = [-0.1;0;0.1]*temp_to;
 
for i = 1:size(t,2)
    i
    %% Noise for PE condition
    if i<= 30000
        noise = 0*[2*rand(1,1)-1;2*rand(1,1)-1;2*rand(1,1)-1];
    else
        noise = 0;
    end
    
    %% Disturbances
    d{i} = [10.3+50*sin(0.5*t(i))+50*sin(0.1*t(i));-20.9+2*sin(0.5*t(i)-pi/6)+50*sin(0.3*t(i));-30*sin(0.9*t(i)+pi/3)-30*sin(0.1*t(i))];
    
    %% Generate reference trajectory
    eta_dot{i} = J(eta{i})*upsilon{i};
    eta_d{i} = [12*sin(0.2*t(i));-12*cos(0.2*t(i));0.2*t(i)];
    eta_d_dot{i} = [12*0.2*cos(0.2*t(i));12*0.2*sin(0.2*t(i));0.2]; 
    eta_d_dot_dot{i} = [-12*0.2*0.2*sin(0.2*t(i));12*0.2*0.2*cos(0.2*t(i));0];   
    
    %% Error
    e1{i} = eta_d{i} - eta{i};
    e2{i} = eta_d_dot{i} - eta_dot{i} + K*e1{i};
    
    %% Sliding surface and switching control input
    sigma{i} = e2{i} - e2{1} - v{i};
    u_s{i} = -M(eta{i})*J(eta{i})'*(0+0*omega_bar)*sigma{i}./(abs(sigma{i})+c);
    
    %% Compute nonlinear funtion f+d
    f{i} = h(eta{i},upsilon{i},eta_dot{i},e1{i},eta_d_dot{i},eta_d_dot_dot{i}) + d{i};
    
    %% Disturbance observer
    theta = e2{i} - e2_hat{i};
    lambda1 = -epsi1*sqrt(abs(lambda0{i}-theta)).*sign(lambda0{i}-theta) + nu1{i};
    phi = [amp_delta1*e2{i}(1); amp_delta2*e2{i}(2); amp_delta3*e2{i}(3)];
    Delta_hat{i} = e_phi_hat{i} + phi + 1*compensator{i};
    
    %% RISE feedback
    epsilon{i} = (k_s + 1)*e2{i} - (k_s + 1)*e2{1} + rho{i};
    
    %% state space model
    z = [e1{i};e2{i}];
    A_X = [-K*e1{i}+e2{i}; (J_dot(eta{i},eta_dot{i})*J(eta{i})' - J(eta{i})*pinv(M(eta{i}))*C(upsilon{i})*J(eta{i})')*e2{i}];
    B_X = [zeros(n);J(eta{i})*pinv(M(eta{i}))];
      
    %% Data stack
    if mod(t(i),0.02) == 0
        sum = 0;
        for q = 1:length(to1_stack) 
            for a = 1:length(to2_stack)
                for g = 1:length(to3_stack)
                    for b = 1:length(to4_stack)
                        for p = 1:length(to5_stack)
                            for w = 1:length(to6_stack)
                                x_stack = [to1_stack(q);to2_stack(a);to3_stack(g);to4_stack(b);to5_stack(p);to6_stack(w)]; 
                                e_temp = x_stack(1:3);
                                s_temp = x_stack(4:6);
                                eta_temp = eta_d{i} - e_temp;
                                eta_dot_temp = eta_d_dot{i} - s_temp + K*e_temp;
                                upsilon_temp = J(eta_temp)'*eta_dot_temp;
                                A_X_temp = [-K*e_temp+s_temp; (J_dot(eta_temp,eta_dot_temp)*J(eta_temp)' - J(eta_temp)*pinv(M(eta_temp))*C(upsilon_temp)*J(eta_temp)')*s_temp];
                                B_X_temp = [zeros(n); J(eta_temp)*pinv(M(eta_temp))];
                                muy_i = -inv(R)*B_X_temp'*gradSigma(x_stack,eta_temp(3))'*W_a{i};
                                omega_i = gradSigmad(x_stack,eta_temp(3))*eta_d_dot{i} + gradSigma(x_stack,eta_temp(3))*(A_X_temp + B_X_temp*muy_i);
                                rho_i = 1 + nuy*omega_i'*GAMMA{i}*omega_i;
                                delta_hjb_i = W_c{i}'*gradSigmad(x_stack,eta_temp(3))*eta_d_dot{i} + W_c{i}'*gradSigma(x_stack,eta_temp(3))*(A_X_temp + B_X_temp*muy_i)+1/2*muy_i'*R*muy_i + 1/2*x_stack'*Q*x_stack;
                                sum = sum + eta_c2*GAMMA{i}/729*omega_i*delta_hjb_i/rho_i;
                            end
                        end
                    end
                end
            end
        end
    end
    %% compute control input
    u = -inv(R)*B_X'*gradSigma(z,eta{i}(3))'*W_a{i};
    tau = M(eta{i})*J(eta{i})'*Delta_hat{i} - u + u_s{i};
    temp{i} = M(eta{i})*J(eta{i})'*Delta_hat{i};
    
    %% compute omega
    omega = gradSigmad(z,eta{i}(3))*eta_d_dot{i} + gradSigma(z,eta{i}(3))*(A_X + B_X*u);
    
    %% Reset gain to prevent covariance wind-up in critic NN
    if min(eig(GAMMA{i})) <= 0.02
        GAMMA{i} = GAMMA{1};
    end
    
    %% Compute delta HJB
    delta_hjb = W_c{i}'*gradSigmad(z,eta{i}(3))*eta_d_dot{i} + W_c{i}'*gradSigma(z,eta{i}(3))*(A_X + B_X*u)+1/2*u'*R*u + 1/2*z'*Q*z; 
    
    %% update state
    W_c_dot = -eta_c*GAMMA{i}*omega/(1+nuy*omega'*GAMMA{i}*omega)*delta_hjb - sum;
    W_a_dot = -eta_a1/sqrt(1+(omega'*omega))*gradSigma(z,eta{i}(3))*B_X/R*B_X'*gradSigma(z,eta{i}(3))'*(W_a{i} - W_c{i})*delta_hjb - eta_a2*(W_a{i} - W_c{i});
    upsilon_dot{i} = pinv(M(eta{i}))*(tau - d{i} - C(upsilon{i})*upsilon{i});
    
    v_dot = (J_dot(eta{i},eta_dot{i})*J(eta{i})' - J(eta{i})*pinv(M(eta{i}))*C(upsilon{i})*J(eta{i})')*e2_hat{i} - J(eta{i})*pinv(M(eta{i}))*(M(eta{i})*J(eta{i})'*Delta_hat{i} - u);
    
    %DO
    compensator_dot = M_compensator*lambda1./(abs(lambda1)+c);
    e_phi_hat_dot = -delta*( (J_dot(eta{i},eta_dot{i})*J(eta{i})' - J(eta{i})*pinv(M(eta{i}))*C(upsilon{i})*J(eta{i})')*e2_hat{i} - J(eta{i})*pinv(M(eta{i}))*tau + Delta_hat{i});
    e2_hat_dot = (J_dot(eta{i},eta_dot{i})*J(eta{i})' - J(eta{i})*pinv(M(eta{i}))*C(upsilon{i})*J(eta{i})')*e2_hat{i} - J(eta{i})*pinv(M(eta{i}))*tau + Delta_hat{i};
    lambda0_dot = lambda1;
    nu1_dot = -epsi2*sign(nu1{i}-lambda1);
    
    if i == size(t,2)
        break;
    else
        %RISE
        rho{i + 1} = rho{i} + ((k_s + 1)*beta2*e2{i} + beta1*sign(e2{i}))*Step;
        %DO
        e_phi_hat{i+1} = e_phi_hat{i} + Step*e_phi_hat_dot;
        e2_hat{i+1} = e2_hat{i} + Step*e2_hat_dot;
        lambda0{i+1} = lambda0{i} + Step*lambda0_dot;
        nu1{i+1} = nu1{i} + Step*nu1_dot;
        compensator{i+1} = compensator{i} + Step*compensator_dot;
        
        v{i+1} = v{i} + Step*v_dot;
        
        upsilon{i + 1} = upsilon{i} + upsilon_dot{i}*Step;
        eta{i + 1} = eta{i} + eta_dot{i}*Step + noise;
        W_a{i+1} = W_a{i} + Step*W_a_dot;
        W_c{i+1} = W_c{i} + Step*W_c_dot;
        GAMMA{i+1} = GAMMA{i} + Step*(-eta_c*GAMMA{i}*(omega*omega')/(1+nuy*omega'*GAMMA{i}*omega)*GAMMA{i});
    end
end
 
eta = cell2mat(eta);
eta_d = cell2mat(eta_d);
f = cell2mat(f);
figure;
subplot(3,1,1);
plot(t,eta(1,:),t,eta_d(1,:));
title('\eta');
legend('\eta_1','\eta_{1d}');
ylabel('\eta_1');
subplot(3,1,2);
plot(t,eta(2,:),t,eta_d(2,:));
legend('\eta_2','\eta_{2d}');
ylabel('\eta_2');
subplot(3,1,3);
plot(t,eta(3,:),t,eta_d(3,:));
legend('\eta_3','\eta_{3d}');
xlabel('Time(s)');
ylabel('\eta_3');

figure;
plot(eta(1,:),eta(2,:),eta_d(1,:),eta_d(2,:));
legend('Tracking trajectory','Desired trajectory');
xlabel('x[m]');
ylabel('y[m]');
title('Trajectory');

W_a = cell2mat(W_a);
W_c = cell2mat(W_c);
figure;
W_c = W_c*2;
plot(t,W_c);
title('W_c');
xlabel('Time(s)');
ylabel('W_c');
figure;
W_a = W_a*2;
plot(t,W_a);
title('W_a');
xlabel('Time(s)');
ylabel('W_a');

temp = cell2mat(temp);
figure;
subplot(3,1,1);
plot(t,temp(1,:),t,f(1,:));
title('Disturbance observer estimation of h+d');
legend('estimate','real');
ylabel('(h+d)[1]');
ylim([-200,200]);
subplot(3,1,2);
plot(t,temp(2,:),t,f(2,:));
legend('estimate','real');
ylabel('(h+d)[2]');
ylim([-200,200]);
subplot(3,1,3);
plot(t,temp(3,:),t,f(3,:));
legend('estimate','real');
ylabel('(h+d)[3]');
xlabel('Time(s)');
ylim([-200,200]);

function a = J(eta)
     a = [cos(eta(3)) -sin(eta(3)) 0;
          sin(eta(3))  cos(eta(3)) 0;
                    0            0 1];
end

function a = J_dot(eta,eta_dot)
    a = [-sin(eta(3)) -cos(eta(3)) 0;
          cos(eta(3)) -sin(eta(3)) 0;
                    0            0 0]*eta_dot(3);
end

function a = C(v)
    a = [0 0 -19*v(2)-0.72*v(3);0 0 20*v(1);19*v(2)+0.72*v(3) -20*v(1) 0];
end

function a = M(eta)
    m11 = 20;
    m22 = 19;
    m33 = 2.7;
    m23 = 0.72;
    m32 = m23;
    a = [m11    0       0;
         0    m22     m23;
         0    m32     m33]; 
end

function a = h(eta,v, eta_dot, e_1, eta_d_dot, eta_d_dot_dot)
    lambda_1    = [13.4000  -9.7000   -4.4000;
                   -8.4000  18.8000    3.6000;
                   -4.2000   0.9000    3.8000];  
    a = M(eta)*J(eta)'*(J(eta)*pinv(M(eta))*C(v)-J_dot(eta,eta_dot))*J(eta)'*(eta_d_dot + lambda_1*e_1) + M(eta)*J(eta)'*(eta_d_dot_dot + lambda_1*(eta_d_dot-eta_dot));
end

function a = gradSigma(x,eta3)
    a = [2*x(1) 0 0 0 0 0;
         x(2) x(1) 0 0 0 0;
         x(3) 0 x(1) 0 0 0;
         0 2*x(2) 0 0 0 0;
         0 x(3) x(2) 0 0 0;
         0 0 2*x(3) 0 0 0;
         0 0 2*cos(eta3)*sin(eta3)*x(4)^2 (19+cos(eta3)^2)*2*x(4) 0 0;
         0 0 (sin(eta3)^2-cos(eta3)^2)*x(4)*x(5) sin(eta3)*cos(eta3)*x(5) sin(eta3)*cos(eta3)*x(4) 0;
         0 0 0.72*cos(eta3)*x(4)*x(6) -0.72*sin(eta3)*x(6) 0 -0.72*sin(eta3)*x(4);         
         0 0 -2*sin(eta3)*cos(eta3)*x(5)^2 0 (19+sin(eta3)^2)*2*x(5) 0;
         0 0 0.72*sin(eta3)*x(5)*x(6) 0 0.72*cos(eta3)*x(6) 0.72*cos(eta3)*x(5);
         0 0 0 0 0 2.7*2*x(6)
         ];
end

function a = gradSigmad(x,eta3)
    a = [0 0 0;
         0 0 0;
         0 0 0;
         0 0 0;
         0 0 0;
         0 0 0;
         0 0 -2*cos(eta3)*sin(eta3)*x(4)^2;
         0 0 (cos(eta3)^2-sin(eta3)^2)*x(4)*x(5);
         0 0 -0.72*cos(eta3)*x(4)*x(6);
         0 0 2*sin(eta3)*cos(eta3)*x(5)^2;
         0 0 -0.72*sin(eta3)*x(5)*x(6);
         0 0 0];
end
