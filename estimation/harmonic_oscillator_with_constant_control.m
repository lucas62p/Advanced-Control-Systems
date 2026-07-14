%% Kalman Filter for Harmonic Oscillator
% Description: Estimates position and velocity of an oscillator 
% under constant excitation and stochastic process noise.
% Reference: Exercices d'Estimation et Filtrage Optimal - TPS

clear all; close all; clc;

%% 1. System Definition
w = 5; ksi = 0.2; Ts = 0.01; u = 1; 
T_sim = 1; temps = 0:Ts:T_sim; N = length(temps);

% Continuous State-Space
A = [0 1; -w^2 -2*ksi*w];
B = [0; 12];
C = [1 0]; D = 0;
G = [0; 1]; % Perturbation input matrix

% Discretization (Zero-Order-Hold)
sysd = c2d(ss(A, [B, G], C, D), Ts, 'zoh');
Ad = sysd.A; Bd = sysd.B(:,1); Gd = sysd.B(:,2); Cd = sysd.C;

% Noise Covariances
Qd = 4.47; R = 0.01;
std_w = sqrt(Qd); std_v = sqrt(R);

%% 2. Stochastic Simulation (Ground Truth)
X_true = zeros(2, N); Y = zeros(1, N);
for k = 2:N
    w_k = std_w * randn(); v_k = std_v * randn();
    X_true(:,k) = Ad * X_true(:, k-1) + Bd * u + Gd * w_k;
    Y(k) = Cd * X_true(:, k) + v_k;
end

%% 3. Kalman Filter Implementation
X_hat = zeros(2, N); P_est = zeros(2, 2, N);
P_est(:,:,1) = 2 * eye(2); % Initial uncertainty

for k = 2:N
    % Prediction step
    X_pred = Ad * X_hat(:, k-1) + Bd * u;
    P_pred = Ad * P_est(:, :, k-1) * Ad' + Gd * Qd * Gd';
    
    % Correction step (Innovation)
    Ke = P_pred * Cd' / (Cd * P_pred * Cd' + R);
    X_hat(:, k) = X_pred + Ke * (Y(k) - Cd * X_pred);
    P_est(:, :, k) = (eye(2) - Ke * Cd) * P_pred;
end

%% 4. Professional Visualization
figure('Color', 'w');
subplot(2, 1, 1);
plot(temps, X_true(1,:), 'b', temps, X_hat(1,:), 'g--', 'LineWidth', 1.5);
title('Position Estimation'); xlabel('Time (s)'); ylabel('x_1 (m)');
legend('True', 'Estimated'); grid on;

subplot(2, 1, 2);
plot(temps, X_true(2,:), 'b', temps, X_hat(2,:), 'g--', 'LineWidth', 1.5);
title('Velocity Estimation'); xlabel('Time (s)'); ylabel('x_2 (m/s)');
legend('True', 'Estimated'); grid on;
