%% Kalman Filter for 2D Particle Trajectory with Boundary Conditions
% Description: Simulates a particle in a 2D space [ax, bx] x [ay, by]
% and estimates its state using a Kalman Filter with wall-rebound logic.
% Reference: Estimation et Filtrage Optimal - Télécom Physique Strasbourg

clear all; close all; clc;

%% 1. Simulation Parameters
ax = 0; bx = 9; ay = 0; by = 5; % Field limits
dt = 0.05; N = 500; time = (0:N-1)*dt;
sigma = 0.2; % Measurement noise std

% Dynamic Matrix (Constant velocity model)
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
C = [1 0 0 0; 0 1 0 0]; % We measure only position [x, y]
R = (sigma^2) * eye(2);
Q = diag([0, 0, (0.01*dt)^2, (0.01*dt)^2]); % Process noise (velocity uncertainty)

%% 2. Simulation Loop (Ground Truth Generation)
X_true = zeros(4, N);
Z_mesure = zeros(2, N);
X_true(:, 1) = [4; 2; 1.2; 2.7]; % [x; y; vx; vy]

for k = 2:N
    % State update with reflection logic
    x_new = X_true(1,k-1) + X_true(3,k-1)*dt;
    y_new = X_true(2,k-1) + X_true(4,k-1)*dt;
    
    % Rebound logic
    vx = X_true(3,k-1); vy = X_true(4,k-1);
    if x_new < ax || x_new > bx, vx = -vx; x_new = max(ax, min(x_new, bx)); end
    if y_new < ay || y_new > by, vy = -vy; y_new = max(ay, min(y_new, by)); end
    
    X_true(:,k) = [x_new; y_new; vx; vy];
    Z_mesure(:, k) = C * X_true(:, k) + sigma * randn(2, 1);
end

%% 3. Kalman Filter Implementation
X_hat = zeros(4, N);
P_est = zeros(4, 4, N);
X_hat(:, 1) = [Z_mesure(:, 1); 0; 0]; % Initial guess
P_est(:, :, 1) = diag([sigma^2, sigma^2, 1, 1]);

for k = 2:N
    % Prediction
    X_pred = A * X_hat(:, k-1);
    P_pred = A * P_est(:, :, k-1) * A' + Q;
    
    % Reflection check for state prediction
    if X_pred(1) <= ax || X_pred(1) >= bx, X_pred(3) = -X_pred(3); end
    if X_pred(2) <= ay || X_pred(2) >= by, X_pred(4) = -X_pred(4); end
    
    % Correction (Update)
    Ke = P_pred * C' / (C * P_pred * C' + R);
    X_hat(:, k) = X_pred + Ke * (Z_mesure(:, k) - C * X_pred);
    P_est(:, :, k) = (eye(4) - Ke * C) * P_pred;
end

%% 4. Professional Visualization
figure('Color', 'w');
plot(X_true(1,:), X_true(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(X_hat(1,:), X_hat(2,:), 'g--', 'LineWidth', 1.5);
rectangle('Position', [ax ay bx-ax by-ay], 'EdgeColor', 'k', 'LineWidth', 2);
legend('True Trajectory', 'Kalman Estimate');
title('2D Trajectory Estimation with Kalman Filter');
xlabel('X (m)'); ylabel('Y (m)'); grid on; axis equal;
