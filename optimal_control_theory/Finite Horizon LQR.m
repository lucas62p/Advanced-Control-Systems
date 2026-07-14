% SOLVE_FINITE_LQR Computes the optimal control law for a discrete system
% over a finite horizon N using backward Riccati recursion.
% Inputs: A, B (System matrices), Q, R (Weights), N (Horizon)

%% 1. Parameters & System Definition
A = [0.9974 0.0539; -0.1078 1.1591];
B = [0.0013; 0.0539];
% function to minimise : J=0.5*sum(0.25x_1^2+0.05*x_2^2+0.05*u^2)
Q = diag([0.25, 0.05]);
R = 0.05;
N = 200;
x_k = [2; 1];
F_130=[-0.5522 -5.9668]; % for k between 0 and 130

%% 2. Riccati Solver (Backward Recursion)
K = zeros(N, 2);
P = zeros(2, 2, N + 1);
P(:, :, N + 1) = zeros(2, 2); % Finite Horizon : P(N)=0

for j = N:-1:1
    inv_term = inv(R + B' * P(:, :, j + 1) * B);
    K(j, :) = -inv_term * B' * P(:, :, j + 1) * A;
    % update of P (Riccati)
    P(:, :, j) = Q + A' * P(:, :, j + 1) * (A + B * K(j, :));
end

fprintf("[%f %f]\n", K(131, 1), K(131, 2)) % comparison of computed gain and gain for k=130

%% 3. Simulation (Forward Loop)
u=zeros(1, N);
x=zeros(2, N + 1);
x(:, 1) = x_k;
for j = 1:N
    u(:, j) = K(j, :) * x(:, j);
    x(:, j + 1) = A * x(:, j) + B * u(:, j);
end

%% 4. Professional Plotting (Expert)
% Ajoutons une grille et une meilleure lisibilité
figure('Color', 'w');
plot(0:N, x(1, :), 'LineWidth', 1.5); hold on;
plot(0:N, x(2, :), 'LineWidth', 1.5);
plot(0:N-1, u, 'k--', 'LineWidth', 1.2);
grid on; legend('x_1', 'x_2', 'u');
title(['Optimal Control Trajectory (N=' num2str(N) ')']);
ylabel('Amplitude'); xlabel('Time steps');
% saveas(gcf, 'optimal_trajectory.png');
