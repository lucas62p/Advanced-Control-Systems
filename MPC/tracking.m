%% MPC Tracking Initialization
% Project: Tracking of Y(s)/U(s) = 1/(s(s+1))
% Reference: Télécom Physique Strasbourg - MPC Practicals

clear all; close all; clc;

%% 1. System Discretization & Augmented Model
G_s = tf(1, [1 1 0]);
Ts = 1;
[A, B, C, D] = ssdata(c2d(G_s, Ts, 'zoh'));
[n, m] = size(B);

% Augmented State-Space for Tracking (Delta u formulation)
Phi   = [A, B; zeros(m, n), eye(m)];
Gamma = [B; eye(m)];
Cbar  = [C, zeros(1, m)];

%% 2. MPC Parameters
N = 10;          % Prediction horizon
Q = 2; R = 0.5;  % Weights
Q_ = eye(N) * Q; 
R_ = eye(N) * R;

% Constraints
umax = 2; umin = -2;
du_max = 1; du_min = -1;

%% 3. Optimization Matrices (Prediction)
W = zeros(N, n+m);
G = zeros(N, N);

for i = 1:N
    W(i,:) = Cbar * (Phi^i);
    for j = 1:i
        G(i,j) = Cbar * (Phi^(i-j)) * Gamma;
    end
end

% Hessian H (Constant)
H = 2 * (G' * Q_ * G + R_);

%% 4. QP Solver Setup
% Current state: x_0 (from system), u_prev (from previous step)
x_0 = [8; -5]; u_prev = 0; 
x_aug = [x_0; u_prev];
ref = 1; 

% Gradient f (Depends on state x_aug and reference r)
f = 2 * ( (W * x_aug - ref * ones(N, 1))' * Q_ * G )';

% Constraints: 
% 1. dU limits: du_min <= dU <= du_max
% 2. U limits: umin <= L*dU + u_{prev} <= umax  => umin-u_{prev} <= L*dU <= umax-u_{prev}
L = tril(ones(N, N));
A_c = [eye(N); -eye(N); L; -L];
b_c = [du_max*ones(N,1); -du_min*ones(N,1); (umax-u_prev)*ones(N,1); -(umin-u_prev)*ones(N,1)];

% Solver
options = optimoptions('quadprog', 'Display', 'none');
dU_opt = quadprog(H, f, A_c, b_c, [], [], [], [], [], options);

%% 5. Visualization (Optional)
figure('Color', 'w');
stairs(dU_opt, 'LineWidth', 2);
grid on; title('Optimal Control Increments (\Delta u)');
