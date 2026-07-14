%% MPC Stabilization Initialization
% Project: Stabilization of Y(s)/U(s) = 1/(s(s+1))
% Reference: Télécom Physique Strasbourg - MPC Practicals

clear all; close all; clc;

%% 1. System Discretization
G_s = tf(1, [1 1 0]);
Ts = 1;
[A, B, C, D] = ssdata(c2d(G_s, Ts, 'zoh'));

%% 2. MPC Parameters & Horizon
n = size(A, 1);
m = size(B, 2);
N = 10; % Prediction horizon
x0 = [8; -5]
Q = diag([2, 1]); % State weight matrix 
R = 0.5;          % Control effort weight
P = diag([1, 1]); % Terminal cost 

%% 3. Constraints Definition
umax = 4; umin = -4; % Constraints on u 

Q_=zeros(2*N,2*N);
R_=eye(N)*R;
for i=1:N-1
    Q_(2*i-1:2*i,2*i-1:2*i)=Q;
end
Q_(2*N-1:2*N,2*N-1:2*N)=P;
S = zeros(N*n, N*m); % Matrice de commande
T_ = zeros(N*n, n);  % Matrice d'état libre
for i = 1:N
    T_( (i-1)*n + 1 : i*n, : ) = A^i;
    for j = 1:i
        S( (i-1)*n + 1 : i*n, (j-1)*m + 1 : j*m ) = A^(i-j) * B;
    end
end

%% 4. Optimization Matrices Formulation
% Construction of H (Hessian) and F (Linear term)
% Using the prediction model: X_pred = T*x0 + S*U
H = 2 * (R_ + S'*Q_*S)
F = 2 * (x0'*T'*Q_*S)

%% 5. Optimization (QP Solver)
% Minimise 1/2 * U'*H*U + f'*U under constraint A_c*U <= b_c
f = (x0' * T_' * Q_* S)'; 
A_c = [eye(N); -eye(N)]; % Contraintes d'entrée : -4 <= u <= 4
b_c = [umax*ones(N,1); -umin*ones(N,1)];

options = optimoptions('quadprog', 'Display', 'none');
U_opt = quadprog(H, F, A_c, b_c, [], [], [], [], [], options);

%% 6. Plotting
figure('Color', 'w');
stairs(0:N-1, U_opt, 'LineWidth', 2);
grid on;
title('Séquence de commande optimale (Horizon N=10)');
xlabel('Pas de temps (k)');
ylabel('Commande u(k)');
ylim([umin-0.5, umax+0.5]); % Pour visualiser la saturation
