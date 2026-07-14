%% Segway Stabilization & Observer Design
% Project: Stabilization of an inverted pendulum on a cart.
% Reference: Télécom Physique Strasbourg - Segway Lab Session

clear all; close all; clc;

%% 1. System Parameters
mc = 1.5; mp = 0.5; g = 9.82; L = 1; d1 = 1e-2; d2 = 1e-2;

%% 2. State-Space Matrices
% A0, B0: Linearization around upward equilibrium (theta = 0)
A0 = [0 0 1 0; 0 0 0 1; 0 (g*mp)/mc -d1/mc -d2/(mc*L); 0 (g*(mc+mp))/(mc*L) -d1/(mc*L) -(d2*(mc+mp))/(mp*mc*L^2)];
B0 = [0; 0; 1/mc; 1/(mc*L)];

% Api, Bpi: Linearization around downward equilibrium (theta = pi)
Api=[0 0 1 0; 0 0 0 1; 0 -g*mp/mc -d1/mc d2/(mc*L); 0 (mc+mp)*g/(mc*L) d1/(mc*L) d2*(mc+mp)/(mp*mc*L^2)];
Bpi=[0; 0; 1/mc; -1/(mc*L)];

C=[1 0 0 0];

%% 3. Controller Design
% Comparison between LQR and Pole Placement
Q = 2*eye(4); R = 5;
[KK, ~, ~] = lqr(A0, B0, Q, R);
p = [-3.4, -3.8, -0.4287+0.3627i, -0.4287-0.3627i];
K = place(A0, B0, p);

% Pre-compensation gain for static error (N)
C = [1 0 0 0];
N = -inv(C * inv(A0 - B0*K) * B0);

%% 4. Observer Design (Luenberger)
C1 = [1 0 0 0; 0 1 0 0]; % Measuring position and angle
p_obs = p / 0.5;         % Observer poles 2x faster than controller
Lobs = place(A0', C1', p_obs);

% Observer State-Space Matrices
Aobs = A0 - Lobs' * C1;
Bobs = [Lobs' B0];
Cobs = eye(4);
Dobs = zeros(4, 3);
