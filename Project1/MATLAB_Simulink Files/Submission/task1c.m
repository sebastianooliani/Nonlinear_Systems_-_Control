%% Model parameters
m  = 28e-3;
Ix = 16.57e-6;
Iy = 29.51e-6;
Iz = 29.26e-6;
g  = 9.8;
kx = 10e-3;
ky = 10e-3;
kz = 10e-3;
kp = 10e-6;
kq = 10e-6;
kr = 10e-6;

%% Simulation Parameters

psi_0 = 2*pi/16;
%psi_0 = 2*pi/8;
%psi_0 = 2*pi/4;
%psi_0 = 2*pi/2;

% [x,y,z,xdot,ydot,zdot,phi,theta,psi,p,q,r]
X_init = [1.,1.,1.,  0,0,0,  0,0,psi_0, 0,0,0]; % Initial point

psi_t = 0;
%psi_t = pi/2;
% [x,y,z,xdot,ydot,zdot,phi,theta,psi,p,q,r]
X_ref = [0,0,0, 0,0,0, 0,0,psi_t, 0,0,0]; % Reference point

%X_ref = [30,20,10, 0,0,0, 0,0,psi_t, 0,0,0]; % Reference point

% steady state input
U_ss = [m*g; 0; 0; 0];

%% Linear Control with LQR
% define matrix A and B
n = 12; % order of the system
n_u = 4; % number of inputs
A = zeros(n, n);
B = zeros(n, n_u);

psi_lin = 0; % keep this fixed!
%psi_lin = pi/2;

A = [0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 -kx/m 0 0 sin(psi_lin)*g cos(psi_lin)*g 0 0 0 0;
    0 0 0 0 -ky/m 0 -cos(psi_lin)*g sin(psi_lin)*g 0 0 0 0;
    0 0 0 0 0 -kz/m 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 -kp/Ix 0 0;
    0 0 0 0 0 0 0 0 0 0 -kq/Iy 0;
    0 0 0 0 0 0 0 0 0 0 0 -kr/Iz];

B = [zeros(5, n_u);
    1/m 0 0 0;
    zeros(3, n_u);
    0 1/Ix 0 0;
    0 0 1/Iy 0;
    0 0 0 1/Iz];

Q = eye(n);
R = eye(n_u);
sys = ss(A, B, ones(1, n), zeros(1, n_u));

K = lqr(sys, Q, R);
