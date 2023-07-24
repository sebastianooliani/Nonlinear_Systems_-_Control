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
% [x,y,z,xdot,ydot,zdot,phi,theta,psi,p,q,r]
X_init = [1,1,0,  0,0,0,  0,0,pi/8, 0,0,0]; % Initial conditions
% [x,y,z,psi]
ref_set = [1.5,1.3,1,0]; % Reference point pi/4


%% Controller parameters
a = 1;
b = 1;
