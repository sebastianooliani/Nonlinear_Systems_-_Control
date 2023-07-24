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
X_init = [1,1,0,  0,0,0,  0,0,1, 0,0,0]; % Initial conditions
% [x,y,z,psi]
ref_set = [1.5,-1.5,1,0]; % Reference point pi/4


%% Controller parameters
function u_4  = yawController(X,ref,u_1,u_2,u_3)
% control params
K_1 = 4; % P
K_2 = 3; % D
% system params
Ix = 16.57e-6;
Iy = 29.51e-6;
Iz = 29.26e-6;
kq = 10e-6;
kr = 10e-6;

den = cos(X(7))/(Iz * cos(X(8)));
num = (1 / Iy) * ((Iz - Ix) * X(10) * X(12) + u_3 - kq * X(11)) * sin(X(7)) / cos(X(8)) + ...
    (X(11) * cos(X(7)) / cos(X(8)) - X(12) * sin(X(7)) / cos(X(8))) * (X(10) + X(11) * sin(X(7)) * tan(X(8)) + X(12) * cos(X(7)) * tan(X(8))) + ...
    (X(11) * sin(X(7)) * sin(X(8)) + X(12) * cos(X(7)) * sin(X(8))) * (X(11) * cos(X(7)) - X(12) * sin(X(7))) * (1 / (cos(X(8))) ^ 2) + ...
    (1 / Iz) * ((Ix - Iy) * X(10) * X(11) - kr * X(12)) * cos(X(7)) / cos(X(8));

psi_dot = X(11) * sin(X(7)) / cos(X(8)) + X(12) * cos(X(7)) / cos(X(8));
u_4 = (1 / den) * (- num - K_1 * (X(9) - ref(4)) - K_2 * psi_dot);

end
