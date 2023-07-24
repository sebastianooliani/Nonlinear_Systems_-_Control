function u_4  = yawController(X, ref, u123)
Ix = 16.57e-6;
Iy = 29.51e-6;
Iz = 29.26e-6;

% control parameters
beta_0 = 1;
ro = 10^-3 / Iz;
a = 0.8;

sigma = X(12) + a * X(9) * cos(X(8)) / cos(X(7)) + X(11) * tan(X(7));

v = - (ro + beta_0) * sign(sigma); % with chattering
% v = - (ro + beta_0) * tanh(sigma); % without chattering

c = - (Ix - Iy) * X(10) * X(11) ...
    - Iz * a * X(11) * tan(X(7)) ...
    - Iz * a * X(12);

u_4 = c + Iz * v;

end