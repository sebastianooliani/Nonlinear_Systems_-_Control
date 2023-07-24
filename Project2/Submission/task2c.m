function u_4  = yawController(X, ref, u_1, u_2, u_3)
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