function [f0, F_reg, g] = Dynamics(x, params)
% Returns the (f0, F, g) decomposition of   xdot = f0(x) + F(x)*theta + g(x)*u.

    phi = x(1);  th = x(2);
    p = x(4);  q = x(5);  r = x(6);
    omega = [p; q; r];

    J = euler_rate_matrix(phi, th);

    f0 = [J*omega; 0; 0; 0];

    F_omega = [ p*q, -q*r,  0,    0,           0,   -p;
                0,    0,    p*r, -(p^2 - r^2), 0,   -q;
               -q*r,  0,    0,    0,           p*q, -r];

    F_reg = [zeros(3,6); F_omega];

    g = [zeros(3,4); params.I \ params.B_alloc];
end


function J = euler_rate_matrix(phi, th)
    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(th);   sth  = sin(th);
    if abs(cth) < 1e-6, cth = sign(cth + 1e-6)*1e-6; end
    tth = sth / cth;
    J = [1, sphi*tth, cphi*tth;
         0, cphi,    -sphi;
         0, sphi/cth, cphi/cth];
end
