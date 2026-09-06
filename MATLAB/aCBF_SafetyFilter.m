function [u, theta_hat_dot_cbf, info] = aCBF_SafetyFilter(x, u_nom, theta_hat, params)
% HOCBF-based aCBF safety filter for the pitch limit  h(x) = theta_max - theta >= 0.

    phi = x(1);  th = x(2);
    p = x(4);  q = x(5);  r = x(6);
    omega = [p; q; r];

    h = params.theta_max - th;

    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(th);   sth  = sin(th);
    if abs(cth) < 1e-6, cth = sign(cth + 1e-6)*1e-6; end

    J2 = [0, cphi, -sphi];
    theta_dot = J2 * omega;
    h_dot = -theta_dot;

    alpha1 = params.alpha1;
    alpha2 = params.alpha2;
    psi1   = h_dot + alpha1 * h;

    phi_dot = p + sphi*(sth/cth)*q + cphi*(sth/cth)*r;
    J2_dot_omega = -sphi*phi_dot*q - cphi*phi_dot*r;

    [~, F_reg, g_mat] = Dynamics(x, params);
    F_omega = F_reg(4:6,:);
    G_omega = g_mat(4:6,:);

    Lf_psi1 = -J2_dot_omega - alpha1*theta_dot;
    phi_h   = -(J2*F_omega)';
    Lg_psi1 = -(J2*G_omega);

    H    = 2*eye(4);
    f_qp = -2*u_nom;
    A    = -Lg_psi1;
    b    = Lf_psi1 + phi_h'*theta_hat + alpha2*psi1;
    lb   = params.u_min*ones(4,1);
    ub   = params.u_max*ones(4,1);
    opts = optimoptions('quadprog','Display','off');
    [u, ~, flag] = quadprog(H, f_qp, A, b, [], [], lb, ub, u_nom, opts);
    if flag < 0, u = max(lb, min(ub, u_nom)); end

    cv = Lf_psi1 + phi_h'*theta_hat + Lg_psi1*u + alpha2*psi1;

    theta_hat_dot_cbf = -params.Gamma_adapt * phi_h;

    info.h              = h;
    info.h_dot          = h_dot;
    info.psi1           = psi1;
    info.Lf_psi1        = Lf_psi1;
    info.phi_h          = phi_h;
    info.Lg_psi1        = Lg_psi1;
    info.constraint_val = cv;
    info.filtered       = (cv < 1e-6);
    info.exitflag       = flag;
end
