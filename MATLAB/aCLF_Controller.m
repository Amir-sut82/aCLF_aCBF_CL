function [u, theta_hat_dot, info] = aCLF_Controller(x, theta_hat, params)
% Certainty-equivalence aCLF-QP for the bi-copter, with the stability-based
% update law  theta_hat_dot = Gamma * phi(x).

    phi = x(1);  th  = x(2);
    p   = x(4);  q   = x(5);  r = x(6);
    omega = [p; q; r];

    y     = [phi; th; x(3)] - params.x_eq(1:3);
    J     = euler_rate_matrix(phi, th);
    y_dot = J*omega;

    eta = [y; y_dot];
    P   = params.P;
    V   = eta' * P * eta;

    P11 = P(1:3,1:3);  P12 = P(1:3,4:6);  P22 = P(4:6,4:6);

    [dJ_dphi, dJ_dth] = euler_rate_partials(phi, th);
    J_dot  = dJ_dphi*y_dot(1) + dJ_dth*y_dot(2);
    alpha0 = J_dot * omega;

    [~, F_reg, g_mat] = Dynamics(x, params);
    F_omega = F_reg(4:6,:);
    G_omega = g_mat(4:6,:);

    alpha_F = J * F_omega;
    A_dec   = J * G_omega;

    sigma     = 2*(y'*P12 + y_dot'*P22);
    omega_clf = 2*(y'*P11 + y_dot'*P12')*y_dot + sigma*alpha0;
    phi_clf   = alpha_F' * sigma';
    LgV       = sigma * A_dec;

    b_clf = -omega_clf - phi_clf'*theta_hat - params.gamma_clf*V;

    H    = 2*eye(4);
    f_qp = zeros(4,1);
    lb   = params.u_min*ones(4,1);
    ub   = params.u_max*ones(4,1);
    opts = optimoptions('quadprog','Display','off');
    [u, ~, flag] = quadprog(H, f_qp, LgV, b_clf, [], [], lb, ub, [], opts);
    if flag < 0, u = zeros(4,1); end

    theta_hat_dot = params.Gamma_adapt * phi_clf;

    info.V         = V;
    info.omega_clf = omega_clf;
    info.phi_clf   = phi_clf;
    info.LgV       = LgV;
    info.b_clf     = b_clf;
    info.eta       = eta;
    info.exitflag  = flag;
end


function J = euler_rate_matrix(phi, th)
    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(th);   sth  = sin(th);
    if abs(cth) < 1e-6, cth = sign(cth + 1e-6)*1e-6; end
    tth = sth/cth;
    J = [1, sphi*tth, cphi*tth;
         0, cphi,    -sphi;
         0, sphi/cth, cphi/cth];
end


function [dJ_dphi, dJ_dtheta] = euler_rate_partials(phi, th)
    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(th);   sth  = sin(th);
    if abs(cth) < 1e-6, cth = sign(cth + 1e-6)*1e-6; end
    tth  = sth/cth;
    sec2 = 1/cth^2;
    dJ_dphi = [0, cphi*tth, -sphi*tth;
               0,-sphi,     -cphi;
               0, cphi/cth, -sphi/cth];
    dJ_dtheta = [0, sphi*sec2,     cphi*sec2;
                 0, 0,             0;
                 0, sphi*sth*sec2, cphi*sth*sec2];
end
