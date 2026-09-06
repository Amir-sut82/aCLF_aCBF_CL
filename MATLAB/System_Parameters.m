function params = System_Parameters()
% All physical, controller and adaptive parameters for the bi-copter.
% State x = [phi; theta; psi; p; q; r],  input u in R^4,
% unknown theta = [G1; G2; G5; G6; G7; cd].

    params.m     = 0.486;
    params.g_acc = 9.81;

    params.Ixx = 0.0043;
    params.Iyy = 0.0142;
    params.Izz = 0.0176;
    params.Ixz = 0.0001;

    params.I = [ params.Ixx, 0, -params.Ixz;
                 0, params.Iyy, 0;
                -params.Ixz, 0, params.Izz];

    params.dcg = 0.175;
    params.hcg = 0.085;
    params.b   = 3.13e-5;
    params.d   = 1.2e-6;
    params.d_over_b = params.d / params.b;

    params.B_alloc = [0, 0,          params.dcg,      params.d_over_b;
                      0, params.hcg, 0,               0;
                      0, 0,          params.d_over_b, -params.dcg];

    Ixx = params.Ixx;  Iyy = params.Iyy;
    Izz = params.Izz;  Ixz = params.Ixz;
    Gam = Ixx*Izz - Ixz^2;

    Gam1 = Ixz*(Ixx - Iyy + Izz)/Gam;
    Gam2 = (Izz*(Izz - Iyy) + Ixz^2)/Gam;
    Gam5 = (Izz - Ixx)/Iyy;
    Gam6 = Ixz/Iyy;
    Gam7 = (Ixx*(Ixx - Iyy) + Ixz^2)/Gam;

    params.Gam  = Gam;
    params.Gam1 = Gam1;  params.Gam2 = Gam2;
    params.Gam5 = Gam5;  params.Gam6 = Gam6;  params.Gam7 = Gam7;

    params.c_d        = 0.01;
    params.theta_true = [Gam1; Gam2; Gam5; Gam6; Gam7; params.c_d];
    params.n_theta    = 6;
    params.theta_nodamp = [Gam1; Gam2; Gam5; Gam6; Gam7; 0];

    % Wrong initial guess on purpose
    params.theta_hat0 = [0; 0.5; 0.5; 0; -0.3; 0];
    params.theta_bar  = 1.5 * norm(params.theta_true - params.theta_hat0);

    params.Gamma_adapt = 10 * eye(params.n_theta);
    params.gamma_clf   = 0.5;
    params.gamma_cbf   = 5;
    params.gamma_cl    = 5;
    params.N_stack     = 20;

    params.theta_max = deg2rad(20);
    params.alpha1    = 5;
    params.alpha2    = 5;

    params.u_min = -0.1;
    params.u_max =  0.1;

    params.Kp = 4*eye(3);
    params.Kd = 4*eye(3);
    F_mat = [zeros(3), eye(3); zeros(3), zeros(3)];
    G_mat = [zeros(3); eye(3)];
    K     = [params.Kp, params.Kd];
    Acl   = F_mat - G_mat*K;
    P     = lyap(Acl', 0.1*eye(6));

    params.P = P;  params.F_mat = F_mat;  params.G_mat = G_mat;
    params.K = K;  params.Acl   = Acl;    params.Q     = 0.1*eye(6);

    params.p_penalty = 1e4;

    params.theta_ref = deg2rad(30);
    params.x_ref = [0; params.theta_ref; 0; 0; 0; 0];
    params.x_eq  = zeros(6,1);

    params.t_final  = 60;
    params.ode_opts = odeset('RelTol',1e-8, 'AbsTol',1e-10, 'MaxStep',0.01);
end
