%% Master script  --  Adaptive Control Mini-Project 2
clear; clc; close all;

params  = System_Parameters();
results = struct();

%% Task 1  --  validate the (f0, F, g) reformulation
fprintf('=== TASK 1: Validation ===\n');

x0 = [deg2rad(5); deg2rad(3); deg2rad(10); 0.5; -0.3; 0.2];
u0 = zeros(4,1);

[t_new, x_new]   = ode45(@(t,x) ode_uncertain(x, u0, params.theta_nodamp, params), ...
                         [0 60], x0, params.ode_opts);
[t_orig, x_orig] = ode45(@(t,x) ode_original(x, u0, params), ...
                         [0 60], x0, params.ode_opts);

err = max(abs(x_new - interp1(t_orig, x_orig, t_new)), [], 1);
fprintf('  Max error: %.2e\n\n', max(err));

results.task1.t_new = t_new;  results.task1.x_new = x_new;
results.task1.t_orig = t_orig; results.task1.x_orig = x_orig;


%% Task 2  --  baseline aCLF (stability-based update)
fprintf('=== TASK 2: aCLF ===\n');

tspan = [0, params.t_final];

ICs = {
    [deg2rad(10);  0;          deg2rad(30);  0.5;  0.4;  0   ], 'IC1: Example One';
    [deg2rad(-5);  deg2rad(15); deg2rad(-20); 0.2; -0.1;  0.3 ], 'IC2: Example Two';
};

for ic = 1:size(ICs,1)
    fprintf('  Simulating %s ...\n', ICs{ic,2});
    z0 = [ICs{ic,1}; params.theta_hat0];
    [t, z] = ode45(@(tt,zz) ode_aclf_closed_loop(zz, params), tspan, z0, params.ode_opts);

    x          = z(:, 1:6);
    theta_hat  = z(:, 7:12);
    N          = length(t);
    u_log      = zeros(N,4);
    V_log      = zeros(N,1);
    for k = 1:N
        [u_k, ~, ik] = aCLF_Controller(x(k,:)', theta_hat(k,:)', params);
        u_log(k,:) = u_k';
        V_log(k)   = ik.V;
    end

    results.task2(ic).t                 = t;
    results.task2(ic).x                 = x;
    results.task2(ic).theta_hat         = theta_hat;
    results.task2(ic).u                 = u_log;
    results.task2(ic).V                 = V_log;
    results.task2(ic).label             = ICs{ic,2};
    results.task2(ic).theta_tilde_norm  = vecnorm(theta_hat - params.theta_true', 2, 2);
end
fprintf('  Task 2 complete.\n\n');


%% Task 3  --  aCLF with concurrent learning
fprintf('=== TASK 3: aCLF + Concurrent Learning ===\n');

dt  = 0.01;
t3v = (0:dt:params.t_final)';
Nt  = length(t3v);

for ic = 1:size(ICs,1)
    fprintf('  Simulating %s ...\n', ICs{ic,2});

    z   = [ICs{ic,1}; params.theta_hat0];
    x3  = zeros(Nt,6);  th3  = zeros(Nt,6);
    u3  = zeros(Nt,4);  V3   = zeros(Nt,1);  lam3 = zeros(Nt,1);

    stack.L = cell(params.N_stack,1);
    stack.Y = cell(params.N_stack,1);
    stack.count = 0;
    stack.N     = params.N_stack;

    for k = 1:Nt
        xk = z(1:6);  thk = z(7:12);
        x3(k,:) = xk';  th3(k,:) = thk';

        [uk, ~, ik, stack] = aCLF_Controller_CL(xk, thk, stack, params);
        u3(k,:) = uk';  V3(k) = ik.V;  lam3(k) = ik.lam_min;

        if k < Nt
            rhs = @(zz) rk_rhs(zz, stack, params);
            k1 = rhs(z);
            k2 = rhs(z + dt/2*k1);
            k3 = rhs(z + dt/2*k2);
            k4 = rhs(z + dt*k3);
            z  = z + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
    end

    results.task3(ic).t                = t3v;
    results.task3(ic).x                = x3;
    results.task3(ic).theta_hat        = th3;
    results.task3(ic).u                = u3;
    results.task3(ic).V                = V3;
    results.task3(ic).lam_min          = lam3;
    results.task3(ic).label            = ICs{ic,2};
    results.task3(ic).theta_tilde_norm = vecnorm(th3 - params.theta_true', 2, 2);

    fprintf('    Stack: %d/%d, lam_min=%.4f, final ||th_tilde||=%.4f\n', ...
        stack.count, stack.N, lam3(end), results.task3(ic).theta_tilde_norm(end));
end
fprintf('  Task 3 complete.\n\n');


%% Task 4  --  aCBF safety filter (reference = 30 deg > theta_max)
fprintf('=== TASK 4: aCBF Safety Filter ===\n');

params4       = params;
params4.x_eq  = [0; params.theta_ref; 0; 0; 0; 0];

for ic = 1:size(ICs,1)
    fprintf('  Simulating %s ...\n', ICs{ic,2});
    z0 = [ICs{ic,1}; params.theta_hat0; params.theta_hat0];
    [t, z] = ode45(@(tt,zz) ode_acbf_closed_loop(zz, params4, params), ...
                   tspan, z0, params.ode_opts);

    x         = z(:, 1:6);
    th_clf    = z(:, 7:12);
    th_cbf    = z(:, 13:18);
    N         = length(t);
    u_nom_log = zeros(N,4);  u_log = zeros(N,4);
    h_log     = zeros(N,1);  psi1_log = zeros(N,1);  V_log = zeros(N,1);

    for k = 1:N
        xk = x(k,:)';
        thk_cl = th_clf(k,:)';
        thk_cb = th_cbf(k,:)';
        [un, ~, ik]  = aCLF_Controller(xk, thk_cl, params4);
        [us, ~, cik] = aCBF_SafetyFilter(xk, un, thk_cb, params);

        u_nom_log(k,:) = un';
        u_log(k,:)     = us';
        h_log(k)       = cik.h;
        psi1_log(k)    = cik.psi1;
        V_log(k)       = ik.V;
    end

    results.task4(ic).t                = t;
    results.task4(ic).x                = x;
    results.task4(ic).theta_hat        = th_clf;
    results.task4(ic).theta_cbf        = th_cbf;
    results.task4(ic).u_nom            = u_nom_log;
    results.task4(ic).u                = u_log;
    results.task4(ic).h                = h_log;
    results.task4(ic).psi1             = psi1_log;
    results.task4(ic).V                = V_log;
    results.task4(ic).label            = ICs{ic,2};
    results.task4(ic).theta_tilde_norm = vecnorm(th_clf - params.theta_true', 2, 2);

    fprintf('    min h(x) = %.4f deg  (must be >= 0)\n', rad2deg(min(h_log)));
end
fprintf('  Task 4 complete.\n\n');


%% Task 5  --  raCBF safety filter (same scenario, tighter constraint)
fprintf('=== TASK 5: raCBF Safety Filter ===\n');

for ic = 1:size(ICs,1)
    fprintf('  Simulating %s ...\n', ICs{ic,2});
    z0 = [ICs{ic,1}; params.theta_hat0; params.theta_hat0];
    [t, z] = ode45(@(tt,zz) ode_racbf_closed_loop(zz, params4, params), ...
                   tspan, z0, params.ode_opts);

    x      = z(:, 1:6);
    th_clf = z(:, 7:12);
    th_cbf = z(:, 13:18);
    N      = length(t);
    u_nom_log = zeros(N,4);  u_log = zeros(N,4);
    h_log = zeros(N,1);  psi1_log = zeros(N,1);  margin_log = zeros(N,1);

    for k = 1:N
        xk = x(k,:)';
        thk_cl = th_clf(k,:)';
        thk_cb = th_cbf(k,:)';
        [un, ~, ~]   = aCLF_Controller(xk, thk_cl, params4);
        [us, ~, cik] = raCBF_SafetyFilter(xk, un, thk_cb, params);

        u_nom_log(k,:) = un';
        u_log(k,:)     = us';
        h_log(k)       = cik.h;
        psi1_log(k)    = cik.psi1;
        margin_log(k)  = cik.margin;
    end

    results.task5(ic).t                = t;
    results.task5(ic).x                = x;
    results.task5(ic).theta_hat        = th_clf;
    results.task5(ic).theta_cbf        = th_cbf;
    results.task5(ic).u_nom            = u_nom_log;
    results.task5(ic).u                = u_log;
    results.task5(ic).h                = h_log;
    results.task5(ic).psi1             = psi1_log;
    results.task5(ic).margin           = margin_log;
    results.task5(ic).label            = ICs{ic,2};
    results.task5(ic).theta_tilde_norm = vecnorm(th_clf - params.theta_true', 2, 2);

    fprintf('    min h(x) = %.4f deg  (aCBF was %.4f deg)\n', ...
        rad2deg(min(h_log)), rad2deg(min(results.task4(ic).h)));
end
fprintf('  Task 5 complete.\n\n');


%% Plots
Plot_Results(results, params);
fprintf('All done.\n');


%% ---- local helpers ----

function zdot = ode_racbf_closed_loop(z, params_clf, params_cbf)
    x = z(1:6);  th_clf = z(7:12);  th_cbf = z(13:18);
    [u_nom,  th_clf_dot] = aCLF_Controller(x, th_clf, params_clf);
    [u_safe, th_cbf_dot] = raCBF_SafetyFilter(x, u_nom, th_cbf, params_cbf);
    [f0, F_reg, g] = Dynamics(x, params_cbf);
    xdot = f0 + F_reg*params_cbf.theta_true + g*u_safe;
    zdot = [xdot; th_clf_dot; th_cbf_dot];
end

function zdot = ode_acbf_closed_loop(z, params_clf, params_cbf)
    x = z(1:6);  th_clf = z(7:12);  th_cbf = z(13:18);
    [u_nom,  th_clf_dot] = aCLF_Controller(x, th_clf, params_clf);
    [u_safe, th_cbf_dot] = aCBF_SafetyFilter(x, u_nom, th_cbf, params_cbf);
    [f0, F_reg, g] = Dynamics(x, params_cbf);
    xdot = f0 + F_reg*params_cbf.theta_true + g*u_safe;
    zdot = [xdot; th_clf_dot; th_cbf_dot];
end

function zdot = rk_rhs(z, stack, params)
    x = z(1:6);  th = z(7:12);
    [u, thd] = aCLF_Controller_CL(x, th, stack, params);
    [f0, F_reg, g] = Dynamics(x, params);
    xdot = f0 + F_reg*params.theta_true + g*u;
    zdot = [xdot; thd];
end

function zdot = ode_aclf_closed_loop(z, params)
    x = z(1:6);  theta_hat = z(7:12);
    [u, theta_hat_dot] = aCLF_Controller(x, theta_hat, params);
    [f0, F_reg, g] = Dynamics(x, params);
    xdot = f0 + F_reg*params.theta_true + g*u;
    zdot = [xdot; theta_hat_dot];
end

function xdot = ode_uncertain(x, u, theta, params)
    [f0, F_reg, g] = Dynamics(x, params);
    xdot = f0 + F_reg*theta + g*u;
end

function xdot = ode_original(x, u, params)
    phi = x(1);  th = x(2);
    p = x(4);  q = x(5);  r = x(6);
    omega = [p; q; r];
    cph = cos(phi); sph = sin(phi);
    cth = cos(th);  sth = sin(th);
    if abs(cth) < 1e-6, cth = sign(cth + 1e-6)*1e-6; end
    J  = [1, sph*sth/cth, cph*sth/cth;  0, cph, -sph;  0, sph/cth, cph/cth];
    tau = -cross(omega, params.I*omega);
    f   = [J*omega; params.I\tau];
    g   = [zeros(3,4); params.I\params.B_alloc];
    xdot = f + g*u;
end
