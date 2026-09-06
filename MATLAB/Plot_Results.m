function Plot_Results(results, params)
% PLOT_RESULTS  Generate all figures with LaTeX interpreter and export to EPS

    th_max_d = rad2deg(params.theta_max);
    th_des_d = rad2deg(params.theta_ref);
    th_lab = {'$\Gamma_1$','$\Gamma_2$','$\Gamma_5$','$\Gamma_6$','$\Gamma_7$','$c_d$'};

    %% ============= TASK 1 =============
    if isfield(results,'task1')
        figure('Name','Task 1 Validation','Position',[50 300 1100 550]);
        ayl = {'$\phi$ [deg]','$\theta$ [deg]','$\psi$ [deg]'};
        ryl = {'$p$ [rad/s]','$q$ [rad/s]','$r$ [rad/s]'};
        atl = {'Roll $\phi$','Pitch $\theta$','Yaw $\psi$'};
        rtl = {'Roll rate $p$','Pitch rate $q$','Yaw rate $r$'};
        for k=1:3
            subplot(2,3,k);
            plot(results.task1.t_orig, rad2deg(results.task1.x_orig(:,k)),'b-','LineWidth',2); hold on;
            plot(results.task1.t_new, rad2deg(results.task1.x_new(:,k)),'r--','LineWidth',1.3);
            sty(ayl{k},'',atl{k});
            if k==1, legend({'Original: $\dot{x}=f+gu$','New: $\dot{x}=f_0+F\theta+gu$'},'Interpreter','latex','Location','best','FontSize',9); end
        end
        for k=1:3
            subplot(2,3,k+3);
            plot(results.task1.t_orig, results.task1.x_orig(:,k+3),'b-','LineWidth',2); hold on;
            plot(results.task1.t_new, results.task1.x_new(:,k+3),'r--','LineWidth',1.3);
            sty(ryl{k},'Time [s]',rtl{k});
        end
    end

    %% ============= TASK 2 =============
    if isfield(results,'task2')
        n=length(results.task2); c=lines(n);

        figure('Name','Task 2 Euler Angles','Position',[50 50 1100 500]);
        atl = {'Roll $\phi$','Pitch $\theta$','Yaw $\psi$'};
        ayl = {'$\phi$ [deg]','$\theta$ [deg]','$\psi$ [deg]'};
        for k=1:3, subplot(1,3,k);
            for ic=1:n, plot(results.task2(ic).t, rad2deg(results.task2(ic).x(:,k)),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
            sty(ayl{k},'Time [s]',[atl{k} ' --- aCLF']);
            if k==1, ileg(results.task2,c); end
        end

        figure('Name','Task 2 Norms','Position',[100 50 1100 450]);
        subplot(1,3,1);
        for ic=1:n, plot(results.task2(ic).t,vecnorm(results.task2(ic).x,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|x\|$','Time [s]','State norm $\|x(t)\|$'); ileg(results.task2,c);
        subplot(1,3,2);
        for ic=1:n, plot(results.task2(ic).t,vecnorm(results.task2(ic).u,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|u\|$','Time [s]','Control effort $\|u(t)\|$');
        subplot(1,3,3);
        for ic=1:n, plot(results.task2(ic).t,results.task2(ic).theta_tilde_norm,'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|\tilde{\theta}\|$','Time [s]','Parameter error $\|\tilde{\theta}(t)\|$');

        pfig('Task 2: aCLF --- Parameter Estimates', results.task2, params, th_lab, c);

        figure('Name','Task 2 CLF','Position',[200 50 750 400]);
        for ic=1:n, semilogy(results.task2(ic).t, max(results.task2(ic).V,1e-16),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$V(t)$','Time [s]','CLF value $V(t)$ (log scale)'); ileg(results.task2,c);
    end

    %% ============= TASK 3 =============
    if isfield(results,'task3')
        n=length(results.task3); c=lines(n);

        figure('Name','Task 3 Euler Angles','Position',[50 50 1100 500]);
        atl = {'Roll $\phi$','Pitch $\theta$','Yaw $\psi$'};
        ayl = {'$\phi$ [deg]','$\theta$ [deg]','$\psi$ [deg]'};
        for k=1:3, subplot(1,3,k);
            for ic=1:n, plot(results.task3(ic).t, rad2deg(results.task3(ic).x(:,k)),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
            sty(ayl{k},'Time [s]',[atl{k} ' --- aCLF+CL']);
            if k==1, ileg(results.task3,c); end
        end

        figure('Name','Task 3 Norms','Position',[100 50 1100 450]);
        subplot(1,3,1);
        for ic=1:n, plot(results.task3(ic).t,vecnorm(results.task3(ic).x,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|x\|$','Time [s]','State norm $\|x(t)\|$'); ileg(results.task3,c);
        subplot(1,3,2);
        for ic=1:n, plot(results.task3(ic).t,vecnorm(results.task3(ic).u,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|u\|$','Time [s]','Control effort $\|u(t)\|$');
        subplot(1,3,3);
        for ic=1:n, plot(results.task3(ic).t,results.task3(ic).theta_tilde_norm,'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|\tilde{\theta}\|$','Time [s]','Parameter error $\|\tilde{\theta}(t)\|$');

        pfig('Task 3: aCLF + CL --- Parameter Estimates', results.task3, params, th_lab, c);

        figure('Name','Task 3 CLF','Position',[200 50 750 400]);
        for ic=1:n, semilogy(results.task3(ic).t, max(results.task3(ic).V,1e-16),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$V(t)$','Time [s]','CLF value $V(t)$ (log scale)'); ileg(results.task3,c);

        % ---- COMPARISON: ||theta_tilde|| Task 2 vs Task 3 ----
        figure('Name','Task 3 CL vs No-CL','Position',[250 50 850 420]);
        if isfield(results,'task2')
            for ic=1:length(results.task2)
                plot(results.task2(ic).t, results.task2(ic).theta_tilde_norm,'--','Color',c(ic,:),'LineWidth',1.3,...
                     'DisplayName',['No CL --- ' results.task2(ic).label]); hold on;
            end
        end
        for ic=1:n
            plot(results.task3(ic).t, results.task3(ic).theta_tilde_norm,'-','Color',c(ic,:),'LineWidth',1.5,...
                 'DisplayName',['With CL --- ' results.task3(ic).label]); hold on;
        end
        sty('$\|\tilde{\theta}(t)\|$','Time [s]','$\|\tilde{\theta}\|$: No CL (dashed) vs CL (solid)');
        legend('Interpreter','latex','Location','best','FontSize',10);

        % ---- lambda_min(Phi) ----
        figure('Name','Task 3 Lambda Min','Position',[300 50 850 420]);
        for ic=1:n
            plot(results.task3(ic).t, results.task3(ic).lam_min,'-','Color',c(ic,:),'LineWidth',1.5); hold on;
        end
        sty('$\lambda_{\min}(\Phi)$','Time [s]','Minimum eigenvalue $\lambda_{\min}(\Phi)$');
        ileg(results.task3,c);
    end

    %% ============= TASK 4 =============
    if isfield(results,'task4')
        n=length(results.task4); c=lines(n);

        figure('Name','Task 4 Euler Angles','Position',[50 50 1100 500]);
        atl = {'Roll $\phi$','Yaw $\psi$'};
        ayl = {'$\phi$ [deg]','$\psi$ [deg]'};
        aidx = [1 3]; sp = [1 3];
        for j=1:2, subplot(1,3,sp(j));
            for ic=1:n, plot(results.task4(ic).t, rad2deg(results.task4(ic).x(:,aidx(j))),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
            sty(ayl{j},'Time [s]',[atl{j} ' --- aCBF']);
            if j==1, ileg(results.task4,c); end
        end
        subplot(1,3,2);
        for ic=1:n, plot(results.task4(ic).t, rad2deg(results.task4(ic).x(:,2)),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        yline(th_max_d,'r--','LineWidth',1.8); yline(th_des_d,'b:','LineWidth',1.5);
        sty('$\theta$ [deg]','Time [s]',['Pitch $\theta$ --- $\theta_{\max}=' num2str(th_max_d,'%.0f') '^\circ$, desired: $' num2str(th_des_d,'%.0f') '^\circ$']);
        legend({'$\theta(t)$','$\theta_{\max}$','$\theta_{\mathrm{des}}$'},'Interpreter','latex','Location','best','FontSize',10);

        figure('Name','Task 4 h(x)','Position',[100 50 850 420]);
        for ic=1:n, plot(results.task4(ic).t, rad2deg(results.task4(ic).h),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        yline(0,'r-','LineWidth',1.5);
        sty('$h(x)$ [deg]','Time [s]','Barrier value $h(x) = \theta_{\max} - \theta$');
        ileg(results.task4,c);

        figure('Name','Task 4 Norms','Position',[150 50 1100 450]);
        subplot(1,3,1);
        for ic=1:n, plot(results.task4(ic).t,vecnorm(results.task4(ic).x,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|x\|$','Time [s]','State norm $\|x(t)\|$'); ileg(results.task4,c);
        subplot(1,3,2);
        for ic=1:n, plot(results.task4(ic).t,vecnorm(results.task4(ic).u,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|u\|$','Time [s]','Control effort $\|u(t)\|$');
        subplot(1,3,3);
        for ic=1:n, plot(results.task4(ic).t,results.task4(ic).theta_tilde_norm,'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|\tilde{\theta}\|$','Time [s]','Parameter error $\|\tilde{\theta}(t)\|$');

        pfig('Task 4: aCBF --- Parameter Estimates', results.task4, params, th_lab, c);
    end

    %% ============= TASK 5 =============
    if isfield(results,'task5')
        n=length(results.task5); c=lines(n);

        figure('Name','Task 5 Euler Angles','Position',[50 50 1100 500]);
        atl = {'Roll $\phi$','Yaw $\psi$'};
        ayl = {'$\phi$ [deg]','$\psi$ [deg]'};
        aidx = [1 3]; sp = [1 3];
        for j=1:2, subplot(1,3,sp(j));
            for ic=1:n, plot(results.task5(ic).t, rad2deg(results.task5(ic).x(:,aidx(j))),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
            sty(ayl{j},'Time [s]',[atl{j} ' --- raCBF']);
            if j==1, ileg(results.task5,c); end
        end
        subplot(1,3,2);
        for ic=1:n, plot(results.task5(ic).t, rad2deg(results.task5(ic).x(:,2)),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        yline(th_max_d,'r--','LineWidth',1.8); yline(th_des_d,'b:','LineWidth',1.5);
        sty('$\theta$ [deg]','Time [s]',['Pitch $\theta$ --- raCBF, $\bar{\theta}=' num2str(params.theta_bar,'%.2f') '$']);
        legend({'$\theta(t)$','$\theta_{\max}$','$\theta_{\mathrm{des}}$'},'Interpreter','latex','Location','best','FontSize',10);

        % ---- h(x) comparison ----
        figure('Name','Task 5 h(x) Compare','Position',[100 50 850 420]);
        if isfield(results,'task4')
            for ic=1:n, plot(results.task4(ic).t, rad2deg(results.task4(ic).h),'--','Color',c(ic,:),'LineWidth',1.3); hold on; end
        end
        for ic=1:n, plot(results.task5(ic).t, rad2deg(results.task5(ic).h),'-','Color',c(ic,:),'LineWidth',1.5); hold on; end
        yline(0,'r-','LineWidth',1.5);
        sty('$h(x)$ [deg]','Time [s]','$h(x)$: aCBF (dashed) vs raCBF (solid)');

        % ---- Control effort comparison ----
        figure('Name','Task 5 Control Compare','Position',[150 50 850 420]);
        if isfield(results,'task4')
            for ic=1:n
                plot(results.task4(ic).t, vecnorm(results.task4(ic).u,2,2),'--','Color',c(ic,:),'LineWidth',1.3,...
                     'DisplayName',['aCBF --- ' results.task4(ic).label]); hold on;
            end
        end
        for ic=1:n
            plot(results.task5(ic).t, vecnorm(results.task5(ic).u,2,2),'-','Color',c(ic,:),'LineWidth',1.5,...
                 'DisplayName',['raCBF --- ' results.task5(ic).label]); hold on;
        end
        sty('$\|u\|$','Time [s]','$\|u\|$: aCBF (dashed) vs raCBF (solid)');
        legend('Interpreter','latex','Location','best','FontSize',10);

        figure('Name','Task 5 Norms','Position',[200 50 1100 450]);
        subplot(1,3,1);
        for ic=1:n, plot(results.task5(ic).t,vecnorm(results.task5(ic).x,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|x\|$','Time [s]','State norm $\|x(t)\|$'); ileg(results.task5,c);
        subplot(1,3,2);
        for ic=1:n, plot(results.task5(ic).t,vecnorm(results.task5(ic).u,2,2),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|u\|$','Time [s]','Control effort $\|u(t)\|$');
        subplot(1,3,3);
        for ic=1:n, plot(results.task5(ic).t,results.task5(ic).theta_tilde_norm,'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|\tilde{\theta}\|$','Time [s]','Parameter error $\|\tilde{\theta}(t)\|$');

        pfig('Task 5: raCBF --- Parameter Estimates', results.task5, params, th_lab, c);

        % ---- Robustness margin ----
        figure('Name','Task 5 Margin','Position',[300 50 850 420]);
        for ic=1:n, plot(results.task5(ic).t, results.task5(ic).margin,'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        sty('$\|\phi_h\| \bar{\theta}$','Time [s]','Robustness margin $\|\phi_h\| \bar{\theta}$');
        ileg(results.task5,c);
    end

    %% ============= EXPORT TO EPS =============
    drawnow;
    export_dir = 'plots_eps';
    if ~exist(export_dir,'dir'), mkdir(export_dir); end

    fig_handles = findall(0,'Type','figure');
    for i = 1:length(fig_handles)
        fig = fig_handles(i);
        if isempty(fig.Name)
            fname = sprintf('Figure_%d', fig.Number);
        else
            fname = regexprep(fig.Name, '[^a-zA-Z0-9]', '_');
        end
        fpath = fullfile(export_dir, [fname '.eps']);
        try
            exportgraphics(fig, fpath, 'ContentType','vector','BackgroundColor','w');
        catch
            print(fig, fullfile(export_dir, fname), '-depsc', '-r300');
        end
    end
    fprintf('All figures exported to: %s/\n', export_dir);
end

%% ================================================================
%  HELPERS
% =================================================================
function sty(yl, xl, tl)
    grid on; grid minor;
    set(gca,'FontSize',11,'LineWidth',1.2,'TickLabelInterpreter','latex');
    ylabel(yl,'Interpreter','latex','FontSize',13);
    if ~isempty(xl), xlabel(xl,'Interpreter','latex','FontSize',13); end
    if nargin>=3 && ~isempty(tl), title(tl,'Interpreter','latex','FontSize',14,'FontWeight','bold'); end
end

function ileg(res,c)
    n=length(res); h=gobjects(n,1);
    for ic=1:n, h(ic)=plot(NaN,NaN,'-','Color',c(ic,:),'LineWidth',1.5); end
    legend(h, arrayfun(@(s)s.label, res,'UniformOutput',false),'Interpreter','latex','Location','best','FontSize',9);
end

function pfig(ttl, res, params, th_lab, c)
    n=length(res);
    figure('Name',ttl,'Position',[100 50 1200 600]);
    for k=1:params.n_theta
        subplot(2,3,k);
        for ic=1:n, plot(res(ic).t, res(ic).theta_hat(:,k),'-','Color',c(ic,:),'LineWidth',1.3); hold on; end
        yline(params.theta_true(k),'k--','LineWidth',1.8);
        sty(['$\hat{\theta}_' num2str(k) '$'],'',th_lab{k});
        if k>=4, xlabel('Time [s]','Interpreter','latex','FontSize',13); end
        if k==params.n_theta
            legend([arrayfun(@(s)s.label, res,'UniformOutput',false), {'$\theta^*$ (true)'}],...
                   'Interpreter','latex','Location','best','FontSize',9);
        end
    end
end
