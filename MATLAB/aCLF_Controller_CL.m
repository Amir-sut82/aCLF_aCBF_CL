function [u, theta_hat_dot, info, stack] = aCLF_Controller_CL(x, theta_hat, stack, params)
% aCLF-QP with concurrent learning. Maintains the history stack internally
% using a lambda_min(Phi)-maximizing replacement rule.

    [u, ~, info] = aCLF_Controller(x, theta_hat, params);

    [~, F_j, ~] = Dynamics(x, params);

    if norm(F_j(4:6,:), 'fro') > 1e-4
        Y_j = F_j * params.theta_true;

        if stack.count < stack.N
            stack.count = stack.count + 1;
            stack.L{stack.count} = F_j;
            stack.Y{stack.count} = Y_j;
        else
            lam_cur = min(eig(compute_Phi(stack)));
            best_idx = -1;  best_lam = lam_cur;
            for jj = 1:stack.N
                L_old = stack.L{jj};
                stack.L{jj} = F_j;
                lam_try = min(eig(compute_Phi(stack)));
                stack.L{jj} = L_old;
                if lam_try > best_lam
                    best_lam = lam_try;  best_idx = jj;
                end
            end
            if best_idx > 0
                stack.L{best_idx} = F_j;
                stack.Y{best_idx} = Y_j;
            end
        end
    end

    cl_term = zeros(params.n_theta,1);
    for j = 1:stack.count
        cl_term = cl_term + stack.L{j}' * (stack.Y{j} - stack.L{j}*theta_hat);
    end
    cl_term = params.gamma_cl * cl_term;

    theta_hat_dot = params.Gamma_adapt * (info.phi_clf + cl_term);

    if stack.count > 0
        info.lam_min = min(eig(compute_Phi(stack)));
    else
        info.lam_min = 0;
    end
    info.cl_term    = cl_term;
    info.stack_size = stack.count;
end


function Phi = compute_Phi(stack)
    p = size(stack.L{1}, 2);
    Phi = zeros(p);
    for j = 1:stack.count
        Phi = Phi + stack.L{j}' * stack.L{j};
    end
end
