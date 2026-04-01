function [q_hist, qd_hist, qdd_hist, x_hist, xdot_hist, x_c_hist, err_hist, tau_hist, Fext_hist] = ...
    simulate_commande_impedance_verticale(robot, q0, qd0, t_vec, x_d, xd_d, xdd_d, ...
                                          Kp_task, Kd_task, Md, Bd, Kd_imp, ...
                                          z_table, Ke_env, Be_env)
% simulate_commande_impedance_verticale simule une commande d'impedance
% verticale simple sur une table.
%
% Structure :
%   1) calcul d'une trajectoire corrigee x_c par admittance
%   2) suivi dynamique de x_c dans l'espace de la tache
%
% Entrees :
%   robot     : Objet RigidBodyTree avec gripper_tip
%   q0, qd0   : Etat initial
%   t_vec     : Vecteur temps
%   x_d       : Position desiree
%   xd_d      : Vitesse desiree
%   xdd_d     : Acceleration desiree
%   Kp_task   : Gain proportionnel de suivi task-space
%   Kd_task   : Gain derive de suivi task-space
%   Md        : Masse desiree de l'impedance
%   Bd        : Amortissement desire
%   Kd_imp    : Raideur desiree
%   z_table   : Hauteur de la table
%   Ke_env    : Raideur de l'environnement
%   Be_env    : Amortissement de l'environnement
%
% Sorties :
%   q_hist, qd_hist, qdd_hist : historiques articulaires
%   x_hist, xdot_hist         : historiques cartesiennes reelles
%   x_c_hist                  : trajectoire corrigee
%   err_hist                  : erreur de suivi de x_c
%   tau_hist                  : couples articulaires
%   Fext_hist                 : force de contact

    q  = q0(:)';
    qd = qd0(:)';

    N_total = length(t_vec);
    dt = t_vec(2) - t_vec(1);

    q_hist    = zeros(N_total,6);
    qd_hist   = zeros(N_total,6);
    qdd_hist  = zeros(N_total,6);
    x_hist    = zeros(3,N_total);
    xdot_hist = zeros(3,N_total);
    x_c_hist  = zeros(3,N_total);
    err_hist  = zeros(3,N_total);
    tau_hist  = zeros(N_total,6);
    Fext_hist = zeros(3,N_total);

    % Trajectoire corrigee initiale = trajectoire desiree initiale
    x_c   = x_d(:,1);
    xd_c  = xd_d(:,1);
    xdd_c = xdd_d(:,1);

    Jv_prev = [];

    % Parametres numeriques de stabilisation
    lambda_pinv = 1e-2;   % pseudo-inverse amortie
    qdd_max = 20;         % saturation acceleration articulaire (rad/s^2)
    Fz_max  = 50;         % saturation force normale (N)

    % Stabilisation de posture dans l'espace nul
    q_null = q0(:);
    Kq_null = 6;
    Dq_null = 2;

    for k = 1:N_total

        %% Etat courant du robot
        T_current = getTransform(robot, q, 'gripper_tip');
        x_current = T_current(1:3,4);

        J_geo = geometricJacobian(robot, q, 'gripper_tip');
        Jv = J_geo(4:6,:);

        xdot_current = Jv * qd';

        %% Force externe d'environnement
        F_ext = [0;0;0];

        if x_current(3) <= z_table
            penetration = z_table - x_current(3);

            % vitesse d'enfoncement uniquement
            vz_down = min(xdot_current(3), 0);

            Fz = Ke_env * penetration - Be_env * vz_down;
            Fz = max(Fz, 0);
            Fz = min(Fz, Fz_max);

            F_ext = [0;0;Fz];
        end

        %% 1) Calcul de la trajectoire corrigee x_c (admittance)
        % X et Y suivent directement la trajectoire desiree
        x_c(1:2)   = x_d(1:2,k);
        xd_c(1:2)  = xd_d(1:2,k);
        xdd_c(1:2) = xdd_d(1:2,k);

        % Impedance uniquement sur Z!
        Delta_z  = x_d(3,k)  - x_c(3);
        Delta_dz = xd_d(3,k) - xd_c(3);

     
        % xdd_c = xdd_d + Md^-1 * ( Bd*Delta_dz + Kd*Delta_z - Fext )
        xdd_c(3) = xdd_d(3,k) + ...
                   ( Bd(3,3) * Delta_dz + Kd_imp(3,3) * Delta_z + F_ext(3) ) / Md(3,3);

        % Integration de la trajectoire corrigee
        xd_c(3) = xd_c(3) + xdd_c(3) * dt;
        x_c(3)  = x_c(3)  + xd_c(3)  * dt;

        %% 2) Suivi de la trajectoire corrigee x_c
        e_pos = x_c  - x_current;
        e_vel = xd_c - xdot_current;

        u_x = xdd_c + Kd_task * e_vel + Kp_task * e_pos;

        %% Approximation de Jdot * qd
        if k == 1
            Jdot_qd = zeros(3,1);
        else
            Jdot = (Jv - Jv_prev) / dt;
            Jdot_qd = Jdot * qd';
        end

        %% Pseudo-inverse amortie
        Jv_pinv = Jv' / (Jv*Jv' + lambda_pinv^2 * eye(3));

        %% Stabilisation espace nul
        u_null = Kq_null * (q_null - q') - Dq_null * qd';
        Nmat = eye(6) - Jv_pinv * Jv;

        qdd_ref = Jv_pinv * (u_x - Jdot_qd) + Nmat * u_null;

        % Saturation
        qdd_ref = max(min(qdd_ref, qdd_max), -qdd_max);

        %% Commande dynamique articulaire
        tau_cmd = inverseDynamics(robot, q, qd, qdd_ref');

        %% Simulation directe du robot
        M_q  = massMatrix(robot, q);
        vp_q = velocityProduct(robot, q, qd);
        g_q  = gravityTorque(robot, q);

        

        % qdd = M^-1 * ( tau + velocityProduct - gravity )
        qdd = (M_q \ (tau_cmd' + vp_q' - g_q'))';

        %% Integration Euler
        qd = qd + qdd * dt;
        q  = q  + qd  * dt;

        %% Sauvegarde
        q_hist(k,:)    = q;
        qd_hist(k,:)   = qd;
        qdd_hist(k,:)  = qdd;
        x_hist(:,k)    = x_current;
        xdot_hist(:,k) = xdot_current;
        x_c_hist(:,k)  = x_c;
        err_hist(:,k)  = x_c - x_current;
        tau_hist(k,:)  = tau_cmd;
        Fext_hist(:,k) = F_ext;

        Jv_prev = Jv;
    end
end