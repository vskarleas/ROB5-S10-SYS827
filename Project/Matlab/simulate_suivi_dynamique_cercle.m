function [q_hist, qd_hist, qdd_hist, x_hist, xdot_hist, err_hist, tau_hist] = ...
    simulate_suivi_dynamique_cercle(robot, q0, qd0, t_vec, x_d, xd_d, xdd_d, Kp_task, Kd_task)
% simulate_suivi_dynamique_cercle simule un suivi dynamique d'une
% trajectoire circulaire dans l'espace de la tache, sans contact avec
% l'environnement.
%
% La loi de commande suit la structure vue en cours :
%   u_x = xdd_d + Kd*(xd_d - xd) + Kp*(x_d - x)
%
% Puis, avec la relation :
%   xdd = J(q) qdd + Jdot(q,qd) qd
%
% on calcule :
%   qdd_ref = J^+ * (u_x - Jdot*qd)
%
% Enfin, la commande dynamique articulaire est :
%   tau = M(q) qdd_ref + h(q,qd) + g(q)
%
% Entrees :
%   robot    : Objet RigidBodyTree avec gripper_tip
%   q0       : Configuration articulaire initiale (1x6 ou 6x1)
%   qd0      : Vitesse articulaire initiale (1x6 ou 6x1)
%   t_vec    : Vecteur temps (1xN)
%   x_d      : Position desiree (3xN)
%   xd_d     : Vitesse desiree (3xN)
%   xdd_d    : Acceleration desiree (3xN)
%   Kp_task  : Gain proportionnel (3x3)
%   Kd_task  : Gain derive (3x3)
%
% Sorties :
%   q_hist    : Historique des positions articulaires (N x 6)
%   qd_hist   : Historique des vitesses articulaires (N x 6)
%   qdd_hist  : Historique des accelerations articulaires (N x 6)
%   x_hist    : Historique des positions cartesiennes reelles (3 x N)
%   xdot_hist : Historique des vitesses cartesiennes reelles (3 x N)
%   err_hist  : Historique de l'erreur cartesienne (3 x N)
%   tau_hist  : Historique des couples articulaires (N x 6)

    %% Initialisation
    q  = q0(:)';     % format ligne
    qd = qd0(:)';    % format ligne

    N_total = length(t_vec);

    if N_total < 2
        error('Le vecteur temps doit contenir au moins deux instants.');
    end

    dt = t_vec(2) - t_vec(1);

    %% Allocation memoire
    q_hist    = zeros(N_total, 6);
    qd_hist   = zeros(N_total, 6);
    qdd_hist  = zeros(N_total, 6);
    x_hist    = zeros(3, N_total);
    xdot_hist = zeros(3, N_total);
    err_hist  = zeros(3, N_total);
    tau_hist  = zeros(N_total, 6);

    %% Jacobien precedent pour approximer Jdot
    Jv_prev = [];

    %% Boucle de simulation
    for k = 1:N_total

        % --- Etat courant de l'effecteur ---
        T_current = getTransform(robot, q, 'gripper_tip');
        x_current = T_current(1:3, 4);

        J_geo = geometricJacobian(robot, q, 'gripper_tip');
        Jv = J_geo(4:6, :);   % partie lineaire uniquement

        xdot_current = Jv * qd';

        % --- Erreurs cartesiennes ---
        e_pos = x_d(:,k)  - x_current;
        e_vel = xd_d(:,k) - xdot_current;

        % --- Commande PD dans l'espace de la tache ---
        u_x = xdd_d(:,k) + Kd_task * e_vel + Kp_task * e_pos;

        % --- Approximation numerique de Jdot * qd ---
        if k == 1
            Jdot_qd = zeros(3,1);
        else
            Jdot = (Jv - Jv_prev) / dt;
            Jdot_qd = Jdot * qd';
        end

        % --- Reference articulaire par pseudo-inverse ---
        qdd_ref = pinv(Jv) * (u_x - Jdot_qd);   % 6x1

        % --- Commande dynamique articulaire ---
        tau_cmd = inverseDynamics(robot, q, qd, qdd_ref');

        % --- Simulation directe du robot ---
        M_q = massMatrix(robot, q);
        h_q = velocityProduct(robot, q, qd);   % termes centrifuges/Coriolis
        g_q = gravityTorque(robot, q);

        qdd = (M_q \ (tau_cmd' - h_q' - g_q'))';   % format ligne

        % --- Integration Euler explicite ---
        qd = qd + qdd * dt;
        q  = q  + qd  * dt;

        % --- Sauvegarde ---
        q_hist(k,:)    = q;
        qd_hist(k,:)   = qd;
        qdd_hist(k,:)  = qdd;
        x_hist(:,k)    = x_current;
        xdot_hist(:,k) = xdot_current;
        err_hist(:,k)  = e_pos;
        tau_hist(k,:)  = tau_cmd;

        % --- Mise a jour pour l'iteration suivante ---
        Jv_prev = Jv;
    end
end