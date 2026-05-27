function [q_hist, qd_hist, qdd_hist, x_hist, xdot_hist, x_c_hist, err_hist, tau_hist, Fext_hist] = ...
    simulate_commande_impedance_verticale(robot, q0, qd0, t_vec, x_d, xd_d, xdd_d, ...
                                          Kp_task, Kd_task, Md, Bd, Kd_imp, ...
                                          z_table, Ke_env, Be_env)
% simulate_commande_impedance_verticale simule une commande en contact
% verticale basee sur une loi d'impedance implantee sous forme d'admittance.
%
% Principe :
%   1) une trajectoire corrigee x_c est calculee a partir de la trajectoire
%      desiree x_d et de la force de contact F_ext ;
%   2) cette trajectoire corrigee est ensuite suivie par une commande
%      dynamique dans l'espace de la tache.
%
% La correction n'est appliquee que sur l'axe Z, correspondant a la
% direction normale de contact avec la table. Les composantes X et Y
% suivent directement la trajectoire desiree.
%
% Entrees :
%   robot     : Objet RigidBodyTree avec le corps 'gripper_tip'
%   q0, qd0   : Etat articulaire initial (position et vitesse)
%   t_vec     : Vecteur temps
%   x_d       : Trajectoire desiree en position cartesienne (3xN)
%   xd_d      : Trajectoire desiree en vitesse cartesienne (3xN)
%   xdd_d     : Trajectoire desiree en acceleration cartesienne (3xN)
%   Kp_task   : Gain proportionnel de suivi dans l'espace de la tache
%   Kd_task   : Gain derive de suivi dans l'espace de la tache
%   Md        : Masse desiree de la loi d'impedance/admittance
%   Bd        : Amortissement desire de la loi d'impedance/admittance
%   Kd_imp    : Raideur desiree de la loi d'impedance/admittance
%   z_table   : Hauteur de la surface de contact
%   Ke_env    : Raideur du modele d'environnement
%   Be_env    : Amortissement du modele d'environnement
%
% Sorties :
%   q_hist, qd_hist, qdd_hist : Historiques articulaires
%   x_hist, xdot_hist         : Historiques cartesiennes reelles
%   x_c_hist                  : Historique de la trajectoire corrigee
%   err_hist                  : Erreur de suivi de la trajectoire corrigee
%   tau_hist                  : Historique des couples articulaires
%   Fext_hist                 : Historique de la force de contact

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

    % Initialisation : au debut de la simulation, la trajectoire corrigee
    % est prise egale a la trajectoire desiree
    x_c   = x_d(:,1);
    xd_c  = xd_d(:,1);
    xdd_c = xdd_d(:,1);

    Jv_prev = [];

    % Parametres numeriques de stabilisation
    lambda_pinv = 1e-2;   % coefficient de regularisation de la pseudo-inverse amortie
    qdd_max = 20;         % saturation sur l'acceleration articulaire (rad/s^2)
    Fz_max  = 50;         % saturation sur la force normale de contact (N)

    % Stabilisation de posture dans l'espace nul
    q_null = q0(:);
    Kq_null = 6;
    Dq_null = 2;

    for k = 1:N_total

        %% Etat courant du robot
        % Calcul de la position actuelle de l'effecteur
        T_current = getTransform(robot, q, 'gripper_tip');
        x_current = T_current(1:3,4);

        % Calcul du Jacobien geometrique et extraction de sa partie lineaire
        J_geo = geometricJacobian(robot, q, 'gripper_tip');
        Jv = J_geo(4:6,:);

        % Vitesse cartesienne actuelle de l'effecteur
        xdot_current = Jv * qd';

        %% Force externe d'environnement
        % Modele simplifie ressort-amortisseur normal a la table
        F_ext = [0;0;0];

        if x_current(3) <= z_table
            penetration = z_table - x_current(3);

            % On ne conserve que la vitesse d'enfoncement vers la table
            vz_down = min(xdot_current(3), 0);

            % Force normale de contact due a l'environnement
            Fz = Ke_env * penetration - Be_env * vz_down;
            Fz = max(Fz, 0);
            Fz = min(Fz, Fz_max);

            F_ext = [0;0;Fz];
        end

        %% 1) Calcul de la trajectoire corrigee x_c (admittance)
        % Les directions X et Y suivent directement la trajectoire desiree
        x_c(1:2)   = x_d(1:2,k);
        xd_c(1:2)  = xd_d(1:2,k);
        xdd_c(1:2) = xdd_d(1:2,k);

        % La correction d'impedance/admittance est appliquee uniquement sur Z
        Delta_z  = x_d(3,k)  - x_c(3);
        Delta_dz = xd_d(3,k) - xd_c(3);

        % Expression utilisee pour calculer l'acceleration corrigee sur Z.
        
        xdd_c(3) = xdd_d(3,k) + ...
                   ( Bd(3,3) * Delta_dz + Kd_imp(3,3) * Delta_z + F_ext(3) ) / Md(3,3);

        % Integration numerique de la trajectoire corrigee
        xd_c(3) = xd_c(3) + xdd_c(3) * dt;
        x_c(3)  = x_c(3)  + xd_c(3)  * dt;

        %% 2) Suivi de la trajectoire corrigee x_c
        % Erreurs de suivi dans l'espace de la tache
        e_pos = x_c  - x_current;
        e_vel = xd_c - xdot_current;

        % Commande PD dans l'espace de la tache
        u_x = xdd_c + Kd_task * e_vel + Kp_task * e_pos;

        %% Approximation de Jdot * qd
        if k == 1
            Jdot_qd = zeros(3,1);
        else
            Jdot = (Jv - Jv_prev) / dt;
            Jdot_qd = Jdot * qd';
        end

        %% Pseudo-inverse amortie
        % Utilisee pour calculer une acceleration articulaire de reference
        % a partir de la commande cartesienne
        Jv_pinv = Jv' / (Jv*Jv' + lambda_pinv^2 * eye(3));

        %% Stabilisation espace nul
        % Terme de posture ajoute dans l'espace nul du Jacobien
        u_null = Kq_null * (q_null - q') - Dq_null * qd';
        Nmat = eye(6) - Jv_pinv * Jv;

        qdd_ref = Jv_pinv * (u_x - Jdot_qd) + Nmat * u_null;

        % Saturation pour limiter les accelerations articulaires
        qdd_ref = max(min(qdd_ref, qdd_max), -qdd_max);

        %% Commande dynamique articulaire
        % Calcul des couples necessaires a partir de la dynamique inverse
        tau_cmd = inverseDynamics(robot, q, qd, qdd_ref');

        %% Simulation directe du robot
        % Calcul des termes dynamiques du modele
        M_q  = massMatrix(robot, q);
        vp_q = velocityProduct(robot, q, qd);
        g_q  = gravityTorque(robot, q);

        % Calcul de l'acceleration articulaire reelle a partir du modele
        qdd = (M_q \ (tau_cmd' + vp_q' - g_q'))';

        %% Integration Euler
        % Mise a jour des vitesses et positions articulaires
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