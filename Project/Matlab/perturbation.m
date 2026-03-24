function [x_pert, err_pert, tau_pert, q_pert, F_ext_hist] = perturbation( ...
    robot, traj_full, xd_traj, xdd_traj, ...
    q0, Md, Kd, Bd, dt, N_total, ...
    F_contact, t_contact_start, t_contact_end, t_vec)
% PERTURBATION Simule la commande en impedance avec une force externe
% transitoire appliquee sur l'effecteur.
%
% Cette fonction reproduit la boucle de simulation de la partie 4.5 mais
% en ajoutant une force de contact F_contact pendant l'intervalle
% [t_contact_start, t_contact_end]. Cela permet de valider le
% comportement compliant du controleur en impedance : selon la relation
%   F = K_d * x_def, une force de F N avec K_d N/m doit produire
%   une deflexion de F/K_d metres.
%
% Entrees :
%   robot           - Objet RigidBodyTree (avec gripper_tip)
%   traj_full       - Trajectoire cartesienne desiree (3 x N_total)
%   xd_traj         - Vitesses cartesiennes desirees (3 x N_total)
%   xdd_traj        - Accelerations cartesiennes desirees (3 x N_total)
%   q0              - Configuration articulaire initiale (1 x 6)
%   Md, Kd, Bd      - Matrices d'impedance desiree (3 x 3 chacune)
%   dt              - Pas de temps de simulation (s)
%   N_total         - Nombre total de points
%   F_contact       - Vecteur de force de contact (3 x 1), ex. [20; 0; 0]
%   t_contact_start - Debut de la perturbation (s)
%   t_contact_end   - Fin de la perturbation (s)
%   t_vec           - Vecteur de temps (1 x N_total)
%
% Sorties :
%   x_pert     - Trajectoire cartesienne reelle perturbee (3 x N_total)
%   err_pert   - Erreur de position (3 x N_total)
%   tau_pert   - Couples articulaires (N_total x 6)
%   q_pert     - Configurations articulaires (N_total x 6)
%   F_ext_hist - Force externe appliquee a chaque pas (3 x N_total)

    fprintf('\n--- Simulation avec perturbation ---\n');
    fprintf('Force de contact : [%.1f, %.1f, %.1f] N\n', F_contact);
    fprintf('Intervalle : [%.2f, %.2f] s\n', t_contact_start, t_contact_end);
    fprintf('Deflexion attendue (regime permanent) : [%.4f, %.4f, %.4f] m\n', ...
        Kd \ F_contact);

    % Preallocations
    x_pert     = zeros(3, N_total);
    err_pert   = zeros(3, N_total);
    tau_pert   = zeros(N_total, 6);
    q_pert     = zeros(N_total, 6);
    F_ext_hist = zeros(3, N_total);

    % Conditions initiales
    q  = q0;
    qd = zeros(1, 6);

    for k = 1:N_total
        % Cinematique directe du gripper
        T_current = getTransform(robot, q, 'gripper_tip');
        x_current = T_current(1:3, 4);

        % Jacobien geometrique au point du gripper
        J_geo = geometricJacobian(robot, q, 'gripper_tip');
        Jv_ctrl = J_geo(4:6, :);

        % Vitesse cartesienne actuelle
        xdot_current = Jv_ctrl * qd';

        % Erreurs cartesiennes
        e_pos = traj_full(:,k) - x_current;
        e_vel = xd_traj(:,k)  - xdot_current;

        % Force externe : active seulement pendant l'intervalle de contact
        if t_vec(k) >= t_contact_start && t_vec(k) <= t_contact_end
            F_ext = F_contact;
        else
            F_ext = zeros(3, 1);
        end

        % Loi de commande en impedance
        xdd_ref = xdd_traj(:,k) + Md \ (Kd * e_pos + Bd * e_vel - F_ext);

        % Passage en espace articulaire
        Jv_pinv = pinv(Jv_ctrl);
        qdd_ref = Jv_pinv * xdd_ref;

        % Couple de commande par dynamique inverse
        M_q  = massMatrix(robot, q);
        g_q  = gravityTorque(robot, q);
        vp_q = velocityProduct(robot, q, qd);

        tau_cmd = (M_q * qdd_ref + (-vp_q)' + g_q')';

        % Integration (Euler semi-implicite)
        qdd = (M_q \ (tau_cmd' - (-vp_q)' - g_q'))';
        qd  = qd + qdd * dt;
        q   = q  + qd  * dt;

        % Enregistrement
        q_pert(k,:)     = q;
        x_pert(:,k)     = x_current;
        err_pert(:,k)   = e_pos;
        tau_pert(k,:)    = tau_cmd;
        F_ext_hist(:,k) = F_ext;

        % Progression
        if mod(k, round(N_total/10)) == 0
            fprintf('  %.0f%%\n', 100*k/N_total);
        end
    end

    fprintf('Simulation avec perturbation terminee.\n');

    % Verification de la deflexion mesuree
    % On mesure la deflexion maximale pendant la phase de contact
    idx_contact = (t_vec >= t_contact_start) & (t_vec <= t_contact_end);
    if any(idx_contact)
        deflexion_max = max(abs(err_pert(:, idx_contact)), [], 2);
        fprintf('\nDeflexion maximale mesuree pendant le contact :\n');
        fprintf('  dx = %.4f m, dy = %.4f m, dz = %.4f m\n', deflexion_max);
        fprintf('  Norme = %.4f m\n', norm(deflexion_max));

        deflexion_attendue = abs(Kd \ F_contact);
        fprintf('Deflexion attendue (F/K) :\n');
        fprintf('  dx = %.4f m, dy = %.4f m, dz = %.4f m\n', deflexion_attendue);
    end
end