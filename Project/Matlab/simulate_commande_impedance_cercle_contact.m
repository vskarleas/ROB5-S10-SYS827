function [q_hist, qd_hist, qdd_hist, x_hist, xdot_hist, x_c_hist, err_hist, tau_hist, Fext_hist] = ...
    simulate_commande_impedance_cercle_contact(robot, q0, qd0, t_vec, x_d, xd_d, xdd_d, ...
                                               Kp_task, Kd_task, Md, Bd, Kd_imp, ...
                                               z_table, Ke_env, Be_env)
% simulate_commande_impedance_cercle_contact simule une commande en contact
% pour une trajectoire circulaire, en reutilisant la meme loi de commande
% que celle definie pour le cas vertical.
%
% Principe :
%   - la trajectoire desiree peut comporter un mouvement circulaire sous contact ;
%   - la loi de commande utilisee reste identique a celle de
%     simulate_commande_impedance_verticale ;
%   - seule la trajectoire de reference change.
%
% Cette fonction sert donc principalement de fonction d'interface afin
% d'appliquer la commande en contact a une tache circulaire sans dupliquer
% le code de simulation.
%
% Entrees :
%   robot     : Objet RigidBodyTree avec le corps 'gripper_tip'
%   q0, qd0   : Etat articulaire initial
%   t_vec     : Vecteur temps
%   x_d       : Trajectoire desiree en position cartesienne
%   xd_d      : Trajectoire desiree en vitesse cartesienne
%   xdd_d     : Trajectoire desiree en acceleration cartesienne
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
%   x_hist, xdot_hist         : Historiques cartesiens reels
%   x_c_hist                  : Historique de la trajectoire corrigee
%   err_hist                  : Erreur de suivi de la trajectoire corrigee
%   tau_hist                  : Historique des couples articulaires
%   Fext_hist                 : Historique de la force de contact

    [q_hist, qd_hist, qdd_hist, x_hist, xdot_hist, x_c_hist, err_hist, tau_hist, Fext_hist] = ...
        simulate_commande_impedance_verticale(robot, q0, qd0, t_vec, x_d, xd_d, xdd_d, ...
                                              Kp_task, Kd_task, Md, Bd, Kd_imp, ...
                                              z_table, Ke_env, Be_env);
end