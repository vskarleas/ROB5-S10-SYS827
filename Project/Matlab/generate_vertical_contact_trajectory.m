function [t_vec, x_d, xd_d, xdd_d, z_table] = ...
    generate_vertical_contact_trajectory(robot, q0, Tf, dt, z_offset_above_table, z_penetration)
% generate_vertical_contact_trajectory genere une trajectoire desiree
% d'approche verticale vers une table pour une tache en contact.
%
% La trajectoire est definie dans l'espace de la tache pour l'effecteur
% 'gripper_tip' et comprend deux phases :
%   - phase 1 : descente verticale vers la zone de contact ;
%   - phase 2 : maintien de la position desiree.
%
% La trajectoire produite ici correspond a la consigne geometrique initiale.
% Lors de la simulation, cette consigne pourra ensuite etre corrigee par la
% loi de commande en contact afin de tenir compte de la force exercee par
% l'environnement.
%
% Entrees :
%   robot                : Objet RigidBodyTree avec le corps 'gripper_tip'
%   q0                   : Configuration articulaire initiale
%   Tf                   : Duree totale de la trajectoire (s)
%   dt                   : Pas de temps (s)
%   z_offset_above_table : Distance initiale entre l'effecteur et la table (m)
%   z_penetration        : Penetration desiree sous la surface de la table (m)
%
% Sorties :
%   t_vec   : Vecteur temps
%   x_d     : Trajectoire desiree en position cartesienne (3xN)
%   xd_d    : Trajectoire desiree en vitesse cartesienne (3xN)
%   xdd_d   : Trajectoire desiree en acceleration cartesienne (3xN)
%   z_table : Hauteur de la surface de contact

    q0 = q0(:)';

    t_vec = 0:dt:Tf;
    N = length(t_vec);

    % Position initiale de l'effecteur
    T0 = getTransform(robot, q0, 'gripper_tip');
    x0 = T0(1:3,4);

    % Position de la table, placee sous l'effecteur initial
    z_table = x0(3) - z_offset_above_table;

    % Position initiale desiree
    x_init = x0;

    % Position finale desiree :
    % la trajectoire de reference amene l'effecteur legerement sous la
    % surface de la table. Cette penetration souhaitee n'est pas suivie
    % rigidement : elle sera ensuite corrigee par la loi de commande en
    % contact en fonction de la force d'interaction.
    x_final = [x0(1);
               x0(2);
               z_table - z_penetration];

    % Allocation memoire
    x_d   = zeros(3,N);
    xd_d  = zeros(3,N);
    xdd_d = zeros(3,N);

    % Un polynome du 5e degre est utilise pour la phase d'approche afin
    % d'imposer des vitesses et accelerations nulles au debut et a la fin
    T_move = 0.6 * Tf;   % 60 % du temps total pour la descente
    T_hold = Tf - T_move;

    for k = 1:N
        t = t_vec(k);

        if t <= T_move
            s = t / T_move;

            % Loi de progression polynomiale du 5e degre
            sigma   = 10*s^3 - 15*s^4 + 6*s^5;
            dsigma  = (30*s^2 - 60*s^3 + 30*s^4) / T_move;
            ddsigma = (60*s - 180*s^2 + 120*s^3) / T_move^2;

            x_d(:,k)   = x_init + sigma * (x_final - x_init);
            xd_d(:,k)  = dsigma * (x_final - x_init);
            xdd_d(:,k) = ddsigma * (x_final - x_init);

        else
            % Phase de maintien de la position desiree
            x_d(:,k)   = x_final;
            xd_d(:,k)  = [0;0;0];
            xdd_d(:,k) = [0;0;0];
        end
    end
end