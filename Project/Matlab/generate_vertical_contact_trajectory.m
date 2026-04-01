function [t_vec, x_d, xd_d, xdd_d, z_table] = ...
    generate_vertical_contact_trajectory(robot, q0, Tf, dt, z_offset_above_table, z_penetration)
% generate_vertical_contact_trajectory genere une trajectoire desiree
% simple pour une commande d'impedance en contact vertical :
%   - phase 1 : approche verticale de la table
%   - phase 2 : maintien sur la table
%
% La trajectoire est definie dans l'espace de la tache pour 'gripper_tip'.
%
% Entrees :
%   robot                : Objet RigidBodyTree avec gripper_tip
%   q0                   : Configuration articulaire initiale
%   Tf                   : Duree totale (s)
%   dt                   : Pas de temps (s)
%   z_offset_above_table : Distance initiale au-dessus de la table (m)
%   z_penetration        : Penetration desiree sous la table (m)
%
% Sorties :
%   t_vec   : Vecteur temps
%   x_d     : Position desiree (3xN)
%   xd_d    : Vitesse desiree (3xN)
%   xdd_d   : Acceleration desiree (3xN)
%   z_table : Hauteur de la table

    q0 = q0(:)';

    t_vec = 0:dt:Tf;
    N = length(t_vec);

    % Position initiale de l'outil
    T0 = getTransform(robot, q0, 'gripper_tip');
    x0 = T0(1:3,4);

    % On place la table sous l'outil initial
    z_table = x0(3) - z_offset_above_table;

    % Position initiale desiree
    x_init = x0;

    % Position finale desiree : l'outil "veut" penetrer legerement la table
    % et c'est l'impedance qui va gerer la reaction au contact
    x_final = [x0(1);
               x0(2);
               z_table - z_penetration];

    % Allocation
    x_d   = zeros(3,N);
    xd_d  = zeros(3,N);
    xdd_d = zeros(3,N);

    % On utilise un polynome du 5e degre pour la phase d'approche
    % afin d'avoir vitesse et acceleration nulles au debut et a la fin
    T_move = 0.6 * Tf;   % 60% du temps pour descendre


    for k = 1:N
        t = t_vec(k);

        if t <= T_move
            s = t / T_move;

            % Polynome 5e degre
            sigma   = 10*s^3 - 15*s^4 + 6*s^5;
            dsigma  = (30*s^2 - 60*s^3 + 30*s^4) / T_move;
            ddsigma = (60*s - 180*s^2 + 120*s^3) / T_move^2;

            x_d(:,k) = x_init + sigma * (x_final - x_init);
            xd_d(:,k) = dsigma * (x_final - x_init);
            xdd_d(:,k) = ddsigma * (x_final - x_init);

        else
            % Phase de maintien
            x_d(:,k) = x_final;
            xd_d(:,k) = [0;0;0];
            xdd_d(:,k) = [0;0;0];
        end
    end
end