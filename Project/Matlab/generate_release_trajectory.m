function [t_vec, x_d, xd_d, xdd_d] = ...
    generate_release_trajectory(robot, q0, Tf, dt, z_table, z_offset_above_table)
% generate_release_trajectory genere une trajectoire verticale de retrait
% apres le cercle sous contact.
%
% Entrees :
%   robot               : Objet RigidBodyTree avec gripper_tip
%   q0                  : Configuration initiale
%   Tf                  : Duree totale
%   dt                  : Pas de temps
%   z_table             : Hauteur de la table
%   z_offset_above_table: Hauteur finale au-dessus de la table
%
% Sorties :
%   t_vec   : vecteur temps
%   x_d     : position desiree (3xN)
%   xd_d    : vitesse desiree (3xN)
%   xdd_d   : acceleration desiree (3xN)

    q0 = q0(:)';
    t_vec = 0:dt:Tf;
    N = length(t_vec);

    % Position initiale de l'outil
    T0 = getTransform(robot, q0, 'gripper_tip');
    x0 = T0(1:3,4);

    % Position finale : remonter au-dessus de la table
    x_final = [x0(1);
               x0(2);
               z_table + z_offset_above_table];

    x_d   = zeros(3,N);
    xd_d  = zeros(3,N);
    xdd_d = zeros(3,N);

    for k = 1:N
        t = t_vec(k);
        s = t / Tf;

        sigma   = 10*s^3 - 15*s^4 + 6*s^5;
        dsigma  = (30*s^2 - 60*s^3 + 30*s^4) / Tf;
        ddsigma = (60*s - 180*s^2 + 120*s^3) / Tf^2;

        x_d(:,k)   = x0 + sigma * (x_final - x0);
        xd_d(:,k)  = dsigma * (x_final - x0);
        xdd_d(:,k) = ddsigma * (x_final - x0);
    end
end