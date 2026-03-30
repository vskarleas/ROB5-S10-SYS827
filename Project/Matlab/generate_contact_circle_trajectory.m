function [t_vec, x_d, xd_d, xdd_d, centre_cercle] = ...
    generate_contact_circle_trajectory(robot, q0, Tf, dt, z_table, z_penetration, rayon, nb_tours)
% generate_contact_circle_trajectory genere une trajectoire pour une
% commande d'impedance sous contact :
%   - phase 1 : mise en contact / recentrage vers le point de depart du cercle
%   - phase 2 : maintien du contact
%   - phase 3 : cercle dans le plan XY avec Z constant
%
% Entrees :
%   robot         : Objet RigidBodyTree avec gripper_tip
%   q0            : Configuration articulaire initiale
%   Tf            : Duree totale
%   dt            : Pas de temps
%   z_table       : Hauteur de la table
%   z_penetration : Penetration desiree sous la table
%   rayon         : Rayon du cercle
%   nb_tours      : Nombre de tours
%
% Sorties :
%   t_vec         : Vecteur temps
%   x_d           : Position desiree (3xN)
%   xd_d          : Vitesse desiree (3xN)
%   xdd_d         : Acceleration desiree (3xN)
%   centre_cercle : Centre du cercle (3x1)

    q0 = q0(:)';
    t_vec = 0:dt:Tf;
    N = length(t_vec);

    % Position initiale de l'outil
    T0 = getTransform(robot, q0, 'gripper_tip');
    x0 = T0(1:3,4);

    % Niveau de travail en contact
    z_contact = z_table - z_penetration;

    % On choisit le centre de sorte que le premier point du cercle
    % soit exactement au point de contact de depart
    centre_cercle = [x0(1) - rayon;
                     x0(2);
                     z_contact];

    x_contact_start = [x0(1);
                       x0(2);
                       z_contact];

    % Repartition des phases
    T_approach = 0.20 * Tf;     % rejoindre proprement le point de contact
    T_hold     = 0.15 * Tf;     % stabilisation
    T_circle   = Tf - T_approach - T_hold;

    if T_circle <= 0
        error('La duree totale Tf est trop petite.');
    end

    % Allocation
    x_d   = zeros(3,N);
    xd_d  = zeros(3,N);
    xdd_d = zeros(3,N);

    for k = 1:N
        t = t_vec(k);

        if t <= T_approach
            % Phase 1 : aller vers le point de depart du cercle au niveau de contact
            s = t / T_approach;

            sigma   = 10*s^3 - 15*s^4 + 6*s^5;
            dsigma  = (30*s^2 - 60*s^3 + 30*s^4) / T_approach;
            ddsigma = (60*s - 180*s^2 + 120*s^3) / T_approach^2;

            x_d(:,k)   = x0 + sigma * (x_contact_start - x0);
            xd_d(:,k)  = dsigma * (x_contact_start - x0);
            xdd_d(:,k) = ddsigma * (x_contact_start - x0);

        elseif t <= T_approach + T_hold
            % Phase 2 : maintien au point de contact de depart
            x_d(:,k)   = x_contact_start;
            xd_d(:,k)  = [0;0;0];
            xdd_d(:,k) = [0;0;0];

        else
            % Phase 3 : cercle en XY avec vitesse tangente progressive
            tau = t - T_approach - T_hold;
            s = tau / T_circle;

            sigma   = 10*s^3 - 15*s^4 + 6*s^5;
            dsigma  = (30*s^2 - 60*s^3 + 30*s^4) / T_circle;
            ddsigma = (60*s - 180*s^2 + 120*s^3) / T_circle^2;

            theta    = 2*pi*nb_tours * sigma;
            theta_d  = 2*pi*nb_tours * dsigma;
            theta_dd = 2*pi*nb_tours * ddsigma;

            x_d(:,k) = [centre_cercle(1) + rayon*cos(theta);
                        centre_cercle(2) + rayon*sin(theta);
                        z_contact];

            xd_d(:,k) = [-rayon*sin(theta)*theta_d;
                          rayon*cos(theta)*theta_d;
                          0];

            xdd_d(:,k) = [-rayon*cos(theta)*theta_d^2 - rayon*sin(theta)*theta_dd;
                          -rayon*sin(theta)*theta_d^2 + rayon*cos(theta)*theta_dd;
                           0];
        end
    end
end