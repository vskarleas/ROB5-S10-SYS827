function [t_vec, x_d, xd_d, xdd_d, centre_cercle] = ...
    generate_contact_circle_trajectory(robot, q0, Tf, dt, z_table, z_penetration, rayon, nb_tours)
% generate_contact_circle_trajectory genere une trajectoire desiree pour
% une tache circulaire sous contact dans l'espace de la tache.
%
% La trajectoire est composee de dans l'espace de la tache.
%
% La trajectoire est composee de trois phases :
%   - phase 1 : approche vers le point de depart du cercle au niveau de contact ;
%   - phase 2 : maintien au point de contact initial ;
%   - phase 3 : mouvement circulaire dans le plan XY avec une hauteur Z constante.
%
% Cette fonction ne contient pas la loi de commande elle-meme. Elle produit
% uniquement la trajectoire de reference qui sera ensuite suivie et, si
% necessaire, corrigee par la loi de commande en contact.
%
% Entrees :
%   robot         : Objet RigidBodyTree avec le corps 'gripper_tip'
%   q0            : Configuration articulaire initiale
%   Tf            : Duree totale de la trajectoire
%   dt            : Pas de temps
%   z_table       : Hauteur de la surface de contact
%   z_penetration : Penetration desiree sous la table
%   rayon         : Rayon du cercle
%   nb_tours      : Nombre de tours a effectuer
%
% Sorties :
%   t_vec         : Vecteur temps
%   x_d           : Trajectoire desiree en position cartesienne (3xN)
%   xd_d          : Trajectoire desiree en vitesse cartesienne (3xN)
%   xdd_d         : Trajectoire desiree en acceleration cartesienne (3xN)
%   centre_cercle : Centre geometrique du cercle (3x1)

    q0 = q0(:)';
    t_vec = 0:dt:Tf;
    N = length(t_vec);

    % Position initiale de l'effecteur
    T0 = getTransform(robot, q0, 'gripper_tip');
    x0 = T0(1:3,4);

    % Niveau de travail souhaite sous contact
    z_contact = z_table - z_penetration;

    % Le centre est choisi de sorte que le premier point du cercle
    % coincide avec le point de depart en contact
    centre_cercle = [x0(1) - rayon;
                     x0(2);
                     z_contact];

    % Point de depart de la phase circulaire
    x_contact_start = [x0(1);
                       x0(2);
                       z_contact];

    % Repartition temporelle des trois phases
    T_approach = 0.20 * Tf;     % approche vers le point de contact initial
    T_hold     = 0.15 * Tf;     % maintien pour stabilisation
    T_circle   = Tf - T_approach - T_hold;

    if T_circle <= 0
        error('La duree totale Tf est trop petite.');
    end

    % Allocation memoire
    x_d   = zeros(3,N);
    xd_d  = zeros(3,N);
    xdd_d = zeros(3,N);

    for k = 1:N
        t = t_vec(k);

        if t <= T_approach
            % Phase 1 : deplacement vers le point de depart du cercle
            % avec une loi polynomiale lisse
            s = t / T_approach;

            sigma   = 10*s^3 - 15*s^4 + 6*s^5;
            dsigma  = (30*s^2 - 60*s^3 + 30*s^4) / T_approach;
            ddsigma = (60*s - 180*s^2 + 120*s^3) / T_approach^2;

            x_d(:,k)   = x0 + sigma * (x_contact_start - x0);
            xd_d(:,k)  = dsigma * (x_contact_start - x0);
            xdd_d(:,k) = ddsigma * (x_contact_start - x0);

        elseif t <= T_approach + T_hold
            % Phase 2 : maintien au point de contact initial
            x_d(:,k)   = x_contact_start;
            xd_d(:,k)  = [0;0;0];
            xdd_d(:,k) = [0;0;0];

        else
            % Phase 3 : mouvement circulaire dans le plan XY
            % avec evolution progressive de la vitesse angulaire
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