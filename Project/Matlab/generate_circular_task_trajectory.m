function [t_vec, x_d, xd_d, xdd_d, centre] = generate_circular_task_trajectory(robot, q0, Tf, dt, rayon, nb_tours)
% generate_circular_task_trajectory genere une trajectoire circulaire
% desiree dans l'espace de la tache pour l'effecteur 'gripper_tip'.
%
% La trajectoire est definie dans le plan XY avec Z constant :
%   x_d(t)   = xc + R cos(w t)
%   y_d(t)   = yc + R sin(w t)
%   z_d(t)   = z0
%
% Entrees :
%   robot    : Objet RigidBodyTree avec le gripper deja ajoute
%   q0       : Configuration articulaire initiale (1x6 ou 6x1)
%   Tf       : Duree totale de la trajectoire (s)
%   dt       : Pas de temps (s)
%   rayon    : Rayon du cercle (m)
%   nb_tours : Nombre de tours a effectuer (optionnel, par defaut = 1)
%
% Sorties :
%   t_vec    : Vecteur temps (1xN)
%   x_d      : Position desiree (3xN)
%   xd_d     : Vitesse desiree (3xN)
%   xdd_d    : Acceleration desiree (3xN)
%   centre   : Centre du cercle (3x1)

    if nargin < 6
        nb_tours = 1;
    end

    q0 = q0(:)';   % s'assurer d'un format ligne

    %% Vecteur temps
    t_vec = 0:dt:Tf;
    N = length(t_vec);

    %% Position initiale de l'outil
    T0 = getTransform(robot, q0, 'gripper_tip');
    x0 = T0(1:3,4);

    %% Parametres du cercle
    omega = 2*pi*nb_tours / Tf;

    % On choisit le centre de sorte que le premier point du cercle soit x0
    centre = x0 - [rayon; 0; 0];

    %% Allocation memoire
    x_d   = zeros(3, N);
    xd_d  = zeros(3, N);
    xdd_d = zeros(3, N);

    %% Generation de la trajectoire desiree
    for k = 1:N
        t = t_vec(k);

        x_d(:,k) = [centre(1) + rayon*cos(omega*t);
                    centre(2) + rayon*sin(omega*t);
                    x0(3)];

        xd_d(:,k) = [-rayon*omega*sin(omega*t);
                      rayon*omega*cos(omega*t);
                      0];

        xdd_d(:,k) = [-rayon*omega^2*cos(omega*t);
                      -rayon*omega^2*sin(omega*t);
                       0];
    end
end