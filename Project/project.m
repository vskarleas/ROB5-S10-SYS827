%% ========================================================================
%  SYS827
%  Utilisant Robotics System Toolbox
%  ========================================================================
clear; clc; close all;

%% ========================================================================
%  PARTIE 1: CINEMATIQUE DIRECTE (DH Modifié)
%  ========================================================================

%% 1.1 Chargement du robot UR5
fprintf('Chargement du robot UR5\n');
robot = loadrobot("universalUR5", "DataFormat", "row");
% show(robot);

% disp('Informations du robot:');
% showdetails(robot);

%% 1.2 DH Modifiés

fprintf('\nDH Modifiés du UR5\n');

% Dimensions du UR5
% source: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
% https://forum.universal-robots.com/t/denavit-hartenberg-parameters-for-ur5-classical-or-the-modified/21698/3 (verification que ca suit le DH classic)
d1 = 0.089159;   % Distance axe 1-2
a2 = -0.42500;   % Longueur du bras
a3 = -0.39225;   % Longueur de l'avant-bras
d4 = 0.10915;    % Distance axe 3-4
d5 = 0.09465;    % Distance axe 4-5
d6 = 0.0823;     % Distance axe 5-6

% Table DH Modifié: [alpha(i-1), a(i-1), d(i), theta(i)]
DH_params = [
    0,      0,      d1,     0; 
    -pi/2,  0,      0,      0;  
    0,      a2,     0,      0;  
    0,      a3,     d4,     0;  
    pi/2,   0,      d5,     0;   
    -pi/2,  0,      d6,     0  
];

% Afficher la table DH
fprintf('\nTable DH Modifié:\n');
fprintf('Joint\talpha(i-1)\ta(i-1)\t\td(i)\t\ttheta(i)\n');
fprintf('-----\t---------\t------\t\t----\t\t--------\n');
for i = 1:6
    fprintf('%d\t%.4f\t\t%.5f\t\t%.5f\t\tvariable\n', i, DH_params(i,1), DH_params(i,2), DH_params(i,3));
end

%% 1.3 Calcul cinematique directe
q_test = [0, -pi/2, 0, -pi/2, 0, 0];

fprintf('\nCinématique Directe\n');

% source representative de la pose attendue : https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-serv-man/E-series/Images/SelectJointZeroinginthe_4.png (https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-serv-man/E-series/serv-man-joint-zeroing.htm#:~:text=Power%20on%20the%20robot%20and,will%20power%20down%20after%20confirmation.)
fprintf('Configuration de test: q = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] rad\n', q_test);

% Calcul avec notre fonction
T_result = cinematique_directe_UR5(q_test, DH_params);

fprintf('\nMatrice de transformation T_06 (notre calcul):\n');
disp(T_result);

position = T_result(1:3, 4);
fprintf('Position de l''effecteur: [%.4f, %.4f, %.4f] m\n', position);

% Comparaison avec Robotics System Toolbox
T_toolbox = getTransform(robot, q_test, 'tool0');
fprintf('\nMatrice de transformation T_06 (Robotics System Toolbox):\n');
disp(T_toolbox);



figure;
show(robot, q_test); % pose est verifie sur la figure affichée

%% ========================================================================
%  PARTIE 2: CINEMATIQUE INVERSE
%  ========================================================================

fprintf('\nCinématique Inverse\n');

%% 2.1 IK du robot avec robtics system toolbox
ik = inverseKinematics('RigidBodyTree', robot);

% ses poids sotn utlilisés pour prioriser certains joints pour favoriser la choix 
% de la solution optimale pour attendre la pose cible. Ici les joints 6, 5, 4 faut 
% rester le plus alignés possible avec la pose cible
weights = [0.25, 0.25, 0.25, 1, 1, 1]; 
initialGuess = robot.homeConfiguration;

target_position = [0.4; 0.1; 0.3];  
target_rotation = eul2rotm([0, pi, 0]);  % convention euler ZYX (rotation autour de Y de pi)

% Matrice de transformation cible
T_target = [target_rotation, target_position; 0 0 0 1];


fprintf('Pose cible: [%.3f, %.3f, %.3f] m\n', target_position);
disp('Matrice de transformation cible:');
disp(T_target);

%% 2.2 Resoltin de la cinematique inverse
q_solution  = ik('tool0', T_target, weights, initialGuess);

fprintf('Solution de ik, q = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] rad\n', q_solution);

%% 2.3 Verifier la solution
T_verify = getTransform(robot, q_solution, 'tool0');

% pour la verifcation du calcul, 
position_error = norm(T_verify(1:3, 4) - target_position); 
fprintf('\nErreur de position: %.6f m\n', position_error);

figure;
subplot(1,2,1);
show(robot, q_solution);
title('Configuration trouvée par IK');

subplot(1,2,2);
show(robot, q_test);
title('Configuration de référence');

%% 2.5 Decomposition du mouvement (trajectoire circulaire)
fprintf('\nTest avec plusieurs poses\n');

n_points = 10;
theta_traj = linspace(0, 2*pi, n_points);
radius = 0.1;
center = [0.4; 0.0; 0.3];

solutions = zeros(n_points, 6);
figure;

for i = 1:n_points
    % Position sur le cercle
    pos = center + [radius*cos(theta_traj(i)); radius*sin(theta_traj(i)); 0];
    T_traj = [target_rotation, pos; 0 0 0 1];
    
    % Résolution IK
    if i == 1
        guess = initialGuess;
    else
        guess = solutions(i-1, :);
    end
    
    [q_sol, info] = ik('tool0', T_traj, weights, guess);
    solutions(i, :) = q_sol;
    
    fprintf('Point %d: Status = %s\n', i, info.Status);
end

% Animation
for i = 1:n_points
    show(robot, solutions(i,:));
    drawnow;
    pause(0.3);
end


%% ========================================================================
%  ========================================================================

%% Fonction: Matrice de transformation DH Modifié (cours 1, page 30, SYS827)
function T = DH_Modified_Transform(alpha, a, d, theta)
    % T = Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta)
    T = [cos(theta),              -sin(theta),             0,            a;
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),   -sin(alpha),  -sin(alpha)*d;
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d;
         0,                       0,                       0,            1];
end

%% Fonction: Cinématique directe UR5
function T_0n = cinematique_directe_UR5(q, DH_params)
    % q: vecteur des angles articulaires [q1, q2, q3, q4, q5, q6]
    % DH_params: table des paramètres DH
    
    T_0n = eye(4);
    
    for i = 1:6
        alpha = DH_params(i, 1);
        a = DH_params(i, 2);
        d = DH_params(i, 3);
        theta = q(i);
        
        T_i = DH_Modified_Transform(alpha, a, d, theta);
        T_0n = T_0n * T_i;
    end
end
