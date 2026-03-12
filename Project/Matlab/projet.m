close all;
clear;
clc;

%% ***************************************
% PARTIE 1
% ****************************************
fprintf('\n***************************************\n');
fprintf('PARTIE 1\n');
fprintf('***************************************\n');

%% =======================================
% Declaration du DH Modfié (Craig)
% ========================================

% Robot UR5 site officiel du constructeur
d1 = 0.089159;
a2 = -0.425;
a3 = - 0.39225;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;

% DH Modifié UR5 (Craig)
ai = [0; 0; a2; a3; 0; 0];
alphai = [0; pi/2; 0; 0; pi/2; -pi/2];
di = [d1; 0; 0; d4; d5; d6];


% Fonction: Matrice de transformation DH Modifié (cours 1, page 30, SYS827)
function T = DH_Modified_Transform(alpha, a, d, theta)
    % T = Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta)
    T = [cos(theta),              -sin(theta),             0,            a;
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),   -sin(alpha),  -sin(alpha)*d;
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d;
         0,                       0,                       0,            1];
end


%% =======================================
% Model cinematique direct du robot pour la pose donnée ci-dessous
% ========================================

% Pose articulaire pour notre example (voir rapport)
thetai = [deg2rad(-91.06); deg2rad(-111.79); deg2rad(-104.53); deg2rad(-55.59); deg2rad(90.79); deg2rad(-1.16)];

% Calcul des matrices de transformation individuelles T_{i-1}-{i}
T = cell(6,1);
for i = 1:6
    T{i} = DH_Modified_Transform(alphai(i), ai(i), di(i), thetai(i));
    fprintf('\n--- T_%d-%d ---\n', i-1, i);
    disp(T{i});
end

fprintf('\n========================================\n');


% Calcul des matrices de transformation cumulées T_0-i
T_cumul = cell(6,1);
T_cumul{1} = T{1};
for i = 2:6
    T_cumul{i} = T_cumul{i-1} * T{i};
end

% Affichage des matrices cumulées
for i = 1:6
    fprintf('\n=== T_0-%d ===\n', i);
    disp(T_cumul{i});
end


% Position et orientation de l'effecteur pour la pose (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16)
T06 = T_cumul{6};
fprintf('\n========================================\n');
fprintf('Pose de l''effecteur (T_0-6) :\n');
fprintf('========================================\n');
fprintf('Position (x, y, z) : [%f, %f, %f] m\n', T06(1,4), T06(2,4), T06(3,4));
fprintf('Matrice de rotation :\n');
disp(T06(1:3, 1:3));

%% =======================================
%% Model cinematique differentiel
% ========================================

% Liens symboliques
syms sq1 sq2 sq3 sq4 sq5 sq6 real
syms sq23 sq234


% DH Modifié UR5 symbolique (Craig)
ai_sym = [0; 0; a2; a3; 0; 0];
alphai_sym = [sym(0); sym(pi)/2; sym(0); sym(0); sym(pi)/2; -sym(pi)/2];
di_sym = [d1; 0; 0; d4; d5; d6];

thetai_sym = [sq1; sq2; sq3; sq4; sq5; sq6];

% Calcul des matrices de transformation individuelles T_{i-1}-{i}
T_sym = cell(6,1);
for i = 1:6
    T_sym{i} = simplify(DH_Modified_Transform(alphai_sym(i), ai_sym(i), di_sym(i), thetai_sym(i)));
end

% Calcul des matrices de transformation cumulées T_0-i
T_cumul_sym = cell(6,1);
T_cumul_sym{1} = T_sym{1};
for i = 2:6
    T_cumul_sym{i} = simplify(T_cumul_sym{i-1} * T_sym{i}, 'Steps', 50);
end


% Vecteur de position
T06_sym = T_cumul_sym{6};
px_sym = T06_sym(1, 4);
py_sym = T06_sym(2, 4);
pz_sym = T06_sym(3, 4);


% Calcul analytique de la Jacobienne (avec liens symbolics)
q_sym = [sq1, sq2, sq3, sq4, sq5, sq6];

Jp_sym = sym(zeros(3, 6));
for j = 1:6
    Jp_sym(1, j) = diff(px_sym, q_sym(j));
    Jp_sym(2, j) = diff(py_sym, q_sym(j));
    Jp_sym(3, j) = diff(pz_sym, q_sym(j));
end
Jp_sym = simplify(Jp_sym, 'Steps', 50);


% Simplification trigonometrique
Jp_display = Jp_sym;
Jp_display = subs(Jp_display, cos(sq2)*cos(sq3) - sin(sq2)*sin(sq3), cos(sq2+sq3));
Jp_display = subs(Jp_display, cos(sq2)*sin(sq3) + sin(sq2)*cos(sq3), sin(sq2+sq3));
Jp_display = subs(Jp_display, sin(sq2)*cos(sq3) + cos(sq2)*sin(sq3), sin(sq2+sq3));
Jp_display = simplify(Jp_display, 'Steps', 50);
Jp_display = subs(Jp_display, sq2+sq3, sq23);

Jp_display = subs(Jp_display, cos(sq23)*cos(sq4) - sin(sq23)*sin(sq4), cos(sq23+sq4));
Jp_display = subs(Jp_display, cos(sq23)*sin(sq4) + sin(sq23)*cos(sq4), sin(sq23+sq4));
Jp_display = subs(Jp_display, sin(sq23)*cos(sq4) + cos(sq23)*sin(sq4), sin(sq23+sq4));
Jp_display = simplify(Jp_display, 'Steps', 50);
Jp_display = subs(Jp_display, sq23+sq4, sq234);


% Affichage
fprintf('\n========================================\n');
fprintf('Jacobien de position symbolique Jp (3x6) :\n');
fprintf('========================================\n');
for j = 1:6
    fprintf('\n--- dP/dq%d ---\n', j);
    disp(Jp_display(:, j));
end


% Application numerique pour la pose (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16)
Jp_num = double(subs(Jp_sym, q_sym, thetai'));

fprintf('\n========================================\n');
fprintf('Jacobien numérique pour la pose donnée :\n');
fprintf('========================================\n');
disp(Jp_num);


%% =======================================
%% Model dynamique
% ========================================


%% ***************************************
% PARTIE 2
% ****************************************
fprintf('\n***************************************\n');
fprintf('PARTIE 2\n');
fprintf('***************************************\n');

%% =======================================
% Model inverse pour tester le model direct (control en position)
% ========================================

% Le principe est pour le x,y,z et l'orientation calculé par le modele
% direct ci-dessus, on alimente le modele inverse et on attend les angles
% des articulations qui seront les memes avec ceux qui sont données
% initialement au modele direct


% Chargement du model par Robotics Toolbox
robot = loadrobot("universalUR5", "DataFormat", "row");
ik = inverseKinematics('RigidBodyTree', robot);

% Recuperation des information par le partie du modele direct
target_position = T06(1:3, 4);
target_rotation = T06(1:3, 1:3);

T_target = [target_rotation, target_position; 0 0 0 1];

% Parametres pour la fonction inverseKinematics
weights = [1, 0.25, 0.25, 1, 1, 1]; % ses poids sotn utlilisés pour prioriser certains joints pour favoriser la choix de la solution optimale pour attendre la pose cible. Ici les joints 6, 5, 4 faut rester le plus alignés possible avec la pose cible
initialGuess = robot.homeConfiguration;

initialGuess(1) = deg2rad(-90.06);
initialGuess(2) = deg2rad(-111.79); % parametre qui etait modifie
initialGuess(3) = deg2rad(-104.53);
initialGuess(4) = deg2rad(-55.59);
initialGuess(5) = deg2rad(90.79);
initialGuess(6) = deg2rad(-1.16);

% --- Test No 1 : pour une pose donné (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16)
q_verification  = ik('tool0', T_target, weights, initialGuess);

fprintf('\n========================================\n');
fprintf('Test No 1 - Verification par modele inverse (un point) :\n');
fprintf('========================================\n');
fprintf('Angles originaux (deg)  : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', rad2deg(thetai));
fprintf('Angles IK trouvés (deg) : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', rad2deg(q_verification));
fprintf('Erreur (deg)            : [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', rad2deg(q_verification) - rad2deg(thetai'));



%% =======================================
% Vérification des modèles avec la tâche d'un suivi de trajectoire (cntrole
% en position)
% ========================================

distance = -0.20; % m  
N = 500; % nombre des points              
t = linspace(0, 1, N);

% On commence par la position initial
pos_init = T06(1:3, 4);
R_const  = T06(1:3, 1:3);

% Decomposition de la trajectoire dans l'espace cartesien (control en position)
positions = zeros(3, N); % liste des positions
for k = 1:N
    positions(:, k) = pos_init + [distance * t(k); 0; 0];
end

% Calcul de tous les poses à partir du IK
q_traj = zeros(N, 6); % liste des poses
currentGuess = initialGuess;


% --- Test No 2 : à partir d'une position et orientation donné sorti par la 
% pose initiale (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16) pour
% la tâche -0.2 m suivant X
fprintf('\n========================================\n');
fprintf('Test No 2 - Calcul de la trajectoire de -0.2m en X position : \n');
fprintf('========================================\n');

for k = 1:N
    T_desired = [R_const, positions(:,k); 0 0 0 1];
    [q_sol, solInfo] = ik('tool0', T_desired, weights, currentGuess);
    q_traj(k, :) = q_sol;
    
    currentGuess = q_sol;
end

fprintf('Trajectoire calculée avec succes (controle en position)\n');



% Robot 
figure('Name', 'Animation du suivi de trajectoire');
ax = show(robot, q_traj(1,:), 'Visuals', 'on');
hold on;


% Trajectoire à faire en jaune
plot3(positions(1,:), positions(2,:), positions(3,:), 'y-', 'LineWidth', 2);
title('Suivi de trajectoire rectiligne en X');
view([-45 30]);


% Animation
for k = 1:N
    show(robot, q_traj(k,:), 'Parent', ax, 'Visuals', 'on', 'PreservePlot', false);
    drawnow;
    pause(0.05);
end
hold off;


%% =======================================
% Vérification des modèles avec la tâche d'un suivi de trajectoire (controle
% en vitesse)
% ========================================

% --- Test No 3 :
% Vitesse cartesien désirée constante (uniquement en X)
dt = 1.0 / N; % for N voir Test No 1 pour la decomposition de la trajectoire
v_desired = [distance / 1.0; 0; 0];

q_traj_vel = zeros(N, 6);
positions_vel = zeros(3, N);
q_current = thetai';

fprintf('\n========================================\n');
fprintf('Test No 3 - Calcul de la trajectoire -0.2m en X vitesse : \n');
fprintf('========================================\n');

for k = 1:N
    q_traj_vel(k, :) = q_current;
    
    % Calcul du modèle direct pour position actuelle
    T_current = eye(4);
    for i = 1:6
        T_current = T_current * DH_Modified_Transform(alphai(i), ai(i), di(i), q_current(i));
    end
    positions_vel(:, k) = T_current(1:3, 4);
    
    % Calcul du Jacobien numerique à la pose actuelle
    Jp_k = double(subs(Jp_sym, q_sym, q_current));
    
    % Inversion du Jacobien (pseudo-inverse car 3x6)
    dq = pinv(Jp_k) * v_desired;
    
    % Intégration : mise à jour des angles
    q_current = q_current + dq' * dt;
end

fprintf('Trajectoire calculée avec succes (controle en vitesse)\n\n');

% Position finale atteinte vs désirée
pos_finale_vel = positions_vel(:, end);
pos_finale_desiree = pos_init + [distance; 0; 0];
fprintf('Position finale (vitesse)  : [%f, %f, %f] m\n', pos_finale_vel);
fprintf('Position finale (désirée)  : [%f, %f, %f] m\n', pos_finale_desiree);
fprintf('Erreur position            : [%f, %f, %f] mm\n', (pos_finale_vel - pos_finale_desiree)*1000);


