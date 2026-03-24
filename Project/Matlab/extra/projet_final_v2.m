close all;
clear;
clc;

%% =========================================================
% INITIALISATION GENERALE
% ==========================================================
fprintf('\n========================================================\n');
fprintf('MODELISATION ET INITIALISATION DU ROBOT UR5\n');
fprintf('========================================================\n');

%% Parametres geometriques UR5 (par le constructeur)
d1 = 0.089159;
a2 = -0.425;
a3 = -0.39225;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;

%% Parametres DH modifies (Craig)
ai     = [0; 0; a2; a3; 0; 0];
alphai = [0; pi/2; 0; 0; pi/2; -pi/2];
di     = [d1; 0; 0; d4; d5; d6];

%% Configuration articulaire de reference
thetai = [deg2rad(-91.06);
          deg2rad(-111.79);
          deg2rad(-104.53);
          deg2rad(-55.59);
          deg2rad(90.79);
          deg2rad(-1.16)];

%% Chargement du robot UR5 dans Robotics System Toolbox
robot = loadrobot("universalUR5", "DataFormat", "row");
robot.Gravity = [0 0 -9.81]; % par defaut sur MATLAB il n'existe pas de la gravite


%% =========================================================
% PARTIE 1.1 - MODELE CINEMATIQUE DIRECT
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 1.1 - MODELE CINEMATIQUE DIRECT\n');
fprintf('***************************************\n');

%% Calcul des matrices de transformation elementaires T_{i-1}-{i}
T = cell(6,1);
for i = 1:6
    T{i} = DH_Modified_Transform(alphai(i), ai(i), di(i), thetai(i));
    fprintf('\n--- T_%d-%d ---\n', i-1, i);
    disp(T{i});
end

%% Calcul des matrices de transformation cumulees T_0-i
T_cumul = cell(6,1);
T_cumul{1} = T{1};
for i = 2:6
    T_cumul{i} = T_cumul{i-1} * T{i};
end

%% Affichage des matrices cumulees
for i = 1:6
    fprintf('\n=== T_0-%d ===\n', i);
    disp(T_cumul{i});
end

%% Pose finale de l'effecteur
% T06 contient la pose finale de l'effecteur dans le repere de base DH
T06 = T_cumul{6};
position_06 = T06(1:3,4);
rotation_06 = T06(1:3,1:3);

fprintf('\n========================================\n');
fprintf('POSE DE L''EFFECTEUR T_0-6\n');
fprintf('========================================\n');
fprintf('Position (x, y, z) = [%f, %f, %f] m\n', ...
    position_06(1), position_06(2), position_06(3));

fprintf('Matrice de rotation :\n');
disp(rotation_06);

%% =========================================================
% PARTIE 1.2 - VERIFICATION DU MODELE CINEMATIQUE DIRECT PAR IK
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 1.2 - VERIFICATION PAR IK (UN POINT)\n');
fprintf('***************************************\n');

% Le principe est le suivant :
% 1) on calcule la pose T_0-6 a partir du modele cinematique direct ;
% 2) on utilise cette pose comme cible dans l'inverse kinematics ;
% 3) on verifie que les angles retrouves sont coherents avec les angles
%    initiaux ayant servi au modele direct.

%% Chargement de la cinematique inverse
ik = inverseKinematics('RigidBodyTree', robot);

%% Pose cible issue du modele direct
T_target = T06;

%% Choix des poids de l'IK
% Ces poids pondèrent l'erreur sur les composantes de la pose cible
% dans la résolution IK. Ici, certaines composantes sont moins pondérées
% que les autres afin d'assouplir légèrement la résolution
weights = [0.25, 0.25, 0.25, 1, 1, 1];

%% Choix d'une estimation initiale
% On choisit une estimation proche de la solution voulue afin de retrouver
% la meme branche de solution.
initialGuess = thetai' + deg2rad([1 -1 1 -1 1 -1]);

%% Resolution de l'IK pour la pose cible
q_verification = ik('tool0', T_target, weights, initialGuess);

%% Affichage des resultats
fprintf('\n========================================\n');
fprintf('VERIFICATION PAR IK POUR UNE POSE DONNEE\n');
fprintf('========================================\n');
fprintf('Angles originaux (deg)  : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', rad2deg(thetai.'));
fprintf('Angles IK trouves (deg) : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', rad2deg(q_verification));
fprintf('Erreur (deg)            : [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', rad2deg(q_verification - thetai.'));
% Resultat : T06 est exprimee dans notre repere de base DH
% Le modele MATLAB utilise un repere de base different, ce qui explique
% ensuite un decalage de 180 deg sur q1 dans la comparaison IK


% Comparaison des poses pour mettre en evidence la difference de repere
% entre notre modele DH et le modele URDF charge par MATLAB.
fprintf('\n========================================\n');
fprintf('MODELE DIRECTE DH vs MODELE DIRECTE MATLAB :\n');
fprintf('========================================\n');
T_toolbox = getTransform(robot, thetai.', 'tool0');
disp('Transformation toolbox T_base_tool0 :');
disp(T_toolbox);
disp('Notre transformation T06 :');
disp(T06);



%% =========================================================
% PARTIE 1.3 - VERIFICATION PAR SUIVI DE TRAJECTOIRE EN POSITION
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 1.3 - SUIVI DE TRAJECTOIRE EN POSITION\n');
fprintf('***************************************\n');

% Le principe est de partir de la pose initiale obtenue par le modele
% direct, puis d'imposer une trajectoire cartesienne composee de deux
% segments : une ligne droite suivant -X, suivie d'un arc de cercle de
% rayon R dans le plan XY. A chaque point de la trajectoire, l'IK est
% utilisee pour retrouver la configuration articulaire correspondante

%% Parametres de la trajectoire cartesienne
% pour la ligne droite
distance = -0.30;   % deplacement suivant X (m)
N_ligne = 1000;            % nombre de points

% pour l'arc après la ligne droite
R = 0.25;
arc_angle = pi;
N_arc = 2000;
N = N_ligne + N_arc;

t_ligne = linspace(0,1,N_ligne);
t_arc  = linspace(0, arc_angle, N_arc);

%% Position initiale et orientation constante
pos_init = T06(1:3,4);
R_const  = T06(1:3,1:3);

%% Calcul de la trajectoire cartiesienne
% On fait une decomposition de la trajectoire en N nombre des points.
% Après, nous allons calculer les angles articulaires du robot pour chaque
% position de la trajectoire cartesienne
positions = zeros(3,N);

%% Trajectoire rectiligne
for k = 1:N_ligne
    positions(:,k) = pos_init + [distance*t_ligne(k); 0; 0];
end

%% Trajectoire circulaire (arc dans le plan XY)
% le centre du cercle est place a une distance R en +Y par rapport au dernier 
% point de la ligne droite. Ainsi l'equation parametrique utilise -sin pour 
% continuer dans la direction -X et -cos pour assurer la continuite avec le 
% point final de la ligne droite
centre = positions(:, N_ligne) + [0; R; 0];

for k = 1:N_arc
    positions(:, N_ligne + k) = centre + [-R*sin(t_arc(k)); -R*cos(t_arc(k)); 0];
end

%% Calcul des configurations articulaires par IK
% q_traj contient la trajectoire articulaire obtenue a partir de la
% trajectoire cartesienne par resolution successive de l'IK.
q_traj = zeros(N,6);
currentGuess = thetai';

fprintf('\n========================================\n');
fprintf('CALCUL DE LA TRAJECTOIRE CARTESIENNE PAR IK\n');
fprintf('========================================\n');

for k = 1:N
    T_desired = [R_const, positions(:,k); 0 0 0 1];
    q_sol = ik('tool0', T_desired, weights, currentGuess);
    q_traj(k,:) = q_sol;
    currentGuess = q_sol;
end

fprintf('Trajectoire calculee avec succes dans l''espace articulaire. Vous pouvez regarder la figure 1 pour l''animation du robot\n');

%% Affichage de l'animation
N_frames = 200; % nb de frames total pour l'animation
id_frame = round(linspace(1, N, N_frames));
figure('Name', 'Animation du suivi de trajectoire');

% Affichage de la pose initial du robot (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16)
ax = show(robot, q_traj(1,:), ...
    'Visuals', 'on', ...
    'Frames', 'off', ...
    'PreservePlot', false, ...
    'FastUpdate', true);
hold on;

% Trace de la trajectoire cartesienne
plot3(positions(1,:), positions(2,:), positions(3,:), 'r-', 'LineWidth', 2);

title('Suivi de trajectoire rectiligne en X');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view([-45 30]);
grid on;

%% Animation du mouvement du robot
for i = 1:length(id_frame)
    show(robot, q_traj(id_frame(i),:), ...
        'Parent', ax, ...
        'Visuals', 'on', ...
        'Frames', 'off', ...
        'PreservePlot', false, ...
        'FastUpdate', true);
    drawnow;
    pause(0.01);
end
hold off;


%% =========================================================
% PARTIE 2.1 - MODELE CINEMATIQUE DIFFERENTIEL
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 2.1 - MODELE CINEMATIQUE DIFFERENTIEL ANALYTIQUE\n');
fprintf('***************************************\n');

% Le modele cinematique differentiel relie les vitesses articulaires qdot
% a la vitesse de l'effecteur via le jacobien geometrique :
%            xdot = J(q) * qdot
%
% Dans cette section, on construit le jacobien analytique de position Jv
% par derivation symbolique du vecteur position de l'effecteur p(q)
% par rapport a chaque variable articulaire :
%            Jv(i,j) = dp_i / dq_j
%
% Cette approche requiert l'expression symbolique complete de la
% cinematique directe T_0-6(q), a partir de laquelle on extrait le
% vecteur position p = [px; py; pz] et on calcule les derivees partielles

%% Variables symboliques articulaires
syms sq1 sq2 sq3 sq4 sq5 sq6 real
syms sq23 sq234 real

q_sym = [sq1 sq2 sq3 sq4 sq5 sq6];

%% Parametres DH modifies symboliques
ai_sym     = [0; 0; a2; a3; 0; 0];
alphai_sym = [sym(0); sym(pi)/2; sym(0); sym(0); sym(pi)/2; -sym(pi)/2];
di_sym     = [d1; 0; 0; d4; d5; d6];
thetai_sym = [sq1; sq2; sq3; sq4; sq5; sq6];

%% Calcul symbolique des matrices de transformation elementaires
T_sym = cell(6,1);
for i = 1:6
    T_sym{i} = simplify(DH_Modified_Transform(alphai_sym(i), ai_sym(i), di_sym(i), thetai_sym(i)));
end

%% Calcul symbolique des matrices de transformation cumulees
T_cumul_sym = cell(6,1);
T_cumul_sym{1} = T_sym{1};
for i = 2:6
    T_cumul_sym{i} = simplify(T_cumul_sym{i-1} * T_sym{i}, 'Steps', 50);
end

%% Extraction du vecteur position symbolique de l'effecteur
T06_sym = T_cumul_sym{6};
px_sym = T06_sym(1,4);
py_sym = T06_sym(2,4);
pz_sym = T06_sym(3,4);

%% Construction du jacobien analytique de position Jv = dP/dq
% Chaque element Jv(i,j) correspond a la derivee partielle de la i-eme
% composante de la position par rapport a la j-eme variable articulaire
Jv_sym = sym(zeros(3,6));
for j = 1:6
    Jv_sym(1,j) = diff(px_sym, q_sym(j));
    Jv_sym(2,j) = diff(py_sym, q_sym(j));
    Jv_sym(3,j) = diff(pz_sym, q_sym(j));
end
Jv_sym = simplify(Jv_sym, 'Steps', 50);

%% Simplification trigonometrique pour l'affichage
% On substitue les produits trigonometriques par les identites de somme
% (ex. cos(q2)*cos(q3) - sin(q2)*sin(q3) = cos(q2+q3)) afin d'obtenir
% des expressions plus compactes et lisibles. On introduit les variables
% symboliques q23 = q2+q3 et q234 = q2+q3+q4
Jv_sym_display = Jv_sym;

Jv_sym_display = subs(Jv_sym_display, cos(sq2)*cos(sq3) - sin(sq2)*sin(sq3), cos(sq2+sq3));
Jv_sym_display = subs(Jv_sym_display, cos(sq2)*sin(sq3) + sin(sq2)*cos(sq3), sin(sq2+sq3));
Jv_sym_display = subs(Jv_sym_display, sin(sq2)*cos(sq3) + cos(sq2)*sin(sq3), sin(sq2+sq3));
Jv_sym_display = simplify(Jv_sym_display, 'Steps', 50);
Jv_sym_display = subs(Jv_sym_display, sq2+sq3, sq23);

Jv_sym_display = subs(Jv_sym_display, cos(sq23)*cos(sq4) - sin(sq23)*sin(sq4), cos(sq23+sq4));
Jv_sym_display = subs(Jv_sym_display, cos(sq23)*sin(sq4) + sin(sq23)*cos(sq4), sin(sq23+sq4));
Jv_sym_display = subs(Jv_sym_display, sin(sq23)*cos(sq4) + cos(sq23)*sin(sq4), sin(sq23+sq4));
Jv_sym_display = simplify(Jv_sym_display, 'Steps', 50);
Jv_sym_display = subs(Jv_sym_display, sq23+sq4, sq234);

%% Affichage symbolique du jacobien de position
fprintf('\n========================================\n');
fprintf('JACOBIEN ANALYTIQUE DE POSITION Jv(q)\n');
fprintf('========================================\n');
for j = 1:6
    fprintf('\n--- Colonne %d de Jv ---\n', j);

    for row = 1:3
        expr = Jv_sym_display(row, j);

        % coefficients rationnels en decimaux
        expr_vpa = vpa(expr, 6);
        disp(expr_vpa);
    end
end

% Remarque : la derniere colonne de Jv est [0; 0; 0]. Cela s'explique par
% le fait que l'articulation 6 correspond a une rotation autour de l'axe
% z_5 qui passe par l'origine du repere 6 (le point de l'effecteur). Autrement dit, 
% une rotation de la derniere articulation ne modifie que l'orientation de l'outil sans
% deplacer son origine. Ainsi elle n'a aucune contribution a la vitesse lineaire.

%% =========================================================
% PARTIE 2.2 - APPLICATION NUMERIQUE DU JACOBIEN ANALYTIQUE
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 2.2 - APPLICATION NUMERIQUE DU JACOBIEN ANALYTIQUE\n');
fprintf('***************************************\n');

% On evalue le jacobien symbolique Jv(q) a la configuration articulaire
% de reference en substituant les valeurs numeriques de thetai dans
% l'expression symbolique. Le resultat Jv_num servira de reference pour
% la validation croisee avec la methode geometrique (partie 2.3 ci-dessous)

%% Evaluation numerique de Jv a la configuration de reference
Jv_num = double(subs(Jv_sym, q_sym, thetai.'));

fprintf('\n========================================\n');
fprintf('JACOBIEN DE POSITION NUMERIQUE Jv(q) pour la pose (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16)\n');
fprintf('========================================\n');
disp(Jv_num);

%% =========================================================
% PARTIE 2.3 - VALIDATION : JACOBIEN ANALYTIQUE vs GEOMETRIQUE
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 2.3 - VALIDATION : JACOBIEN ANALYTIQUE vs GEOMETRIQUE\n');
fprintf('***************************************\n');

% On dispose maintenant de deux methodes independantes pour calculer la
% jacobienne de position de l'effecteur :
%
%   1) Methode analytique (parties 2.1 et 2.2) :
%      Jv_num = dP/dq, obtenue par derivation symbolique puis evaluation
%      numerique. Cette approche est exacte mais couteuse en calcul car
%      elle necessite le developpement symbolique complet de T_0-6(q).
%
%   2) Methode geometrique (ci-dessous) :
%      Selon le cours, pour une articulation rotoide i, la contribution a la vitesse
%      lineaire de l'effecteur est donnee par :
%           Jv_i = z_{i-1} x (p_e - p_{i-1})
%
%      ou z_{i-1} est l'axe de rotation de l'articulation i exprime dans
%      la base, et (p_e - p_{i-1}) est le bras de levier entre l'origine
%      du repere i-1 et l'effecteur. Cette methode est plus efficace car
%      elle ne requiert que les matrices de transformation cumulees, sans
%      aucune derivation symbolique.
%
% L'objectif de cette section est donc de verifier que les deux methodes
% produisent un resultat identique, validant ainsi notre modele

% Les axes z_i exprimes dans la base du robot
z1 = T_cumul{1}(1:3,3);
z2 = T_cumul{2}(1:3,3);
z3 = T_cumul{3}(1:3,3);
z4 = T_cumul{4}(1:3,3);
z5 = T_cumul{5}(1:3,3);
z6 = T_cumul{6}(1:3,3);

% Pour la jacobienne lineaire Jv, les origines des reperes exprimees 
% dans la base du robot
p1 = T_cumul{1}(1:3,4);
p2 = T_cumul{2}(1:3,4);
p3 = T_cumul{3}(1:3,4);
p4 = T_cumul{4}(1:3,4);
p5 = T_cumul{5}(1:3,4);
p6 = T_cumul{6}(1:3,4);

% Position de l'effecteur
pe = p6;


% Construction de la jacobienne lineaire par la methode geometrique :
% chaque colonne j correspond a z_j x (p_e - p_j)
Jv = [cross(z1, pe - p1), ...
      cross(z2, pe - p2), ...
      cross(z3, pe - p3), ...
      cross(z4, pe - p4), ...
      cross(z5, pe - p5), ...
      cross(z6, pe - p6)];

%% Comparaison des deux methodes
fprintf('Erreur maximale absolue : %e\n', max(abs(Jv - Jv_num), [], 'all'));

% Resultat : La difference entre le jacobien analytique (obtenu par
% derivation symbolique) et le jacobien geometrique (obtenu par produits
% vectoriels) est numeriquement nulle. Cela confirme l'equivalence des
% deux approches et valide notre modele cinematique differentiel.
%
% En fait, la methode geometrique presente l'avantage d'etre moins couteuse en
% calcul car elle n'exige pas le developpement symbolique complet de la
% cinematique directe, mais seulement les matrices de transformation
% cumulees T_0-i deja disponibles



%% =========================================================
% PARTIE 2.4 - CONSTRUCTION DE LA JACOBIENNE GEOMETRIQUE COMPLETE Jg = [Jw ; Jv]
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 2.4 - CONSTRUCTION DE LA JACOBIENNE GEOMETRIQUE COMPLETE Jg = [Jw ; Jv]\n');
fprintf('***************************************\n');

% La jacobienne geometrique complete Jg relie le torseur cinematique
% de l'effecteur (vitesse angulaire + vitesse lineaire) aux vitesses
% articulaires :
%       [omega]     [Jw]
%       [  v  ]  =  [Jv] * qdot
%
% Pour une articulation rotoide i :
%   - La colonne de Jw est simplement l'axe de rotation z_{i-1}
%   - La colonne de Jv est z_{i-1} x (p_e - p_{i-1})  (deja calculee dans la partie 2.3)

% Jacobienne angulaire Jw : chaque colonne est l'axe z_{i-1}
Jw = [z1 z2 z3 z4 z5 z6];

fprintf('\n========================================\n');
fprintf('JACOBIEN ANGULAIRE NUMERIQUE Jw(q) pour la pose (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16)\n');
fprintf('========================================\n');
disp(Jw);


% Jacobienne geometrique complete
Jg = [Jw; Jv];

fprintf('\n========================================\n');
fprintf('JACOBIENNE GEOMETRIQUE COMPLETE Jg = [Jw ; Jv]\n');
fprintf('========================================\n');
disp(Jg);



%% =========================================================
% PARTIE 2.5 - COMPARAISON AVEC geometricJacobian MATLAB
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 2.5 - COMPARAISON AVEC geometricJacobian MATLAB\n');
fprintf('***************************************\n');

% La fonction geometricJacobian de la Robotics System Toolbox retourne
% une matrice 6xn exprimee dans le repere de base du modele URDF.
% La convention de MATLAB place la partie angulaire (Jw) dans les
% 3 premieres lignes et la partie lineaire (Jv) dans les 3 dernieres :
%       J_toolbox = [Jw_matlab ; Jv_matlab]


J_toolbox = geometricJacobian(robot, thetai.', 'tool0');

fprintf('\n========================================\n');
fprintf('COMPARAISON AVEC geometricJacobian MATLAB\n');
fprintf('========================================\n');

fprintf('Jacobien geometrique Toolbox :\n');
disp(J_toolbox);

fprintf('Notre partie angulaire Jw :\n');
disp(Jg(1:3,:));

fprintf('Partie angulaire Toolbox (lignes 1 a 3) :\n');
disp(J_toolbox(1:3,:));

fprintf('Notre partie lineaire Jv :\n');
disp(Jg(4:6,:));

fprintf('Partie lineaire Toolbox (lignes 4 a 6) :\n');
disp(J_toolbox(4:6,:));

% Resultat : Les parties angulaire et lineaire de notre jacobienne
% geometrique correspondent a celles de MATLAB. Les eventuelles
% differences de signe observees s'expliquent par la difference de
% convention de repere de base entre notre parametrisation DH modifiee
% et le modele URDF utilise par la Robotics System Toolbox (cf. le
% decalage de 180 deg sur q1 observe en partie 1.2).

%% Erreurs numeriques de comparaison
err_Jw = Jw - J_toolbox(1:3,:);
err_Jv = Jv - J_toolbox(4:6,:);

fprintf('\n========================================\n');
fprintf('ERREURS DE COMPARAISON\n');
fprintf('========================================\n');
fprintf('Erreur angulaire max : %e\n', max(abs(err_Jw), [], 'all'));
fprintf('Erreur lineaire max  : %e\n', max(abs(err_Jv), [], 'all'));


%% =========================================================
% PARTIE 3.1 - MODELE DYNAMIQUE
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 3 - IMPLEMENTATION DU MODELE DYNAMIQUE\n');
fprintf('***************************************\n');

% Le modele dynamique articulaire s'ecrit sous la forme :
% tau = M(q)*qdd + h(q,qd) + g(q)
%
% avec :
% M(q)   : matrice de masse/inertie
% h(q,qd): terme des effets de vitesse (Coriolis + centrifuges)
% g(q)   : terme gravitaire
%
% Cette partie exploite directement les fonctions de la Robotics System Toolbox.


%% Pour la pose (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16) deg, on choisit des vitesses et accelerations articulaires arbitraires pour tester le modele dynamique
q_dyn = thetai.';
qd_dyn  = [0.2 0.1 -0.1 0.15 0.05 -0.08];
qdd_dyn = [0.5 0.2 -0.3 0.1 0.2 -0.1];

%% 1) Matrice de masse M(q)
M_dyn = massMatrix(robot, q_dyn);

%% 2) Terme gravitaire g(q)
g_dyn = gravityTorque(robot, q_dyn);

%% 3) Terme de vitesse
% MATLAB retourne les couples necessaires pour annuler les effets
% induits par les vitesses articulaires
vp_dyn = velocityProduct(robot, q_dyn, qd_dyn);

% Donc, dans l'ecriture classique du modele dynamique :
h_dyn = -vp_dyn;

%% 4) Recomposition du couple dynamique total
tau_model = M_dyn * qdd_dyn.' + h_dyn.' + g_dyn.';

%% 5) Couple total via la dynamique inverse MATLAB
tau_id = inverseDynamics(robot, q_dyn, qd_dyn, qdd_dyn);

%% 6) Comparaison
% La comparaison tau_model / tau_id nous permet de verifier la coherence de la
% decomposition M(q)*qdd + h(q,qd) + g(q) avec la dynamique inverse MATLAB
err_tau = tau_model.' - tau_id;

fprintf('\n========================================\n');
fprintf('RESULTATS DU MODELE DYNAMIQUE pour la pose (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16) :\n');
fprintf('========================================\n');

fprintf('Matrice de masse M(q) :\n');
disp(M_dyn);

fprintf('Terme gravitaire g(q) :\n');
disp(g_dyn);

fprintf('Terme de vitesse h(q,qd) :\n');
disp(h_dyn);

fprintf('Couple total reconstruit tau_model :\n');
disp(tau_model.');

fprintf('Couple total inverseDynamics tau_id :\n');
disp(tau_id);

fprintf('Erreur tau_model - tau_id :\n');
disp(err_tau);

fprintf('Erreur maximale absolue : %e\n', max(abs(err_tau)));





%% =========================================================
% MATRICE DE TRANSFORMATION DH MODIFIEE
% ==========================================================
function T = DH_Modified_Transform(alpha, a, d, theta)
% Cette fonction calcule la matrice de transformation homogene
% selon la convention DH modifiee de Craig :
% T = Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta)

    T = [cos(theta),              -sin(theta),             0,            a;
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),   -sin(alpha),  -sin(alpha)*d;
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d;
         0,                       0,                       0,            1];
end


%% =========================================================
% PARTIE 4 - CONSTRUCTION DE LA JACOBIENNE GEOMETRIQUE COMPLETE Jg = [Jw ; Jv]
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 4 - CONSTRUCTION DE LA JACOBIENNE GEOMETRIQUE COMPLETE Jg = [Jw ; Jv]\n');
fprintf('***************************************\n');