close all;
clear;
clc;

%% INITIALISATION GENERALE
% ==========================================================
fprintf('\n========================================================\n');
fprintf('MODELISATION ET INITIALISATION DU ROBOT UR5\n');
fprintf('========================================================\n');

% Chargement du robot, des parametres DH, du modele cinematique direct,
% et de la cinematique inverse via le fichier d'initialisation
[robot, thetai, ai, alphai, di, T, T_cumul, T06] = init_UR5();

% Extraction des parametres geometriques individuels pour les parties qui suivent
a2 = ai(3);
a3 = ai(4);
d1 = di(1);
d4 = di(4);
d5 = di(5);
d6 = di(6);


%% PARTIE 1.1 - MODELE CINEMATIQUE DIRECT
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 1.1 - MODELE CINEMATIQUE DIRECT\n');
fprintf('***************************************\n');

% La pose choisi pour laquelle on fait le calcul du modele cinematique
% direct est (-91.06, -111.79, -104.53, -55.59, 90.79, -1.16)

%% Affichage des matrices de transformation elementaires T_{i-1}-{i}
for i = 1:6
    fprintf('\n--- T_%d-%d ---\n', i-1, i);
    disp(T{i});
end

%% Affichage des matrices cumulees
for i = 1:6
    fprintf('\n=== T_0-%d ===\n', i);
    disp(T_cumul{i});
end

%% Pose finale de l'effecteur
% T06 contient la pose finale de l'effecteur dans le repere de base DH
position_06 = T06(1:3,4);
rotation_06 = T06(1:3,1:3);

fprintf('\n========================================\n');
fprintf('POSE DE L''EFFECTEUR T_0-6\n');
fprintf('========================================\n');
fprintf('Position (x, y, z) = [%f, %f, %f] m\n', ...
    position_06(1), position_06(2), position_06(3));

fprintf('Matrice de rotation :\n');
disp(rotation_06);


%% PARTIE 1.2 - VERIFICATION DU MODELE CINEMATIQUE DIRECT PAR IK
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



%% PARTIE 1.3 - VERIFICATION PAR SUIVI DE TRAJECTOIRE EN POSITION
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
plot_commande_en_position(robot, q_traj, positions, 200, 'Animation du suivi de trajectoire (controle en position)');



%% PARTIE 2.1 - MODELE CINEMATIQUE DIFFERENTIEL
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

%% Construction du jacobien analytique de position Jv = dP/dq (cours 3, page 13)
% Chaque element Jv(i,j) correspond a la derivee partielle de la i-eme
% composante de la position par rapport a la j-eme variable articulaire
Jv_sym = sym(zeros(3,6));
for j = 1:6
    Jv_sym(1,j) = diff(px_sym, q_sym(j));
    Jv_sym(2,j) = diff(py_sym, q_sym(j));
    Jv_sym(3,j) = diff(pz_sym, q_sym(j));
end
Jv_sym = simplify(Jv_sym, 'Steps', 50);

%% Simplification trigonometrique pour l'affichage (cours 1, page 157 Annexe A)
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


%% PARTIE 2.2 - APPLICATION NUMERIQUE DU JACOBIEN ANALYTIQUE
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


%% PARTIE 2.3 - VALIDATION : JACOBIEN ANALYTIQUE vs GEOMETRIQUE
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



%% PARTIE 2.4 - CONSTRUCTION DE LA JACOBIENNE GEOMETRIQUE COMPLETE Jg = [Jw ; Jv]
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 2.4 - CONSTRUCTION DE LA JACOBIENNE GEOMETRIQUE COMPLETE Jg = [Jw ; Jv]\n');
fprintf('***************************************\n');

% La jacobienne geometrique complete Jg relie le torseur cinematique (cours 3, page 24)
% de l'effecteur (vitesse angulaire + vitesse lineaire) aux vitesses
% articulaires.
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



%% PARTIE 2.5 - COMPARAISON AVEC geometricJacobian MATLAB
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


%% PARTIE 3 - MODELE DYNAMIQUE
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 3 - IMPLEMENTATION DU MODELE DYNAMIQUE\n');
fprintf('***************************************\n');

% Le modele dynamique articulaire s'ecrit sous la forme :
% tau = M(q)*qdd + h(q,qd) + g(q) (cours 5, page 34)
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


%% PARTIE 4 - COMMANDE EN IMPEDANCE POUR UNE TACHE PICK & PLACE
% ==========================================================

% On implemente maintenant une commande en impedance dans l'espace
% cartesien pour realiser une tache de pick and place. La trajectoire
% reprend celle de la partie 1.3 (ligne droite + arc de cercle) avec
% deux mouvements verticaux supplementaires, à savoir :
%   - Descente en Z au point de depart (pick) et temontee en Z apres saisie
%   - Descente en Z au point d'arrivee (place)et remontee en Z apres depot

% La loi de commande en impedance impose un comportement masse-ressort-
% amortisseur a l'effecteur (selon le cours) :
%   M_d * (xdd - xdd_d) + B_d * (xd - xd_d) + K_d * (x - x_d) = F_ext



%% 4.1 - AJOUT D'UN OUTIL (GRIPPER) AU ROBOT
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 4.1 - AJOUT D''UN OUTIL (GRIPPER) AU ROBOT\n');
fprintf('***************************************\n');

% On ajoute un corps rigide representant un gripper fixe a l'effecteur.
% Ce corps a une masse et une inertie qui seront prises en compte dans
% le modele dynamique lors du calcul des couples

% Affichage des parametres dynamiques du modele du robot avant le gripper
plot_details_dynamiques(robot); % (cours 5, page 44)

robot = gripper(robot, 0.12, 0.8);

% Affichage des parametres dynamiques du modele du robot apres le gripper
plot_details_dynamiques(robot);


%% 4.2 - CONSTRUCTION DE LA TRAJECTOIRE PICK & PLACE
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 4.2 - CONSTRUCTION DE LA TRAJECTOIRE PICK & PLACE\n');
fprintf('***************************************\n');


%% Parametres des mouvements verticaux
hauteur_descente = 0.1;
fprintf('Hauteur de descente pour la tache Pick & Place : %.4f m\n', hauteur_descente);

N_descente = 300; % nb des points pour la decomposition de la trajectoire
N_remontee = 300;

%% La trajectoire complete se decompose en 5 segments :
%   Segment 1 : On descend de hauteur_descente suivant -Z a partir de la position initiale
%   Segment 2 : Remontee verticale apres la saisie
%   Segment 3 : Trajectoire de transport (ligne droite + arc, cf. partie 1.3)
%   Segment 4 : Descente verticale au point de place
%   Segment 5 : Remontee verticale apres le depot

%% Segment 3 (compute first to get the reference positions)
seg3 = zeros(3, N); % pour le N cf. partie 1.3
for k = 1:N
    T_k = getTransform(robot, q_traj(k,:), 'gripper_tip');
    seg3(:, k) = T_k(1:3, 4);
end

pos_avant_place = seg3(:, end);

%% Positions de reference pour les mouvements verticaux
% Le point de depart est la position du gripper_tip pour la premiere
% configuration de la trajectoire IK (q_traj(1,:)), pas thetai.
pos_start = seg3(:, 1);   % debut du segment de transport
pos_end   = seg3(:, end);  % fin du segment de transport

%% Segment 1 : descente verticale au point de pick
t_s1 = linspace(0, 1, N_descente);
seg1 = zeros(3, N_descente);
for k = 1:N_descente
    seg1(:, k) = pos_start + [0; 0; -hauteur_descente * t_s1(k)];
end
pos_pick = seg1(:, end);

%% Segment 2 : remontee verticale apres saisie
t_s2 = linspace(0, 1, N_remontee);
seg2 = zeros(3, N_remontee);
for k = 1:N_remontee
    seg2(:, k) = pos_pick + [0; 0; hauteur_descente * t_s2(k)];
end

%% Segment 4 : descente verticale au point de place
t_s4 = linspace(0, 1, N_descente);
seg4 = zeros(3, N_descente);
for k = 1:N_descente
    seg4(:, k) = pos_end + [0; 0; -hauteur_descente * t_s4(k)];
end
pos_place = seg4(:, end);

%% Segment 5 : remontee verticale apres depot
t_s5 = linspace(0, 1, N_remontee);
seg5 = zeros(3, N_remontee);
for k = 1:N_remontee
    seg5(:, k) = pos_place + [0; 0; hauteur_descente * t_s5(k)];
end



%% Assemblage de la trajectoire complete
traj_full = [seg1, seg2(:,2:end), seg3(:,2:end), seg4(:,2:end), seg5(:,2:end)];
N_total = size(traj_full, 2);

% Identification des phases pour chaque point
phase_ids = [1*ones(1, N_descente), ...
             2*ones(1, N_remontee-1), ...
             3*ones(1, N-1), ... % N cf. partie 1.3
             4*ones(1, N_descente-1), ...
             5*ones(1, N_remontee-1)];

fprintf('Trajectoire assemblee : %d points en 5 segments\n', N_total);
% Resultat : La trajectoire complete contient 4196 points. Le segment 3
% comporte 3000 points et les segments 1, 2, 4, 5 comportent
% 300 points chacun, soit 3000 + 4*300 = 4200 points au total. Or, lors
% de l'assemblage, on retire le premier point des segments 2, 3, 4 et 5
% pour eviter la duplication aux jonctions entre segments consecutifs.
% Ainsi, on obtient donc 4200 - 4 = 4196 points uniques



%% 4.3 - CALCUL DES VITESSES ET ACCELERATIONS DESIREES
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 4.3 - CALCUL DES VITESSES ET ACCELERATIONS DESIREES\n');
fprintf('***************************************\n');

% On attribue une duree a chaque segment puis on calcule les vitesses et
% accelerations desirees par differences finies centrees. Ces derivees
% sont necessaires au controleur en impedance qui requiert xdot_d et xddot_d
% a chaque pas de temps pour calculer l'acceleration de reference

% Durees de chaque segment
duree_descente  = 1.5;
duree_remontee  = 1.5;
duree_transport = 5.0;
duree_place_desc = 1.5;
duree_place_rem  = 1.5;

T_sim = duree_descente + duree_remontee + duree_transport + duree_place_desc + duree_place_rem;

%% Vecteur de temps
dt_global = T_sim / N_total;  % pas de temps moyen
t_vec = linspace(0, T_sim, N_total);
dt = t_vec(2) - t_vec(1);

%% Calcul des vitesses par differences finies centrees (position /dt)
xd_traj = zeros(3, N_total);
for k = 2:N_total-1
    xd_traj(:,k) = (traj_full(:,k+1) - traj_full(:,k-1)) / (2*dt);
end

%% Calcul des accelerations par differences finies centrees
xdd_traj = zeros(3, N_total);
for k = 2:N_total-1
    xdd_traj(:,k) = (traj_full(:,k+1) - 2*traj_full(:,k) + traj_full(:,k-1)) / (dt^2);
end

fprintf('Duree totale de la simulation : %.2f s (dt = %.4f s). Les vitesses et les accelerations sont calculees avec success\n', T_sim, dt);



%% 4.4 - PARAMETRES DE LA COMMANDE EN IMPEDANCE
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 4.4 - PARAMETRES DE LA COMMANDE EN IMPEDANCE\n');
fprintf('***************************************\n');

% Le comportement desire de l'effecteur est celui d'un systeme
% masse-ressort-amortisseur :
%   M_d * (xdd - xdd_d) + B_d * (xd - xd_d) + K_d * (x - x_d) = F_ext
%
% En l'absence de force externe (mouvement libre), le controleur
% corrige les ecarts de position et de vitesse selon les gains K_d et B_d. Donc pas d'inertie appliquee

Md = 5.0  * eye(3); % inertie desiree (kg)
Kd = 800  * eye(3); % raideur desiree (N/m)
Bd = 80   * eye(3); % amortissement desire (Ns/m)

%% Verification du ratio d'amortissement (cours 6, page 36)
% Pour un systeme du second ordre on a B_cr = 2*sqrt(K*M) % niveau d'amortissement optimal pour que le système oscillant revient à sa position d'équilibre
B_critique = 2 * sqrt(Kd(1,1) * Md(1,1));
taux_amort = Bd(1,1) / B_critique;

fprintf('Md = %.1f kg, Kd = %.1f N/m, Bd = %.1f Ns/m\n', Md(1,1), Kd(1,1), Bd(1,1));
fprintf('Amortissement critique = %.1f Ns/m\n', B_critique);
fprintf('Taux d''amortissement  = %.2f\n', taux_amort);

% Resultat : Le taux d'amortissement est proche à 0.6. 
% Cela veut dire que le système est sous-amorti, donc on va observer des oscillations. 
% Puis le système revient à l'équilibre le plus rapidement possible. Donc nous 
% attendons que au debut du mouvement il y aura quelques oscillations mais plus 
% tard le système sera stabilise



%% 4.5 - BOUCLE DE SIMULATION
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 4.5 - BOUCLE DE SIMULATION\n');
fprintf('***************************************\n');

% Le robot n'est pas en contact avec l'environnement dans cette simulation (W_e = 0), mais la loi
% d'impedance definit la reaction dynamique de l'outil face a d'eventuels efforts externes.
% En l'absence de force externe (W_e = 0), la trajectoire corrigee
% coincide avec la trajectoire desiree et le controleur assure le suivi
% de trajectoire par commande linearisante dans l'espace de la tache


% Sauvegarder l'historique
q_hist    = zeros(N_total, 6);
qd_hist   = zeros(N_total, 6);
x_hist    = zeros(3, N_total);
err_hist  = zeros(3, N_total);
tau_hist  = zeros(N_total, 6);

%% Initialisation
% On initialise la simulation a la configuration articulaire correspondant
% au debut du segment de transport (q_traj(1,:) de la partie 1.3), qui est
% la configuration URDF equivalente a thetai.
q  = q_traj(1,:);
qd = zeros(1, 6);


for k = 1:N_total % (cours 6, pages 13, 14 (diapos 25,26,27) et 28 (diapos 55,56))
    %% Cinematique directe du gripper
    T_current = getTransform(robot, q, 'gripper_tip');
    x_current = T_current(1:3, 4);

    %% Jacobien geometrique au point du gripper
    J_geo = geometricJacobian(robot, q, 'gripper_tip');
    Jv_ctrl = J_geo(4:6, :);   % partie lineaire seulement (lignes 4-6) (cf. 2.5 partie ci-dessus)

    %% Vitesse cartesienne actuelle lineaire
    xdot_current = Jv_ctrl * qd';

    %% Erreurs cartesiennes
    e_pos = traj_full(:,k) - x_current;
    e_vel = xd_traj(:,k)  - xdot_current;

    %% Force externe (pas de contact dans cette simulation)
    F_ext = zeros(3, 1);

    %% Loi de commande en impedance
    % Acceleration cartesienne de reference :
    %   xdd_ref = xdd_d + Md^{-1} * (Kd * e_pos + Bd * e_vel - F_ext)
    xdd_ref = xdd_traj(:,k) + Md \ (Kd * e_pos + Bd * e_vel - F_ext);

    %% Passage en espace articulaire par pseudo-inverse du jacobien
    %   qdd_ref = Jv^+ * xdd_ref
    
    Jv_pinv = pinv(Jv_ctrl);
    qdd_ref = Jv_pinv * xdd_ref;

    %% Couple de commande par dynamique inverse
    %   tau = M(q)*qdd_ref + h(q,qd) + g(q)
    M_q  = massMatrix(robot, q);
    g_q  = gravityTorque(robot, q);
    vp_q = velocityProduct(robot, q, qd);

    tau_cmd = (M_q * qdd_ref + (-vp_q)' + g_q')';

    %% Integration simple (Euler)
    qdd = (M_q \ (tau_cmd' - (-vp_q)' - g_q'))';
    qd  = qd + qdd * dt;
    q   = q  + qd  * dt;

    %% Enregistrement
    q_hist(k,:)   = q;
    qd_hist(k,:)  = qd;
    x_hist(:,k)   = x_current;
    err_hist(:,k) = e_pos;
    tau_hist(k,:)  = tau_cmd;

    %% Progression
    if mod(k, round(N_total/10)) == 0
        fprintf('  %.0f%%\n', 100*k/N_total);
    end
end

fprintf('Simulation terminee.\n');


%% 4.6 - RESULTATS DE LA SIMULATION
% ==========================================================
fprintf('\n***************************************\n');
fprintf('PARTIE 4.6 - RESULTATS DE LA SIMULATION\n');
fprintf('***************************************\n');

plot_commande_en_impendance(robot, t_vec, traj_full, x_hist, err_hist, tau_hist, q_hist, phase_ids, pos_pick, pos_place, N_total);