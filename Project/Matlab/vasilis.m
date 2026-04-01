%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% 4.2 - CONSTRUCTION DE LA TRAJECTOIRE PICK & PLACE
% % ==========================================================
% fprintf('\n***************************************\n');
% fprintf('PARTIE 4.2 - CONSTRUCTION DE LA TRAJECTOIRE PICK & PLACE\n');
% fprintf('***************************************\n');
% 
% 
% %% Parametres des mouvements verticaux
% hauteur_descente = 0.1;
% fprintf('Hauteur de descente pour la tache Pick & Place : %.4f m\n', hauteur_descente);
% 
% N_descente = 300; % nb des points pour la decomposition de la trajectoire
% N_remontee = 300;
% 
% %% La trajectoire complete se decompose en 5 segments :
% %   Segment 1 : On descend de hauteur_descente suivant -Z a partir de la position initiale
% %   Segment 2 : Remontee verticale apres la saisie
% %   Segment 3 : Trajectoire de transport (ligne droite + arc, cf. partie 1.3)
% %   Segment 4 : Descente verticale au point de place
% %   Segment 5 : Remontee verticale apres le depot
% 
% %% Segment 3 (compute first to get the reference positions)
% seg3 = zeros(3, N); % pour le N cf. partie 1.3
% for k = 1:N
%     T_k = getTransform(robot, q_traj(k,:), 'gripper_tip');
%     seg3(:, k) = T_k(1:3, 4);
% end
% 
% pos_avant_place = seg3(:, end);
% 
% %% Positions de reference pour les mouvements verticaux
% % Le point de depart est la position du gripper_tip pour la premiere
% % configuration de la trajectoire IK (q_traj(1,:)), pas thetai.
% pos_start = seg3(:, 1);   % debut du segment de transport
% pos_end   = seg3(:, end);  % fin du segment de transport
% 
% %% Segment 1 : descente verticale au point de pick
% t_s1 = linspace(0, 1, N_descente);
% seg1 = zeros(3, N_descente);
% for k = 1:N_descente
%     seg1(:, k) = pos_start + [0; 0; -hauteur_descente * t_s1(k)];
% end
% pos_pick = seg1(:, end);
% 
% %% Segment 2 : remontee verticale apres saisie
% t_s2 = linspace(0, 1, N_remontee);
% seg2 = zeros(3, N_remontee);
% for k = 1:N_remontee
%     seg2(:, k) = pos_pick + [0; 0; hauteur_descente * t_s2(k)];
% end
% 
% %% Segment 4 : descente verticale au point de place
% t_s4 = linspace(0, 1, N_descente);
% seg4 = zeros(3, N_descente);
% for k = 1:N_descente
%     seg4(:, k) = pos_end + [0; 0; -hauteur_descente * t_s4(k)];
% end
% pos_place = seg4(:, end);
% 
% %% Segment 5 : remontee verticale apres depot
% t_s5 = linspace(0, 1, N_remontee);
% seg5 = zeros(3, N_remontee);
% for k = 1:N_remontee
%     seg5(:, k) = pos_place + [0; 0; hauteur_descente * t_s5(k)];
% end
% 
% 
% 
% %% Assemblage de la trajectoire complete
% traj_full = [seg1, seg2(:,2:end), seg3(:,2:end), seg4(:,2:end), seg5(:,2:end)];
% N_total = size(traj_full, 2);
% 
% % Identification des phases pour chaque point
% phase_ids = [1*ones(1, N_descente), ...
%              2*ones(1, N_remontee-1), ...
%              3*ones(1, N-1), ... % N cf. partie 1.3
%              4*ones(1, N_descente-1), ...
%              5*ones(1, N_remontee-1)];
% 
% fprintf('Trajectoire assemblee : %d points en 5 segments\n', N_total);
% % Resultat : La trajectoire complete contient 4196 points. Le segment 3
% % comporte 3000 points et les segments 1, 2, 4, 5 comportent
% % 300 points chacun, soit 3000 + 4*300 = 4200 points au total. Or, lors
% % de l'assemblage, on retire le premier point des segments 2, 3, 4 et 5
% % pour eviter la duplication aux jonctions entre segments consecutifs.
% % Ainsi, on obtient donc 4200 - 4 = 4196 points uniques
% 
% 
% 
% %% 4.3 - CALCUL DES VITESSES ET ACCELERATIONS DESIREES
% % ==========================================================
% fprintf('\n***************************************\n');
% fprintf('PARTIE 4.3 - CALCUL DES VITESSES ET ACCELERATIONS DESIREES\n');
% fprintf('***************************************\n');
% 
% % On attribue une duree a chaque segment puis on calcule les vitesses et
% % accelerations desirees par differences finies centrees. Ces derivees
% % sont necessaires au controleur en impedance qui requiert xdot_d et xddot_d
% % a chaque pas de temps pour calculer l'acceleration de reference
% 
% % Durees de chaque segment
% duree_descente  = 1.5;
% duree_remontee  = 1.5;
% duree_transport = 5.0;
% duree_place_desc = 1.5;
% duree_place_rem  = 1.5;
% 
% T_sim = duree_descente + duree_remontee + duree_transport + duree_place_desc + duree_place_rem;
% 
% %% Vecteur de temps
% dt_global = T_sim / N_total;  % pas de temps moyen
% t_vec = linspace(0, T_sim, N_total);
% dt = t_vec(2) - t_vec(1);
% 
% %% Calcul des vitesses par differences finies centrees (position /dt)
% xd_traj = zeros(3, N_total);
% for k = 2:N_total-1
%     xd_traj(:,k) = (traj_full(:,k+1) - traj_full(:,k-1)) / (2*dt);
% end
% 
% %% Calcul des accelerations par differences finies centrees
% xdd_traj = zeros(3, N_total);
% for k = 2:N_total-1
%     xdd_traj(:,k) = (traj_full(:,k+1) - 2*traj_full(:,k) + traj_full(:,k-1)) / (dt^2);
% end
% 
% fprintf('Duree totale de la simulation : %.2f s (dt = %.4f s). Les vitesses et les accelerations sont calculees avec success\n', T_sim, dt);
% 
% 
% 
% %% 4.4 - PARAMETRES DE LA COMMANDE EN IMPEDANCE
% % ==========================================================
% fprintf('\n***************************************\n');
% fprintf('PARTIE 4.4 - PARAMETRES DE LA COMMANDE EN IMPEDANCE\n');
% fprintf('***************************************\n');
% 
% % Le comportement desire de l'effecteur est celui d'un systeme
% % masse-ressort-amortisseur :
% %   M_d * (xdd - xdd_d) + B_d * (xd - xd_d) + K_d * (x - x_d) = F_ext
% %
% % En l'absence de force externe (mouvement libre), le controleur
% % corrige les ecarts de position et de vitesse selon les gains K_d et B_d. Donc pas d'inertie appliquee
% 
% Md = 5.0  * eye(3); % inertie desiree (kg)
% Kd = 800  * eye(3); % raideur desiree (N/m)
% Bd = 80   * eye(3); % amortissement desire (Ns/m)
% 
% %% Verification du ratio d'amortissement (cours 6, page 36)
% % Pour un systeme du second ordre on a B_cr = 2*sqrt(K*M) % niveau d'amortissement optimal pour que le système oscillant revient à sa position d'équilibre
% B_critique = 2 * sqrt(Kd(1,1) * Md(1,1));
% taux_amort = Bd(1,1) / B_critique;
% 
% fprintf('Md = %.1f kg, Kd = %.1f N/m, Bd = %.1f Ns/m\n', Md(1,1), Kd(1,1), Bd(1,1));
% fprintf('Amortissement critique = %.1f Ns/m\n', B_critique);
% fprintf('Taux d''amortissement  = %.2f\n', taux_amort);
% 
% % Resultat : Le taux d'amortissement est proche à 0.6. 
% % Cela veut dire que le système est sous-amorti, donc on va observer des oscillations. 
% % Puis le système revient à l'équilibre le plus rapidement possible. Donc nous 
% % attendons que au debut du mouvement il y aura quelques oscillations mais plus 
% % tard le système sera stabilise
% 
% 
% 
% %% 4.5 - BOUCLE DE SIMULATION
% % ==========================================================
% fprintf('\n***************************************\n');
% fprintf('PARTIE 4.5 - BOUCLE DE SIMULATION\n');
% fprintf('***************************************\n');
% 
% % Le robot n'est pas en contact avec l'environnement dans cette simulation (W_e = 0), mais la loi
% % d'impedance definit la reaction dynamique de l'outil face a d'eventuels efforts externes.
% % En l'absence de force externe (W_e = 0), la trajectoire corrigee
% % coincide avec la trajectoire desiree et le controleur assure le suivi
% % de trajectoire par commande linearisante dans l'espace de la tache
% 
% 
% % Sauvegarder l'historique
% q_hist    = zeros(N_total, 6);
% qd_hist   = zeros(N_total, 6);
% x_hist    = zeros(3, N_total);
% err_hist  = zeros(3, N_total);
% tau_hist  = zeros(N_total, 6);
% 
% %% Initialisation
% % On initialise la simulation a la configuration articulaire correspondant
% % au debut du segment de transport (q_traj(1,:) de la partie 1.3), qui est
% % la configuration URDF equivalente a thetai.
% q  = q_traj(1,:);
% qd = zeros(1, 6);
% 
% 
% for k = 1:N_total % (cours 6, pages 13, 14 (diapos 25,26,27) et 28 (diapos 55,56))
%     %% Cinematique directe du gripper
%     T_current = getTransform(robot, q, 'gripper_tip');
%     x_current = T_current(1:3, 4);
% 
%     %% Jacobien geometrique au point du gripper
%     J_geo = geometricJacobian(robot, q, 'gripper_tip');
%     Jv_ctrl = J_geo(4:6, :);   % partie lineaire seulement (lignes 4-6) (cf. 2.5 partie ci-dessus)
% 
%     %% Vitesse cartesienne actuelle lineaire
%     xdot_current = Jv_ctrl * qd';
% 
%     %% Erreurs cartesiennes
%     e_pos = traj_full(:,k) - x_current;
%     e_vel = xd_traj(:,k)  - xdot_current;
% 
%     %% Force externe (pas de contact dans cette simulation)
%     F_ext = zeros(3, 1);
% 
%     %% Loi de commande en impedance
%     % Acceleration cartesienne de reference :
%     %   xdd_ref = xdd_d + Md^{-1} * (Kd * e_pos + Bd * e_vel - F_ext)
%     xdd_ref = xdd_traj(:,k) + Md \ (Kd * e_pos + Bd * e_vel - F_ext);
% 
%     %% Passage en espace articulaire par pseudo-inverse du jacobien
%     %   qdd_ref = Jv^+ * xdd_ref
% 
%     Jv_pinv = pinv(Jv_ctrl);
%     qdd_ref = Jv_pinv * xdd_ref;
% 
%     %% Couple de commande par dynamique inverse
%     %   tau = M(q)*qdd_ref + h(q,qd) + g(q)
%     M_q  = massMatrix(robot, q);
%     g_q  = gravityTorque(robot, q);
%     vp_q = velocityProduct(robot, q, qd);
% 
%     tau_cmd = (M_q * qdd_ref + (-vp_q)' + g_q')';
% 
%     %% Integration simple (Euler)
%     qdd = (M_q \ (tau_cmd' - (-vp_q)' - g_q'))';
%     qd  = qd + qdd * dt;
%     q   = q  + qd  * dt;
% 
%     %% Enregistrement
%     q_hist(k,:)   = q;
%     qd_hist(k,:)  = qd;
%     x_hist(:,k)   = x_current;
%     err_hist(:,k) = e_pos;
%     tau_hist(k,:)  = tau_cmd;
% 
%     %% Progression
%     if mod(k, round(N_total/10)) == 0
%         fprintf('  %.0f%%\n', 100*k/N_total);
%     end
% end
% 
% fprintf('Simulation terminee.\n');
% 
% 
% %% 4.6 - RESULTATS DE LA SIMULATION
% % ==========================================================
% fprintf('\n***************************************\n');
% fprintf('PARTIE 4.6 - RESULTATS DE LA SIMULATION\n');
% fprintf('***************************************\n');
% 
% plot_commande_en_impendance(robot, t_vec, traj_full, x_hist, err_hist, tau_hist, q_hist, phase_ids, pos_pick, pos_place, N_total);
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%