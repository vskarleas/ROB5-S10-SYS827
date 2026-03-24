function plot_commande_en_impendance(robot, t_vec, traj_full, x_hist, err_hist, ...
                      tau_hist, q_hist, phase_ids, ...
                      pos_pick, pos_place, N_total)
% PLOT_COMMANDE_EN_IMPENDANCE Affiche les resultats de la simulation de la
% commande en impedance pour la tache de pick & place.
%
% Cette fonction genere 5 figures :
%   1) Trajectoire 3D desiree vs reelle, coloree par phase
%   2) Suivi de trajectoire par composante (X, Y, Z) avec transitions
%   3) Erreur de suivi (norme par phase + composantes)
%   4) Couples articulaires
%   5) Animation du robot avec gripper, balle et trace
%
% Entrees :
%   robot      - Objet RigidBodyTree (avec gripper_tip)
%   t_vec      - Vecteur de temps (1 x N_total)
%   traj_full  - Trajectoire cartesienne desiree (3 x N_total)
%   x_hist     - Trajectoire cartesienne reelle (3 x N_total)
%   err_hist   - Erreur de position (3 x N_total)
%   tau_hist   - Couples articulaires (N_total x 6)
%   q_hist     - Configurations articulaires (N_total x 6)
%   phase_ids  - Identifiant de phase pour chaque point (1 x N_total)
%   pos_pick   - Position de pick (3 x 1)
%   pos_place  - Position de place (3 x 1)
%   N_total    - Nombre total de points

    %% Definitions communes
    couleurs_phase = [0.2 0.6 1.0;   % bleu   - descente pick
                      0.0 0.8 0.4;   % vert   - remontee pick
                      1.0 0.5 0.0;   % orange - transport
                      0.8 0.2 0.2;   % rouge  - descente place
                      0.6 0.2 0.8];  % violet - remontee place

    noms_phase = {'Descente pick', 'Remontee pick', 'Transport', ...
                  'Descente place', 'Remontee place'};

    %% Affichage des metriques dans la console
    err_final = norm(err_hist(:, end));
    fprintf('\n--- Resultats ---\n');
    fprintf('Erreur de position finale : %.4f mm\n', err_final * 1000);
    fprintf('Erreur maximale en cours de trajectoire : %.4f mm\n', ...
        max(vecnorm(err_hist)) * 1000);

    %% ===============================================
    % Figure 1 : Trajectoire 3D avec phases
    % ================================================
    figure('Name', 'Trajectoire 3D - Pick & Place');
    hold on;

    for ph = 1:5
        idx = (phase_ids == ph);
        plot3(traj_full(1,idx), traj_full(2,idx), traj_full(3,idx), ...
            '-', 'Color', couleurs_phase(ph,:), 'LineWidth', 2, ...
            'DisplayName', ['Desire - ' noms_phase{ph}]);
    end

    plot3(x_hist(1,:), x_hist(2,:), x_hist(3,:), ...
        'k--', 'LineWidth', 1.5, 'DisplayName', 'Trajectoire reelle');

    plot3(pos_pick(1), pos_pick(2), pos_pick(3), ...
        'v', 'MarkerSize', 14, 'MarkerFaceColor', [0 0.7 0], ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'Pick');
    plot3(pos_place(1), pos_place(2), pos_place(3), ...
        '^', 'MarkerSize', 14, 'MarkerFaceColor', [0.8 0 0], ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'Place');

    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Trajectoire Pick & Place - Commande en impedance');
    legend('Location', 'best');
    view([-40 25]); grid on; axis equal;
    hold off;

    %% ===============================================
    % Figure 2 : Suivi par composante avec phases
    % ================================================
    figure('Name', 'Suivi de trajectoire par composante');
    axes_labels = {'X', 'Y', 'Z'};

    transitions = zeros(1,4);
    transitions(1) = t_vec(find(phase_ids == 2, 1, 'first'));
    transitions(2) = t_vec(find(phase_ids == 3, 1, 'first'));
    transitions(3) = t_vec(find(phase_ids == 4, 1, 'first'));
    transitions(4) = t_vec(find(phase_ids == 5, 1, 'first'));

    for ax_i = 1:3
        subplot(3,1,ax_i);
        plot(t_vec, traj_full(ax_i,:)*1000, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_vec, x_hist(ax_i,:)*1000, 'r--', 'LineWidth', 1);

        for tr = 1:4
            xline(transitions(tr), 'k:', 'LineWidth', 0.8);
        end

        ylabel(sprintf('%s (mm)', axes_labels{ax_i}));
        if ax_i == 1
            title('Suivi de trajectoire cartesienne');
            legend('Desire', 'Reel', 'Location', 'best');
        end
        if ax_i == 3, xlabel('Temps (s)'); end
        grid on;
    end

    %% ===============================================
    % Figure 3 : Erreur de suivi
    % ================================================
    figure('Name', 'Erreur de suivi');

    subplot(2,1,1);
    hold on;
    for ph = 1:5
        idx = (phase_ids == ph);
        t_ph = t_vec(idx);
        e_ph = vecnorm(err_hist(:, idx)) * 1000;
        area(t_ph, e_ph, 'FaceColor', couleurs_phase(ph,:), ...
            'FaceAlpha', 0.4, 'EdgeColor', couleurs_phase(ph,:), ...
            'DisplayName', noms_phase{ph});
    end
    ylabel('||e|| (mm)');
    title('Norme de l''erreur de suivi par phase');
    legend('Location', 'best'); grid on;

    subplot(2,1,2);
    plot(t_vec, err_hist(1,:)*1000, 'r', ...
         t_vec, err_hist(2,:)*1000, 'g', ...
         t_vec, err_hist(3,:)*1000, 'b', 'LineWidth', 1);
    ylabel('Erreur (mm)'); xlabel('Temps (s)');
    legend('e_x', 'e_y', 'e_z'); grid on;
    title('Erreur de suivi par composante');

    %% ===============================================
    % Figure 4 : Couples articulaires
    % ================================================
    figure('Name', 'Couples articulaires');
    for j = 1:6
        subplot(3,2,j);
        plot(t_vec, tau_hist(:,j), 'LineWidth', 1);
        ylabel(sprintf('\\tau_%d (Nm)', j));
        if j >= 5, xlabel('Temps (s)'); end
        grid on;
    end
    sgtitle('Couples articulaires - Commande en impedance');

    %% ===============================================
    % Figure 5 : Animation avec gripper et balle
    % ================================================
    figure('Name', 'Animation Pick & Place', 'Position', [100 100 1000 700]);

    N_frames_anim = 300;
    idx_anim = round(linspace(1, N_total, N_frames_anim));

    ax_anim = show(robot, q_hist(1,:), ...
        'Visuals', 'on', 'Frames', 'off', ...
        'PreservePlot', false, 'FastUpdate', true);
    hold on;

    % Trajectoire desiree coloree par phase
    for ph = 1:5
        idx = (phase_ids == ph);
        plot3(traj_full(1,idx), traj_full(2,idx), traj_full(3,idx), ...
            '-', 'Color', couleurs_phase(ph,:), 'LineWidth', 2);
    end

    % Marqueurs pick et place
    plot3(pos_pick(1), pos_pick(2), pos_pick(3), ...
        'v', 'MarkerSize', 14, 'MarkerFaceColor', [0 0.7 0], 'MarkerEdgeColor', 'k');
    plot3(pos_place(1), pos_place(2), pos_place(3), ...
        '^', 'MarkerSize', 14, 'MarkerFaceColor', [0.8 0 0], 'MarkerEdgeColor', 'k');

    % Balle
    ball_radius = 0.02;
    [bsx, bsy, bsz] = sphere(15);
    h_ball = surf(ball_radius*bsx + pos_pick(1), ...
                  ball_radius*bsy + pos_pick(2), ...
                  ball_radius*bsz + pos_pick(3), ...
                  'FaceColor', [0.1 0.8 0.1], 'FaceAlpha', 0.9, 'EdgeColor', 'none');

    % Cible de depot (transparente)
    surf(ball_radius*bsx + pos_place(1), ...
         ball_radius*bsy + pos_place(2), ...
         ball_radius*bsz + pos_place(3), ...
         'FaceColor', [0.1 0.8 0.1], 'FaceAlpha', 0.15, ...
         'EdgeColor', [0 0.5 0], 'LineStyle', '--');

    % Gripper initial
    h_grip = draw_gripper(robot, q_hist(1,:), 0.12, true);

    % Trace en temps reel
    h_trace = plot3(nan, nan, nan, 'k--', 'LineWidth', 1);
    trace_x = []; trace_y = []; trace_z = [];

    title('Animation Pick & Place');
    view([-40 25]); grid on;
    light('Position', [1 1 1]);

    % Logique de saisie de la balle
    ball_grasped = false;
    ball_deposited = false;
    ball_pos = pos_pick;

    for i = 1:length(idx_anim)
        k = idx_anim(i);

        % Affichage du robot
        show(robot, q_hist(k,:), 'Parent', ax_anim, ...
            'Visuals', 'on', 'Frames', 'off', ...
            'PreservePlot', false, 'FastUpdate', true);

        % Mise a jour du gripper
        delete(h_grip);
        is_open = (phase_ids(k) == 1) || (phase_ids(k) == 5);
        h_grip = draw_gripper(robot, q_hist(k,:), 0.12, is_open);

        % Logique de la balle
        if ~ball_grasped && phase_ids(k) >= 2
            ball_grasped = true;
        end

        if ball_grasped && ~ball_deposited
            ball_pos = x_hist(:, k);
        end

        if ball_grasped && ~ball_deposited && phase_ids(k) >= 5
            ball_deposited = true;
            ball_pos = pos_place;
        end

        % Mise a jour de la balle
        set(h_ball, 'XData', ball_radius*bsx + ball_pos(1), ...
                    'YData', ball_radius*bsy + ball_pos(2), ...
                    'ZData', ball_radius*bsz + ball_pos(3));

        % Mise a jour de la trace
        trace_x(end+1) = x_hist(1,k); %#ok<AGROW>
        trace_y(end+1) = x_hist(2,k); %#ok<AGROW>
        trace_z(end+1) = x_hist(3,k); %#ok<AGROW>
        set(h_trace, 'XData', trace_x, 'YData', trace_y, 'ZData', trace_z);

        % Titre avec phase courante
        title(sprintf('t = %.2f s - Phase %d : %s', ...
            t_vec(k), phase_ids(k), noms_phase{phase_ids(k)}));

        drawnow;
        pause(0.02);
    end

    hold off;
    fprintf('Animation terminee.\n');
end