function plot_suivi_dynamique_cercle(robot, t_vec, x_d, x_hist, err_hist, tau_hist, q_hist, gripper_length, N_frames)
% plot_suivi_dynamique_cercle affiche les resultats du suivi dynamique
% d'une trajectoire circulaire :
%   1) trajectoire 3D desiree / reelle
%   2) evolution temporelle des positions cartesiennes
%   3) erreur de suivi
%   4) couples articulaires
%   5) animation du robot avec le gripper
%
% Entrees :
%   robot          : Objet RigidBodyTree avec gripper_tip
%   t_vec          : Vecteur temps (1xN)
%   x_d            : Trajectoire desiree (3xN)
%   x_hist         : Trajectoire reelle (3xN)
%   err_hist       : Erreur cartesienne (3xN)
%   tau_hist       : Couples articulaires (N x 6)
%   q_hist         : Historique articulaire (N x 6)
%   gripper_length : Longueur du gripper pour l'affichage
%   N_frames       : Nombre de frames pour l'animation (optionnel)
%
% Sortie :
%   aucune

    if nargin < 9
        N_frames = 150;
    end

    %% 1) Trajectoire 3D
    figure('Name', 'Suivi dynamique - Trajectoire 3D');
    plot3(x_d(1,:),    x_d(2,:),    x_d(3,:),    'r--', 'LineWidth', 2); hold on;
    plot3(x_hist(1,:), x_hist(2,:), x_hist(3,:), 'b-',  'LineWidth', 1.5);
    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Trajectoire circulaire desiree vs reelle');
    legend('Trajectoire desiree', 'Trajectoire reelle', 'Location', 'best');

    %% 2) Composantes X, Y, Z
    figure('Name', 'Suivi dynamique - Positions cartesiennes');
    labels_xyz = {'X (m)', 'Y (m)', 'Z (m)'};

    for i = 1:3
        subplot(3,1,i);
        plot(t_vec, x_d(i,:),    'r--', 'LineWidth', 1.5); hold on;
        plot(t_vec, x_hist(i,:), 'b-',  'LineWidth', 1.2);
        grid on;
        ylabel(labels_xyz{i});
        if i == 1
            title('Evolution temporelle des positions cartesiennes');
        end
        if i == 3
            xlabel('Temps (s)');
        end
        legend('Desiree', 'Reelle', 'Location', 'best');
    end

    %% 3) Erreur de suivi
    figure('Name', 'Suivi dynamique - Erreur');
    subplot(2,1,1);
    plot(t_vec, err_hist(1,:), 'r', 'LineWidth', 1.2); hold on;
    plot(t_vec, err_hist(2,:), 'g', 'LineWidth', 1.2);
    plot(t_vec, err_hist(3,:), 'b', 'LineWidth', 1.2);
    grid on;
    ylabel('Erreur (m)');
    title('Erreur cartesienne de suivi');
    legend('e_x', 'e_y', 'e_z', 'Location', 'best');

    subplot(2,1,2);
    plot(t_vec, vecnorm(err_hist, 2, 1), 'k', 'LineWidth', 1.5);
    grid on;
    xlabel('Temps (s)');
    ylabel('||e|| (m)');
    title('Norme de l''erreur');

    fprintf('\nErreur maximale de suivi : %.4f mm\n', ...
        1000 * max(vecnorm(err_hist, 2, 1)));
    fprintf('Erreur finale de suivi   : %.4f mm\n', ...
        1000 * norm(err_hist(:,end)));

    %% 4) Couples articulaires
    figure('Name', 'Suivi dynamique - Couples articulaires');
    for j = 1:6
        subplot(3,2,j);
        plot(t_vec, tau_hist(:,j), 'LineWidth', 1.2);
        grid on;
        ylabel(sprintf('\\tau_%d (Nm)', j));
        title(sprintf('Articulation %d', j));
        if j >= 5
            xlabel('Temps (s)');
        end
    end

    %% 5) Animation
    id_frame = unique(round(linspace(1, size(q_hist,1), N_frames)));

    figure('Name', 'Animation - Suivi dynamique cercle');

    ax = show(robot, q_hist(id_frame(1),:), ...
        'Visuals', 'on', ...
        'Frames', 'off', ...
        'PreservePlot', false, ...
        'FastUpdate', true);
    hold on;

    % Cercle desire
    plot3(x_d(1,:), x_d(2,:), x_d(3,:), 'r--', 'LineWidth', 2);

    % Trace reelle progressive
    traj_effecteur = animatedline('Color', 'b', 'LineWidth', 1.5);

    % Premier gripper
    h_grip = draw_gripper(robot, q_hist(id_frame(1),:), gripper_length, false);

    title('Animation du suivi dynamique d''une trajectoire circulaire');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    axis equal;
    axis manual;
    view([-45 30]);

    % Cube de visualisation comme tu l'avais demande
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([0 1]);

    for i = 1:length(id_frame)
        k = id_frame(i);

        show(robot, q_hist(k,:), ...
            'Parent', ax, ...
            'Visuals', 'on', ...
            'Frames', 'off', ...
            'PreservePlot', false, ...
            'FastUpdate', true);
        hold on;

        % Mise a jour de la trace reelle
        addpoints(traj_effecteur, x_hist(1,k), x_hist(2,k), x_hist(3,k));

        % Suppression / re-dessin du gripper
        if ~isempty(h_grip)
            for h = 1:length(h_grip)
                if isgraphics(h_grip(h))
                    delete(h_grip(h));
                end
            end
        end
        h_grip = draw_gripper(robot, q_hist(k,:), gripper_length, false);

        drawnow;
    end

    hold off;
    fprintf('Animation du suivi dynamique terminee.\n');
end