function plot_commande_impedance_cercle_contact(robot, t_vec, x_d, x_c_hist, x_hist, err_hist, ...
                                                tau_hist, Fext_hist, q_hist, z_table, ...
                                                gripper_length, Ke_env_visual, N_frames)
% plot_commande_impedance_cercle_contact affiche les resultats de la
% commande d'impedance pour une trajectoire circulaire sous contact,
% avec une table 3D et des ressorts animes visuellement.
%
% Entrees :
%   robot          : Objet RigidBodyTree
%   t_vec          : Vecteur temps
%   x_d            : Trajectoire desiree
%   x_c_hist       : Trajectoire corrigee
%   x_hist         : Trajectoire reelle
%   err_hist       : Erreur de suivi
%   tau_hist       : Couples articulaires
%   Fext_hist      : Force normale de contact
%   q_hist         : Historique articulaire
%   z_table        : Hauteur de la table
%   gripper_length : Longueur visuelle du gripper
%   Ke_env_visual  : raideur utilisee pour convertir la force en compression visuelle
%   N_frames       : nombre de frames d'animation

    if nargin < 13
        N_frames = 220;
    end
    if nargin < 12
        Ke_env_visual = 200;
    end

    % Centre visuel de la table = centre moyen de la trajectoire
    x_center_env = mean(x_d(1,:));
    y_center_env = mean(x_d(2,:));

    % -----------------------------
    % Parametres visuels
    % -----------------------------
    visual_scale = 12;      % amplifie la compression pour qu'elle soit visible
    compression_max = 0.02; % 2 cm max visuellement

    %% 1) Trajectoire 3D
    figure('Name', 'Impedance cercle contact - Trajectoire 3D');
    plot3(x_d(1,:), x_d(2,:), x_d(3,:), 'r--', 'LineWidth', 2); hold on;
    plot3(x_c_hist(1,:), x_c_hist(2,:), x_c_hist(3,:), 'm-.', 'LineWidth', 1.8);
    plot3(x_hist(1,:), x_hist(2,:), x_hist(3,:), 'b-', 'LineWidth', 1.5);

    h_env_static = draw_table_spring_environment(x_center_env, y_center_env, z_table, 0);

    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Commande d''impedance - Trajectoire circulaire sous contact');
    legend('Trajectoire desiree', 'Trajectoire corrigee', 'Trajectoire reelle', ...
           'Location', 'best');

    %% 2) Projection XY
    figure('Name', 'Impedance cercle contact - Projection XY');
    plot(x_d(1,:), x_d(2,:), 'r--', 'LineWidth', 2); hold on;
    plot(x_c_hist(1,:), x_c_hist(2,:), 'm-.', 'LineWidth', 1.8);
    plot(x_hist(1,:), x_hist(2,:), 'b-', 'LineWidth', 1.5);
    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Projection XY du cercle sous contact');
    legend('Desiree', 'Corrigee', 'Reelle', 'Location', 'best');

    %% 3) Positions cartesiennes
    figure('Name', 'Impedance cercle contact - Positions');
    labels_xyz = {'X (m)', 'Y (m)', 'Z (m)'};

    for i = 1:3
        subplot(3,1,i);
        plot(t_vec, x_d(i,:), 'r--', 'LineWidth', 1.5); hold on;
        plot(t_vec, x_c_hist(i,:), 'm-.', 'LineWidth', 1.4);
        plot(t_vec, x_hist(i,:), 'b-', 'LineWidth', 1.2);

        if i == 3
            yline(z_table, 'k--', 'LineWidth', 1.2);
        end

        grid on;
        ylabel(labels_xyz{i});

        if i == 1
            title('Evolution temporelle des positions cartesiennes');
        end
        if i == 3
            xlabel('Temps (s)');
        end

        if i == 3
            legend('Desiree', 'Corrigee', 'Reelle', 'Table', 'Location', 'best');
        else
            legend('Desiree', 'Corrigee', 'Reelle', 'Location', 'best');
        end
    end

    %% 4) Erreur
    figure('Name', 'Impedance cercle contact - Erreur');
    subplot(2,1,1);
    plot(t_vec, err_hist(1,:), 'r', 'LineWidth', 1.2); hold on;
    plot(t_vec, err_hist(2,:), 'g', 'LineWidth', 1.2);
    plot(t_vec, err_hist(3,:), 'b', 'LineWidth', 1.2);
    grid on;
    ylabel('Erreur (m)');
    title('Erreur de suivi de la trajectoire corrigee');
    legend('e_x','e_y','e_z','Location','best');

    subplot(2,1,2);
    plot(t_vec, vecnorm(err_hist,2,1), 'k', 'LineWidth', 1.5);
    grid on;
    xlabel('Temps (s)');
    ylabel('||e|| (m)');
    title('Norme de l''erreur');

    fprintf('\nErreur maximale de suivi (cercle sous contact) : %.4f mm\n', ...
        1000 * max(vecnorm(err_hist,2,1)));
    fprintf('Erreur finale de suivi   (cercle sous contact) : %.4f mm\n', ...
        1000 * norm(err_hist(:,end)));

    %% 5) Force de contact
    figure('Name', 'Impedance cercle contact - Force de contact');
    plot(t_vec, Fext_hist(3,:), 'LineWidth', 1.5);
    grid on;
    xlabel('Temps (s)');
    ylabel('F_z (N)');
    title('Force normale de contact pendant le cercle');

    %% 6) Couples articulaires
    figure('Name', 'Impedance cercle contact - Couples articulaires');
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

    %% 7) Animation visuelle amelioree
    id_frame = unique(round(linspace(1, size(q_hist,1), N_frames)));

    figure('Name', 'Animation - Cercle sous contact avec table et ressorts');

    ax = show(robot, q_hist(id_frame(1),:), ...
        'Visuals', 'on', ...
        'Frames', 'off', ...
        'PreservePlot', false, ...
        'FastUpdate', true);
    hold on;

    % Environnement initial
    h_env = draw_table_spring_environment(x_center_env, y_center_env, z_table, 0);

    % Trajectoires
    plot3(x_d(1,:), x_d(2,:), x_d(3,:), 'r--', 'LineWidth', 2);
    plot3(x_c_hist(1,:), x_c_hist(2,:), x_c_hist(3,:), 'm-.', 'LineWidth', 1.6);
    traj_effecteur = animatedline('Color', 'b', 'LineWidth', 1.8);

    % Gripper initial
    h_grip = draw_gripper(robot, q_hist(id_frame(1),:), gripper_length, false);

    title('Animation de la trajectoire circulaire sous contact');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    axis equal;
    axis manual;
    view([-40 25]);

    % Zoom plus local sur la scene
    xlim([x_center_env - 0.90, x_center_env + 0.90]);
    ylim([y_center_env - 0.90, y_center_env + 0.90]);

    zmin_plot = z_table - 0.90;
    zmax_plot = max([max(x_hist(3,:)), max(x_c_hist(3,:)), max(x_d(3,:))]) + 0.90;
    zlim([zmin_plot zmax_plot]);

    camzoom(1.3);

    for i = 1:length(id_frame)
        k = id_frame(i);

        show(robot, q_hist(k,:), ...
            'Parent', ax, ...
            'Visuals', 'on', ...
            'Frames', 'off', ...
            'PreservePlot', false, ...
            'FastUpdate', true);
        hold on;

        % Compression visuelle de la table a partir de la force de contact
        compression = visual_scale * Fext_hist(3,k) / Ke_env_visual;
        compression = min(compression, compression_max);

        % Mise a jour environnement
        if ~isempty(h_env)
            valid_env = isgraphics(h_env);
            delete(h_env(valid_env));
        end
        h_env = draw_table_spring_environment(x_center_env, y_center_env, z_table, compression);

        % Trace de l'effecteur
        addpoints(traj_effecteur, x_hist(1,k), x_hist(2,k), x_hist(3,k));

        % Mise a jour gripper
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
    fprintf('Animation du cercle sous contact terminee.\n');
end