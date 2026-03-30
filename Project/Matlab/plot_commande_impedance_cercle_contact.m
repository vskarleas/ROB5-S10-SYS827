function plot_commande_impedance_cercle_contact(robot, t_vec, x_d, x_c_hist, x_hist, err_hist, ...
                                                tau_hist, Fext_hist, q_hist, z_table, ...
                                                gripper_length, N_frames)
% plot_commande_impedance_cercle_contact affiche les resultats de la
% commande d'impedance pour une trajectoire circulaire sous contact.

    if nargin < 12
        N_frames = 180;
    end

    % Table
    x_table = [-0.3 0.3; -0.3 0.3];
    y_table = [-0.3 -0.3; 0.3 0.3];
    z_table_plot = z_table * ones(2,2);

    %% 1) Trajectoire 3D
    figure('Name', 'Impedance cercle contact - Trajectoire 3D');
    plot3(x_d(1,:), x_d(2,:), x_d(3,:), 'r--', 'LineWidth', 2); hold on;
    plot3(x_c_hist(1,:), x_c_hist(2,:), x_c_hist(3,:), 'm-.', 'LineWidth', 1.8);
    plot3(x_hist(1,:), x_hist(2,:), x_hist(3,:), 'b-', 'LineWidth', 1.5);
    surf(x_table, y_table, z_table_plot, ...
        'FaceColor', [0.8 0.7 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Commande d''impedance - Trajectoire circulaire sous contact');
    legend('Trajectoire desiree', 'Trajectoire corrigee', 'Trajectoire reelle', 'Table', ...
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

    %% 7) Animation
    id_frame = unique(round(linspace(1, size(q_hist,1), N_frames)));

    figure('Name', 'Animation - Cercle sous contact');

    ax = show(robot, q_hist(id_frame(1),:), ...
        'Visuals', 'on', ...
        'Frames', 'off', ...
        'PreservePlot', false, ...
        'FastUpdate', true);
    hold on;

    surf(x_table, y_table, z_table_plot, ...
        'FaceColor', [0.8 0.7 0.5], 'FaceAlpha', 0.7, 'EdgeColor', 'none');

    plot3(x_d(1,:), x_d(2,:), x_d(3,:), 'r--', 'LineWidth', 2);
    plot3(x_c_hist(1,:), x_c_hist(2,:), x_c_hist(3,:), 'm-.', 'LineWidth', 1.6);
    traj_effecteur = animatedline('Color', 'b', 'LineWidth', 1.5);

    h_grip = draw_gripper(robot, q_hist(id_frame(1),:), gripper_length, false);

    title('Animation de la trajectoire circulaire sous contact');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    axis equal;
    axis manual;
    view([-45 30]);
    xlim([-0.75 1]);
    ylim([-0.75 1]);

    zmin_plot = min([z_table, min(x_hist(3,:)), min(x_c_hist(3,:)), min(x_d(3,:))]) - 0.85;
    zmax_plot = max([max(x_hist(3,:)), max(x_c_hist(3,:)), max(x_d(3,:))]) + 0.85;
    zlim([zmin_plot zmax_plot]);

    for i = 1:length(id_frame)
        k = id_frame(i);

        show(robot, q_hist(k,:), ...
            'Parent', ax, ...
            'Visuals', 'on', ...
            'Frames', 'off', ...
            'PreservePlot', false, ...
            'FastUpdate', true);
        hold on;

        addpoints(traj_effecteur, x_hist(1,k), x_hist(2,k), x_hist(3,k));

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