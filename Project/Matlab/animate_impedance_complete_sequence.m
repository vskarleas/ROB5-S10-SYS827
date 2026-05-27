function animate_impedance_complete_sequence(robot, q_hist_all, x_d_all, x_c_all, x_hist_all, ...
                                             Fext_all, z_table, gripper_length, Ke_env_visual, N_frames)
% animate_impedance_complete_sequence anime toute la sequence :
%   - descente + contact
%   - cercle sous contact
%   - remontee
%
% La table s'abaisse visuellement en fonction de la force de contact, reste
% abaissee pendant le cercle, puis remonte lorsque la force revient a zero.
%
% Entrees :
%   robot          : Objet RigidBodyTree
%   q_hist_all     : Historique articulaire concatene
%   x_d_all        : Trajectoire desiree concatenee
%   x_c_all        : Trajectoire corrigee concatenee
%   x_hist_all     : Trajectoire reelle concatenee
%   Fext_all       : Force normale concatenee (3xN)
%   z_table        : Hauteur de la table
%   gripper_length : Longueur du gripper
%   Ke_env_visual  : Raideur utilisee pour convertir la force en compression visuelle
%   N_frames       : Nombre de frames

    if nargin < 10
        N_frames = 260;
    end
    if nargin < 9
        Ke_env_visual = 200;
    end

    % Centre visuel de la table
    x_center_env = mean(x_d_all(1,:));
    y_center_env = mean(x_d_all(2,:));

    % Parametres purement visuels
    visual_scale = 30;       % amplification visuelle pour rendre la baisse visible
    compression_max = 0.04;  % max 4 cm visuellement

    id_frame = unique(round(linspace(1, size(q_hist_all,1), N_frames)));

    figure('Name', 'Animation complete - Contact + cercle + remontee');

    ax = show(robot, q_hist_all(id_frame(1),:), ...
        'Visuals', 'on', ...
        'Frames', 'off', ...
        'PreservePlot', false, ...
        'FastUpdate', true);
    hold on;

    % Environnement initial
    h_env = draw_table_spring_environment(x_center_env, y_center_env, z_table, 0);

    % Trajectoires
    plot3(x_d_all(1,:), x_d_all(2,:), x_d_all(3,:), 'r--', 'LineWidth', 2);
    plot3(x_c_all(1,:), x_c_all(2,:), x_c_all(3,:), 'm-.', 'LineWidth', 1.6);
    traj_effecteur = animatedline('Color', 'b', 'LineWidth', 1.8);

    % Gripper initial
    h_grip = draw_gripper(robot, q_hist_all(id_frame(1),:), gripper_length, false);

    title('Sequence complete : contact, cercle sous contact, remontee');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    axis equal;
    axis manual;
    view([-40 25]);

    xlim([x_center_env - 0.90, x_center_env + 0.90]);
    ylim([y_center_env - 0.90, y_center_env + 0.90]);
    zlim([z_table - 0.90, max(x_hist_all(3,:)) + 0.90]);

    camzoom(1.35);

    for i = 1:length(id_frame)
        k = id_frame(i);

        show(robot, q_hist_all(k,:), ...
            'Parent', ax, ...
            'Visuals', 'on', ...
            'Frames', 'off', ...
            'PreservePlot', false, ...
            'FastUpdate', true);
        hold on;

        % Compression visuelle de la table
        compression = visual_scale * Fext_all(3,k) / Ke_env_visual;
        compression = min(compression, compression_max);

        % Mise a jour environnement
        if ~isempty(h_env)
            valid_env = isgraphics(h_env);
            delete(h_env(valid_env));
        end
        h_env = draw_table_spring_environment(x_center_env, y_center_env, z_table, compression);

        % Trace reelle
        addpoints(traj_effecteur, x_hist_all(1,k), x_hist_all(2,k), x_hist_all(3,k));

        % Mise a jour gripper
        if ~isempty(h_grip)
            for h = 1:length(h_grip)
                if isgraphics(h_grip(h))
                    delete(h_grip(h));
                end
            end
        end
        h_grip = draw_gripper(robot, q_hist_all(k,:), gripper_length, false);

        drawnow;
    end

    hold off;
    fprintf('Animation complete terminee.\n');
end