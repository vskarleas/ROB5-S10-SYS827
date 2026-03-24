function plot_commande_en_position(robot, q_traj, positions, N_frames, fig_title)
% plot_commande_en_position anime le robot suivant une trajectoire articulaire
% et affiche la trajectoire cartesienne desiree. Cette fonction est utilisee pour
% visualiser le suivi de trajectoire par cinematique inverse.
%
% Entrees :
%   robot : Objet RigidBodyTree de notre robot
%   q_traj : Trajectoire articulaire (N x 6) car le robot UR5 a 6 dof donc 6 variables articulaires
%   positions :Trajectoire cartesienne desiree (3 x N)
%   N_frames : Nombre de frames pour l'animation
%   fig_title : Titre de la figure

    N = size(q_traj, 1);
    id_frame = round(linspace(1, N, N_frames));

    figure('Name', fig_title);

    % Premier positionnement du robot dans la scene
    ax = show(robot, q_traj(1,:), ...
        'Visuals', 'on', ...
        'Frames', 'off', ...
        'PreservePlot', false, ...
        'FastUpdate', true);
    hold on;

    % Trace de la trajectoire cartesienne desiree
    plot3(positions(1,:), positions(2,:), positions(3,:), 'r-', 'LineWidth', 2);

    title(fig_title);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    view([-45 30]);
    grid on;

    % Animation
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
    fprintf('Animation terminee.\n');
end
