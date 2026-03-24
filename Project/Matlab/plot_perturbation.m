function plot_perturbation(t_vec, traj_full, x_hist, x_pert, ...
                          err_hist, err_pert, F_ext_hist, ...
                          Kd, F_contact, t_contact_start, t_contact_end)
% PLOT_PERTURBATION Compare les resultats de la simulation libre et
% de la simulation avec perturbation externe.
%
% Genere 3 figures :
%   1) Comparaison du suivi en position (libre vs perturbee)
%   2) Force externe et deflexion mesuree vs attendue
%   3) Comparaison des erreurs de suivi
%
% Entrees :
%   t_vec           - Vecteur de temps (1 x N)
%   traj_full       - Trajectoire desiree (3 x N)
%   x_hist          - Trajectoire reelle libre (3 x N)
%   x_pert          - Trajectoire reelle perturbee (3 x N)
%   err_hist        - Erreur libre (3 x N)
%   err_pert        - Erreur perturbee (3 x N)
%   F_ext_hist      - Force externe appliquee (3 x N)
%   Kd              - Matrice de raideur (3 x 3)
%   F_contact       - Vecteur de force de contact (3 x 1)
%   t_contact_start - Debut du contact (s)
%   t_contact_end   - Fin du contact (s)

    %% Figure 1 : Suivi par composante (libre vs perturbee)
    figure('Name', 'Suivi de trajectoire par composante (perturbation)');
    axes_labels = {'X', 'Y', 'Z'};

    for ax_i = 1:3
        subplot(3,1,ax_i);
        plot(t_vec, traj_full(ax_i,:)*1000, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_vec, x_hist(ax_i,:)*1000, 'g--', 'LineWidth', 1);
        plot(t_vec, x_pert(ax_i,:)*1000, 'r-', 'LineWidth', 1);

        % Zone de contact
        yl = ylim;
        patch([t_contact_start t_contact_end t_contact_end t_contact_start], ...
              [yl(1) yl(1) yl(2) yl(2)], ...
              [1 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

        ylabel(sprintf('%s (mm)', axes_labels{ax_i}));
        if ax_i == 1
            title('Suivi de trajectoire : libre vs perturbee');
            legend('Desire', 'Libre', 'Perturbee', 'Contact', 'Location', 'best');
        end
        if ax_i == 3, xlabel('Temps (s)'); end
        grid on;
    end

    %% Figure 2 : Force externe et deflexion
    figure('Name', ' Force et deflexion (perturbation)');

    % Force externe
    subplot(3,1,1);
    plot(t_vec, F_ext_hist(1,:), 'r-', ...
         t_vec, F_ext_hist(2,:), 'g-', ...
         t_vec, F_ext_hist(3,:), 'b-', 'LineWidth', 1.5);
    ylabel('Force (N)');
    legend('F_x', 'F_y', 'F_z', 'Location', 'best');
    title('Force externe appliquee'); grid on;

    % Deflexion = difference entre trajectoire libre et perturbee
    deflexion = (x_pert - x_hist) * 1000;  % en mm
    deflexion_attendue = (Kd \ F_contact) * 1000;  % en mm

    subplot(3,1,2);
    plot(t_vec, deflexion(1,:), 'r-', ...
         t_vec, deflexion(2,:), 'g-', ...
         t_vec, deflexion(3,:), 'b-', 'LineWidth', 1);
    hold on;
    % Lignes horizontales pour la deflexion attendue (pendant le contact)
    for ax_i = 1:3
        if abs(deflexion_attendue(ax_i)) > 0.01
            colors = {'r', 'g', 'b'};
            yline(-deflexion_attendue(ax_i), [colors{ax_i} '--'], ...
                sprintf('F/%s = %.1f mm', axes_labels{ax_i}, -deflexion_attendue(ax_i)), ...
                'LineWidth', 1.5);
        end
    end
    ylabel('Deflexion (mm)');
    legend('dx', 'dy', 'dz', 'Location', 'best');
    title('Deflexion mesuree (perturbee - libre)'); grid on;

    % Norme de la deflexion
    subplot(3,1,3);
    plot(t_vec, vecnorm(deflexion), 'k-', 'LineWidth', 1.5);
    hold on;
    yline(norm(deflexion_attendue), 'r--', ...
        sprintf('||F/K|| = %.1f mm', norm(deflexion_attendue)), 'LineWidth', 1.5);
    ylabel('||deflexion|| (mm)');
    xlabel('Temps (s)');
    title('Norme de la deflexion'); grid on;

    %% Figure 3 : Comparaison des erreurs
    figure('Name', 'Comparaison des erreurs (perturbation)');

    subplot(2,1,1);
    plot(t_vec, vecnorm(err_hist)*1000, 'g-', 'LineWidth', 1.5); hold on;
    plot(t_vec, vecnorm(err_pert)*1000, 'r-', 'LineWidth', 1.5);
    % Zone de contact
    yl = ylim;
    patch([t_contact_start t_contact_end t_contact_end t_contact_start], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [1 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    ylabel('||e|| (mm)');
    legend('Libre', 'Perturbee', 'Contact', 'Location', 'best');
    title('Norme de l''erreur de suivi'); grid on;

    subplot(2,1,2);
    plot(t_vec, err_pert(1,:)*1000, 'r-', ...
         t_vec, err_pert(2,:)*1000, 'g-', ...
         t_vec, err_pert(3,:)*1000, 'b-', 'LineWidth', 1);
    hold on;
    yl = ylim;
    patch([t_contact_start t_contact_end t_contact_end t_contact_start], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [1 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    ylabel('Erreur (mm)'); xlabel('Temps (s)');
    legend('e_x', 'e_y', 'e_z', 'Contact', 'Location', 'best');
    title('Erreur de suivi par composante (perturbee)'); grid on;

    fprintf('Figures de perturbation generees.\n');
end