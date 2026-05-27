function h_all = draw_table_spring_environment(x_center, y_center, z_surface, compression)
% draw_table_spring_environment dessine une table 3D avec 4 ressorts
% sous la table. La variable compression permet d'animer visuellement
% l'enfoncement de la table quand le robot pousse dessus.
%
% Entrees :
%   x_center   : centre X de la table
%   y_center   : centre Y de la table
%   z_surface  : hauteur de la surface de la table (sans compression)
%   compression: compression visuelle appliquee a la table (m)
%
% Sortie :
%   h_all : vecteur de handles graphiques

    if nargin < 4
        compression = 0;
    end

    % -----------------------------
    % Parametres geometriques
    % -----------------------------
    table_length    = 0.35;
    table_width     = 0.25;
    table_thickness = 0.025;

    base_length     = 0.42;
    base_width      = 0.32;
    base_thickness  = 0.020;

    spring_height   = 0.14;
    spring_radius   = 0.012;
    spring_turns    = 8;

    % -----------------------------
    % Positions verticales
    % -----------------------------
    z_top_table    = z_surface - compression;
    z_bottom_table = z_top_table - table_thickness;

    z_top_base     = z_bottom_table - spring_height;
    z_bottom_base  = z_top_base - base_thickness;

    % -----------------------------
    % Dessin des elements
    % -----------------------------
    h_all = gobjects(0);

    % Table
    h_table = draw_box_patch(x_center, y_center, z_top_table, ...
                             table_length, table_width, table_thickness, ...
                             [0.62 0.40 0.18], [0.35 0.20 0.08]);
    h_all(end+1) = h_table;

    % Base
    h_base = draw_box_patch(x_center, y_center, z_top_base, ...
                            base_length, base_width, base_thickness, ...
                            [0.18 0.18 0.18], [0.05 0.05 0.05]);
    h_all(end+1) = h_base;

    % Positions des 4 ressorts
    dx = 0.12;
    dy = 0.08;

    spring_positions = [ x_center-dx, y_center-dy;
                         x_center+dx, y_center-dy;
                         x_center-dx, y_center+dy;
                         x_center+dx, y_center+dy ];

    for i = 1:size(spring_positions,1)
        xs = spring_positions(i,1);
        ys = spring_positions(i,2);

        h_spring = draw_spring3d(xs, ys, z_top_base, z_bottom_table, ...
                                 spring_radius, spring_turns, [0.75 0.75 0.75]);
        h_all(end+1) = h_spring;

        % Petite tige centrale pour accentuer l'effet visuel
        h_rod = plot3([xs xs], [ys ys], [z_top_base z_bottom_table], ...
                      'Color', [0.4 0.4 0.4], 'LineWidth', 1.0);
        h_all(end+1) = h_rod;
    end
end


function h = draw_box_patch(xc, yc, z_top, L, W, T, faceColor, edgeColor)
% Dessine une boite 3D dont la face superieure est a z_top

    z_bottom = z_top - T;

    x1 = xc - L/2; x2 = xc + L/2;
    y1 = yc - W/2; y2 = yc + W/2;
    z1 = z_bottom; z2 = z_top;

    vertices = [ ...
        x1 y1 z1;
        x2 y1 z1;
        x2 y2 z1;
        x1 y2 z1;
        x1 y1 z2;
        x2 y1 z2;
        x2 y2 z2;
        x1 y2 z2];

    faces = [ ...
        1 2 3 4;   % dessous
        5 6 7 8;   % dessus
        1 2 6 5;   % face 1
        2 3 7 6;   % face 2
        3 4 8 7;   % face 3
        4 1 5 8];  % face 4

    h = patch('Vertices', vertices, 'Faces', faces, ...
              'FaceColor', faceColor, ...
              'EdgeColor', edgeColor, ...
              'FaceAlpha', 0.95, ...
              'LineWidth', 0.8);
end


function h = draw_spring3d(xc, yc, z_start, z_end, radius, nTurns, colorSpring)
% Dessine un ressort helicoidal entre z_start et z_end

    nPts = 250;
    theta = linspace(0, 2*pi*nTurns, nPts);
    z = linspace(z_start, z_end, nPts);

    x = xc + radius*cos(theta);
    y = yc + radius*sin(theta);

    h = plot3(x, y, z, 'Color', colorSpring, 'LineWidth', 2.0);
end