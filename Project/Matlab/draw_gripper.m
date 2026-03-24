function h_grip = draw_gripper(robot, q, gripper_length, is_open)
% draw_gripper dessine un gripper 3D attache au repere tool0 du robot UR5
%
% Le gripper est compose de :
%   - Un corps cylindrique (entre tool0 et gripper_tip)
%   - Deux doigts rectangulaires a l'extremite
%
% Entrees :
%   robot : Objet RigidBodyTree
%   q : Configuration articulaire (1 x 6)
%   gripper_length : Longueur du gripper (m) (cf. gripper.m)
%   is_open : variable boolean pour controller l\animation d'ouverture et
%   fermeture des doights du gripper
%
% Sortie :
%   h_grip : Vecteur de handles graphiques (pour suppression/maj)


    % Transformation tool0 dans le repere de base
    T_tool = getTransform(robot, q, 'tool0');
    R = T_tool(1:3, 1:3);
    p = T_tool(1:3, 4);

    % Axe local Z du tool (direction du gripper)
    z_local = R(:, 3);
    x_local = R(:, 1);
    y_local = R(:, 2);

    % --- Parametres geometriques du gripper ---
    body_radius  = 0.015;       % rayon du corps
    body_length  = gripper_length * 0.7;  % longueur du corps
    finger_length = gripper_length * 0.3; % longueur des doigts
    finger_width  = 0.008;      % epaisseur des doigts
    finger_depth  = 0.02;       % profondeur des doigts

    if is_open
        finger_offset = 0.025;  % ecartement des doigts (ouvert)
    else
        finger_offset = 0.005;  % ecartement des doigts (ferme)
    end

    h_grip = [];

    % --- Corps du gripper (cylindre) ---
    [cx, cy, cz] = cylinder(body_radius, 12);
    cz = cz * body_length;  % longueur

    % Transformer le cylindre : il est cree le long de Z, on le place
    % le long du z_local du tool
    pts_local = [cx(:)'; cy(:)'; cz(:)'];  % 3 x n_pts
    pts_world = R * pts_local + p;          % rotation + translation

    cx_w = reshape(pts_world(1,:), size(cx));
    cy_w = reshape(pts_world(2,:), size(cy));
    cz_w = reshape(pts_world(3,:), size(cz));

    h_body = surf(cx_w, cy_w, cz_w, ...
        'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.9);
    h_grip = [h_grip; h_body];

    % --- Position de la base des doigts ---
    p_finger_base = p + z_local * body_length;

    % --- Doigt 1 (cote +x_local) ---
    h_f1 = draw_finger(p_finger_base + x_local * finger_offset, ...
                        R, finger_length, finger_width, finger_depth, ...
                        [0.6 0.6 0.6]);
    h_grip = [h_grip; h_f1];

    % --- Doigt 2 (cote -x_local) ---
    h_f2 = draw_finger(p_finger_base - x_local * finger_offset, ...
                        R, finger_length, finger_width, finger_depth, ...
                        [0.6 0.6 0.6]);
    h_grip = [h_grip; h_f2];
end


function h_f = draw_finger(origin, R, length, width, depth, color)
% Dessine un doigt rectangulaire (parallelipipede) a la position donnee
    z_local = R(:, 3);
    x_local = R(:, 1);
    y_local = R(:, 2);

    % 8 sommets d'un parallelipipede en coordonnees locales
    hw = width / 2;
    hd = depth / 2;

    corners_local = [
        -hw, -hd, 0;
         hw, -hd, 0;
         hw,  hd, 0;
        -hw,  hd, 0;
        -hw, -hd, length;
         hw, -hd, length;
         hw,  hd, length;
        -hw,  hd, length;
    ]';  % 3 x 8

    % Transformation en coordonnees monde
    corners_world = R * corners_local + origin;

    vertices = corners_world';  % 8 x 3
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];

    h_f = patch('Vertices', vertices, 'Faces', faces, ...
                'FaceColor', color, 'FaceAlpha', 0.9, ...
                'EdgeColor', color * 0.5, 'LineWidth', 0.5);
end