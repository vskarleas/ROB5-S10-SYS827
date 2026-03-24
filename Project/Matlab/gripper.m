function robot = gripper(robot, gripper_length, gripper_mass)
% Ajoute un corps rigide representant un gripper a
% l'effecteur du robot.
%
% Le gripper est attache a 'tool0' par un joint fixe avec un decalage
% en Z de gripper_length. Sa masse et son inertie sont integrees au
% modele dynamique du robot egalement.
%
% Entrees :
%   robot          - Objet RigidBodyTree
%   gripper_length - Longueur du gripper, decalage en Z (m)
%   gripper_mass   - Masse du gripper (kg)
%
% Sortie :
%   robot          - Robot modifie avec le corps 'gripper_tip' ajoute
%
% Utilisation :
%   robot = gripper(robot, 0.12, 0.8);
%   T = getTransform(robot, q, 'gripper_tip');

    % ne pas ajouter deux fois le gripper
    bodies = robot.Bodies;
    body_names = cell(1, numel(bodies));
    for b = 1:numel(bodies)
        body_names{b} = bodies{b}.Name;
    end
    if any(strcmp(body_names, 'gripper_tip'))
        warning('attach_gripper: le corps ''gripper_tip'' existe deja. Aucune modification.');
        return;
    end

    % Matrice de transformation avec un decalage en Z par rapport a tool0
    T_gripper = [1 0 0 0;
                 0 1 0 0;
                 0 0 1 gripper_length;
                 0 0 0 1];

    % Creation du corps rigide
    gripperBody = rigidBody('gripper_tip');
    gripperBody.Mass = gripper_mass;

    % Inertie approchee comme un cylindre mince de rayon r = 0.02 m
    r_grip = 0.02;
    Ixx = (1/12) * gripper_mass * (3*r_grip^2 + gripper_length^2);
    Iyy = Ixx;
    Izz = (1/2) * gripper_mass * r_grip^2;
    gripperBody.Inertia = [Ixx, Iyy, Izz, 0, 0, 0];

    % Joint fixe
    gripperJoint = rigidBodyJoint('gripper_joint', 'fixed');
    setFixedTransform(gripperJoint, T_gripper);
    gripperBody.Joint = gripperJoint;

    % Ajout au robot
    addBody(robot, gripperBody, 'tool0');

    fprintf('Gripper ajoute : longueur = %.3f m, masse = %.3f kg\n', ...
        gripper_length, gripper_mass);
    fprintf('  Inertie [Ixx, Iyy, Izz] = [%.2e, %.2e, %.2e] kg.m^2\n', ...
        Ixx, Iyy, Izz);
end