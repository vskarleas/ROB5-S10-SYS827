function robot = gripper(robot, gripper_length, gripper_mass)
% gripper ajoute un corps rigide representant un gripper a l'effecteur
% du robot.
%
% Le gripper est attache au corps 'tool0' par un joint fixe avec un
% decalage selon l'axe Z de longueur gripper_length. Sa masse, son centre
% de masse et son inertie sont ajoutes au modele afin de prendre en compte
% l'effet du gripper dans la dynamique du robot.
%
% Entrees :
%   robot          : Objet RigidBodyTree
%   gripper_length : Longueur du gripper, correspondant au decalage selon Z
%                    a partir de 'tool0' (m)
%   gripper_mass   : Masse du gripper (kg)
%
% Sortie :
%   robot          : Modele du robot modifie avec le corps 'gripper_tip' ajoute

    % Transformation fixe entre 'tool0' et le repere du gripper
    T_gripper = [1 0 0 0;
                 0 1 0 0;
                 0 0 1 gripper_length;
                 0 0 0 1];

    % Creation du corps rigide representant le gripper
    gripperBody = rigidBody('gripper_tip');
    gripperBody.Mass = gripper_mass;

    % Approximation de l'inertie du gripper par un cylindre mince
    % de rayon r = 0.02 m
    r_grip = 0.02;

    Ixx = (1/12) * gripper_mass * (3*r_grip^2 + gripper_length^2);
    Iyy = Ixx;
    Izz = (1/2) * gripper_mass * r_grip^2;
    gripperBody.Inertia = [Ixx, Iyy, Izz, 0, 0, 0];

    % Le centre de masse est place a mi-longueur du gripper.
    % Il est exprime dans le repere du corps ajoute ; selon la convention
    % choisie ici, cela conduit a une coordonnee negative sur l'axe Z.
    gripperBody.CenterOfMass = [0 0 -gripper_length/2];

    % Creation du joint fixe reliant le gripper a 'tool0'
    gripperJoint = rigidBodyJoint('gripper_joint', 'fixed');
    setFixedTransform(gripperJoint, T_gripper);
    gripperBody.Joint = gripperJoint;

    % Ajout du gripper au modele du robot
    addBody(robot, gripperBody, 'tool0');

    fprintf('Gripper ajoute : longueur = %.3f m, masse = %.3f kg\n', ...
        gripper_length, gripper_mass);
    fprintf('  Inertie [Ixx, Iyy, Izz] = [%.2e, %.2e, %.2e] kg.m^2\n', ...
        Ixx, Iyy, Izz);
end