function [robot, thetai, ai, alphai, di, T, T_cumul, T06] = init_UR5()
% Initialise le robot UR5, ses parametres DH modifies, et calcule
% le modele cinematique direct pour la configuration de reference.
%
% Cette fonction couvre exclusivement la partie 1.1 du projet :
%   - Definition des parametres geometriques du constructeur
%   - Construction des parametres DH modifies (convention de Craig)
%   - Chargement du modele URDF via la Robotics System Toolbox
%   - Calcul des matrices de transformation elementaires et cumulees
%
% Sorties :
%   robot    - Objet RigidBodyTree du UR5 (Robotics System Toolbox)
%   thetai   - Configuration articulaire de reference (6x1, rad)
%   ai       - Parametres DH : longueurs a_i (6x1)
%   alphai   - Parametres DH : angles alpha_i (6x1)
%   di       - Parametres DH : decalages d_i (6x1)
%   T        - Matrices de transformation elementaires T_{i-1}^{i} (cell 6x1)
%   T_cumul  - Matrices de transformation cumulees T_0^{i} (cell 6x1)
%   T06      - Matrice de transformation de l'effecteur T_0^6 (4x4)

    %% Parametres geometriques UR5 (par le constructeur)
    d1 = 0.089159;
    a2 = -0.425;
    a3 = -0.39225;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;

    %% Parametres DH modifies (convention de Craig)
    ai     = [0; 0; a2; a3; 0; 0];
    alphai = [0; pi/2; 0; 0; pi/2; -pi/2];
    di     = [d1; 0; 0; d4; d5; d6];

    %% Configuration articulaire de reference
    thetai = [deg2rad(-91.06);
              deg2rad(-111.79);
              deg2rad(-104.53);
              deg2rad(-55.59);
              deg2rad(90.79);
              deg2rad(-1.16)];

    %% Chargement du robot UR5 dans la Robotics System Toolbox
    robot = loadrobot("universalUR5", "DataFormat", "row");
    robot.Gravity = [0 0 -9.81];

    %% Matrices de transformation elementaires T_{i-1}^{i}
    T = cell(6,1);
    for i = 1:6
        T{i} = DH_Modified_Transform(alphai(i), ai(i), di(i), thetai(i));
    end

    %% Matrices de transformation cumulees T_0^{i}
    T_cumul = cell(6,1);
    T_cumul{1} = T{1};
    for i = 2:6
        T_cumul{i} = T_cumul{i-1} * T{i};
    end

    %% Pose de l'effecteur
    T06 = T_cumul{6};
end