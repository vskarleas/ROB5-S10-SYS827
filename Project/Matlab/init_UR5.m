function [robot, thetai, ai, alphai, di, T, T_cumul, T06] = init_UR5()
% init_UR: initialise le robot UR5, ses parametres DH modifies, et calcule
% le modele cinematique direct pour une configuration de reference qui est
% definie ci-dessous. De plus il charge le robot par URDF de Robotics System
% Toolbox et il le retourne pour qu'il soit utilisable dans le fichier du projet
% principal pour effectuer des comparaisons et verifications
%
%
% Sorties :
%   robot : Objet RigidBodyTree du UR5 (Robotics System Toolbox)
%   thetai : Configuration articulaire de reference (6x1 en rad)
%   ai : Longueurs a_i (6x1)
%   alphai : Αngles alpha_i (6x1)
%   di : Parametres DH : decalages d_i (6x1)
%   T : Matrices de transformation elementaires T_{i-1}-{i} (cell 6x1)
%   T_cumul : Matrices de transformation cumulees T_0-{i} (cell 6x1)
%   T06 : Matrice de transformation de l'effecteur T_0-6 (4x4)

%% Parametres geometriques UR5 (par le constructeur)
% source : https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
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
    robot.Gravity = [0 0 -9.81]; % la gravite n'est pas definit par defaut sur Matlab

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
