function T = DH_Modified_Transform(alpha, a, d, theta)
% Calcule la matrice de transformation homogene 4x4
% selon la convention DH modifiee de Craig.
%
% La transformation est decomposee en :
%   T = Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta)
%
% Entrees :
%   alpha - Angle de torsion du lien (rad)
%   a     - Longueur du lien (m)
%   d     - Decalage le long de l'axe z (m)
%   theta - Angle articulaire (rad ou symbolique)
%
% Sortie :
%   T     - Matrice de transformation homogene 4x4

    T = [cos(theta),              -sin(theta),             0,            a;
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),   -sin(alpha),  -sin(alpha)*d;
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d;
         0,                       0,                       0,            1];
end