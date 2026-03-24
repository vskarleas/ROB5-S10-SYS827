function T = DH_Modified_Transform(alpha, a, d, theta)
% DH_modified_Transform calcule la matrice de transformation homogene 4x4 pour un
% membre selon la convention DH modifiee de Craig vue en cours (cours 1, page 30)
%
% La transformation est decomposee en :
%   T = Rot_x(alpha) * Trans_x(a) * Trans_z(d) * Rot_z(theta)
%
% Entrees :
%   alpha : Angle de torsion du lien (rad)
%   a : Longueur du lien (m)
%   d : Decalage le long de l'axe z (m)
%   theta : Angle articulaire (rad ou symbolique)
%
% Sortie :
%   T : Matrice de transformation homogene 4x4 pour un membre

    T = [cos(theta),              -sin(theta),             0,            a;
         sin(theta)*cos(alpha),   cos(theta)*cos(alpha),   -sin(alpha),  -sin(alpha)*d;
         sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d;
         0,                       0,                       0,            1];
end
