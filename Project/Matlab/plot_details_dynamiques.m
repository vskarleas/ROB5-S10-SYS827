function plot_details_dynamiques(robot)
% plot_details_dynamiques affiche les parametres dynamiques de la
% reference robot qui est donne comme argument comme vu en cours 5, page 44

bodies = robot.BodyNames;

n = numel(bodies);
cx = zeros(n,1); cy = zeros(n,1); cz = zeros(n,1);
m  = zeros(n,1);
Ixx = zeros(n,1); Iyy = zeros(n,1); Izz = zeros(n,1);
Iyz = zeros(n,1); Ixz = zeros(n,1); Ixy = zeros(n,1);

for i = 1:n
    b = robot.getBody(bodies{i});
    m(i)   = b.Mass;
    cx(i)  = b.CenterOfMass(1);
    cy(i)  = b.CenterOfMass(2);
    cz(i)  = b.CenterOfMass(3);
    Ixx(i) = b.Inertia(1);
    Iyy(i) = b.Inertia(2);
    Izz(i) = b.Inertia(3);
    Iyz(i) = b.Inertia(4);
    Ixz(i) = b.Inertia(5);
    Ixy(i) = b.Inertia(6);
end

T = table(bodies', cx, cy, cz, m, Ixx, Iyy, Izz, Iyz, Ixz, Ixy, ...
    'VariableNames', {'Membre','cx','cy','cz','m','Ixx','Iyy','Izz','Iyz','Ixz','Ixy'});
disp(T);
