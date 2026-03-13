robot = importrobot('universalUR5.urdf');
bodies = robot.BodyNames;

% Preallocate
n = numel(bodies);
cx = zeros(n,1); cy = zeros(n,1); cz = zeros(n,1);
m = zeros(n,1);
Ix = zeros(n,1); Iy = zeros(n,1); Iz = zeros(n,1);

for i = 1:n
    b = robot.getBody(bodies{i});
    m(i)  = b.Mass;
    cx(i) = b.CenterOfMass(1);
    cy(i) = b.CenterOfMass(2);
    cz(i) = b.CenterOfMass(3);
    Ix(i) = b.Inertia(1);  % Ixx
    Iy(i) = b.Inertia(2);  % Iyy
    Iz(i) = b.Inertia(3);  % Izz
end

T = table(bodies', cx, cy, cz, m, Ix, Iy, Iz, ...
    'VariableNames', {'Membre','cx','cy','cz','m','Ix','Iy','Iz'});
disp(T);