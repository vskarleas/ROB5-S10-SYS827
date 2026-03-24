clear all;
close all;


% Les 2 segments de 2R
body1 = rigidBody('body1');
body2 = rigidBody('body2');

% Les articulations des 2 segments
jnt1 = rigidBodyJoint('jnt1', 'revolute');
jnt1.HomePosition = 0;
setFixedTransform(jnt1, [0.5 0 0 0], 'dh');
body1.Joint = jnt1;

jnt2 = rigidBodyJoint('jnt2', 'revolute');
jnt2.HomePosition = 0;
setFixedTransform(jnt2, [0.25 0 0 0], 'dh');
body2.Joint = jnt2;

% Definition du robot
robot = rigidBodyTree;

addBody(robot,body1,'base')
addBody(robot,body2,'body1')

% Affichage du robot
figure;
show(robot);
title('Ligne droite');
axis equal;

% Config pi/4
config = homeConfiguration(robot);
config(1).JointPosition = 0;      % joint 1 = 0
config(2).JointPosition = pi/4;   % joint 2 = pi/4
figure;
show(robot, config);
title('Configuration: q2 = \pi/4');
axis equal;




% Cinematique Inverse via Robotics Toolbox
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0 0 0 1 1 0];  % [orientX orientY orientZ posX posY posZ] - on se concentre sur x, y (robot planair)
initialGuess = homeConfiguration(robot);
initialGuess(1).JointPosition = 0;
initialGuess(2).JointPosition = pi/4;


T_target = getTransform(robot, config, 'body2');
[configSol, solInfo] = ik('body2', T_target, weights, initialGuess);


% Animation
q1_end = configSol(1).JointPosition;
q2_end = configSol(2).JointPosition;
N = 50;
figure;
for i = 1:N
    t = i / N;
    anim_config = homeConfiguration(robot);
    anim_config(1).JointPosition = t * q1_end;
    anim_config(2).JointPosition = t * q2_end;
    
    show(robot, anim_config, 'PreservePlot', false);
    view(0, 90);
    axis equal;
    axis([-0.5 1 -0.5 1]);
    title(sprintf('IK Solution: q1=%.2f° q2=%.2f°', ...
        rad2deg(anim_config(1).JointPosition), ...
        rad2deg(anim_config(2).JointPosition)));
    drawnow;
    pause(0.05);
end