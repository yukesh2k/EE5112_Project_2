% EE5112 task5 submission
% robot_name = 'kinovaJacoJ2N6S300';
robot_name = 'kinovaJacoJ2N6S300';
robot = loadrobot(robot_name, 'DataFormat', 'column'); % Note there are different data formats for loading the robot (chosen in the 2nd argument)
% disp(robot.BodyNames);

numJoints = numel(homeConfiguration(robot));

% endEffector = "EndEffector_Link";
endEffector = "j2n6s300_end_effector";

% Initial end-effector pose
% task5: new intial pose
% taskInit = trvec2tform([[0.4 0 0.2]])*axang2tform([0 1 0 pi]);
taskInit = trvec2tform([[0.4 0 0.2]])*axang2tform([0 1 0 -pi/2]);

% Compute current robot joint configuration using inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);

% The IK solver respects joint limits, but for those joints with infinite
% range, they must be wrapped to a finite range on the interval [-pi, pi].
% Since the the other joints are already bounded within this range, it is
% sufficient to simply call wrapToPi on the entire robot configuration
% rather than only on the joints with infinite range.
currentRobotJConfig = wrapToPi(currentRobotJConfig);

% Final (desired) end-effector pose
% taskFinal = trvec2tform([0.2 0.55 0.35])*axang2tform([0 1 0 pi]); 
% task5: new final pose with upright y axis 
taskFinal = trvec2tform([-0.2 0.4 0.3])*axang2tform([0 1 0 -pi/2]);
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

% Final configuration
finalRobotJConfig = ik(endEffector, taskFinal, weights, currentRobotJConfig);

finalRobotJConfig = wrapToPi(finalRobotJConfig);

helperCreateObstaclesKINOVA;

x0 = [currentRobotJConfig', zeros(1,numJoints)];
helperInitialVisualizerKINOVA;

safetyDistance = 0.01; 

helperSTOMP;