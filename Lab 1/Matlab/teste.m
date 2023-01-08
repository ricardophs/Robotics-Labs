%% Clear everything

clear
clc

%% Create the model

% Import the robot model
robot = importrobot('urdf/niryo.urdf');
% Print details
showdetails(robot);

% Add the visuals to the robot
addVisual(robot.Bodies{1, 1},"Mesh",'stl/base_link.stl')
addVisual(robot.Bodies{1, 2},"Mesh",'stl/shoulder_link.stl')
addVisual(robot.Bodies{1, 3},"Mesh",'stl/arm_link.stl')
addVisual(robot.Bodies{1, 4},"Mesh",'stl/elbow_link.stl')
addVisual(robot.Bodies{1, 5},"Mesh",'stl/forearm_link.stl')
addVisual(robot.Bodies{1, 6},"Mesh",'stl/wrist_link.stl')
addVisual(robot.Bodies{1, 7},"Mesh",'stl/hand_link.stl')

% Show the robot
axes = show(robot);
axes.CameraPositionMode = 'auto';

% Choose some points where the trajectory is going to pass through
wayPoints = [0 0.3 0.5; -0.3 0 0.5; 0 -0.3 0.5; 0.3 0 0.5]; 
hold on;
% Plot waypoints
exampleHelperPlotWaypoints(wayPoints);

% Get the trajectory using a spline
trajectory = cscvn(wayPoints');

% Plot the trajectory
fnplt(trajectory, 'r', 2);

% Motion

% Estimation of the position of the gripper
eeOffset = 0.025;
% Create a new body and coonect it to the body 'tool_link'
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint, trvec2tform([eeOffset 0 0]));
addBody(robot, eeBody, 'tool_link');

% Create an inverse kinematics solver
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;

% Number os points in the trajectory
numTotalPoints = 30;

% Discretize the trajectory into seveval points
eePositions = ppval(trajectory, linspace(0, trajectory.breaks(end), numTotalPoints));

% For each point in the trajectory
for idx = 1:numTotalPoints
    % Get the next point
    tform = trvec2tform(eePositions(:,idx)');
    % Solve from  the current point to the next point
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    % The next 'initial point' will be the current point
    initialguess = configSoln(idx,:);
end

% Visualization
title('Robot waypoint tracking visualization')
axis([-0.3 0.5 -0.3 0.5 -0.3 0.8]);
for idx = 1:numTotalPoints
    show(robot,configSoln(idx,:), 'PreservePlot', false, 'Frames', 'off');
    pause(0.1)
end
hold off