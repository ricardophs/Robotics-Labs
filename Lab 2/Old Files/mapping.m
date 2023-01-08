map_im = imread('bw.png');
% map_im = ~map_im;
height = size(map_im, 1);
width = size(map_im, 2);

axis ij;
axis manual;
axis([0 height 0 width])
imshow(map_im);
coords = ginput(2);
pixel_length = sqrt((coords(1,1)- coords(1,2))^2+(coords(1,2)-coords(2,2))^2);
prompt = 'Distance in meters:';
meters_length = input(prompt);
resolution = pixel_length/meters_length;

imshow(map_im);
start_pixel = ginput(1);
end_pixel = ginput(1);
close all

start_pixel(2) = height-start_pixel(2);
end_pixel(2) = height-end_pixel(2);

% Hybrid A*
% map = binaryOccupancyMap(map_im,resolution);
% space = stateSpaceSE2;
% space.StateBounds = [0 height*meters_length/pixel_length; 0 width*meters_length/pixel_length; -pi pi];
% validator = validatorOccupancyMap(space);
% validator.Map = map;
% validator.ValidationDistance = 1;
% MinTurningRadius = 3;
% MotionPrimitiveLength = 2*pi*MinTurningRadius/4;
% planner = plannerHybridAStar(validator,'MinTurningRadius',MinTurningRadius,'MotionPrimitiveLength',MotionPrimitiveLength);
% startPose = [start_pixel(1)*meters_length/pixel_length start_pixel(2)*meters_length/pixel_length pi/2];
% goalPose = [end_pixel(1)*meters_length/pixel_length end_pixel(2)*meters_length/pixel_length -pi/2];
% refpath = plan(planner,startPose,goalPose);
% show(planner)

% RRT*
% map = binaryOccupancyMap(map_im,resolution);
% state = stateSpaceSE2;
% state.StateBounds = [0 height*meters_length/pixel_length; 0 width*meters_length/pixel_length; -pi pi];
% validator = validatorOccupancyMap(state);
% validator.Map = map;
% validator.ValidationDistance = 0.5;
% planner = plannerRRTStar(state,validator);
% planner.ContinueAfterGoalReached = true;
% planner.MaxIterations = 20000;
% planner.MaxConnectionDistance = 3;
% start = [start_pixel(1)*meters_length/pixel_length start_pixel(2)*meters_length/pixel_length pi/2];
% goal = [end_pixel(1)*meters_length/pixel_length end_pixel(2)*meters_length/pixel_length -pi/2];
% rng(100, 'twister') % repeatable result
% [pthObj, solnInfo] = plan(planner,start,goal);
% map.show;
% hold on;
% plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
% plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path