rbsensor = rangeSensor;

truePose = [0 0 pi/4];
trueMap = binaryOccupancyMap(eye(10));
show(trueMap)

[ranges, angles] = rbsensor(truePose, trueMap);

scan = lidarScan(ranges, angles);
figure
plot(scan)

%% 

Create a binary warehouse map and place obstacles at defined locations
map = helperCreateBinaryOccupancyMap;

% Visualize map with obstacles and AGV
figure
show(map)
title('Warehouse Floor Plan With Obstacles and AGV')

% Add AGV to the map
pose = [5 40 0];
helperPlotRobot(gca, pose);

% Simulate lidar sensor and set the detection angles to [-pi/2 pi/2]
lidar = rangeSensor;
lidar.HorizontalAngle = [-pi/2 pi/2];
% Set min and max values of the detectable range of the sensor in meters
lidar.Range = [0 5];

% Load waypoints through which AGV moves
load waypoints.mat
traj = waypointsMap;

% Select a waypoint to visualize scan data
Vehiclepose = traj(350, :);

% Generate lidar readings
[ranges, angles] = lidar(Vehiclepose, map);

% Store and visualize 2-D lidar scan
scan = lidarScan(ranges, angles);
plot(scan)
title('Ego View')
helperPlotRobot(gca, [0 0 Vehiclepose(3)]);

%%

load lidarScans.mat
referenceScan = lidarScans(180);
currentScan = lidarScans(202);
currScanCart = currentScan.Cartesian;
refScanCart = referenceScan.Cartesian;
figure
plot(refScanCart(:,1),refScanCart(:,2),'k.');
hold on
plot(currScanCart(:,1),currScanCart(:,2),'r.');
legend('Reference laser scan','Current laser scan','Location','NorthWest');
transform = matchScans(currentScan,referenceScan)
transScan = transformScan(currentScan,transform);
figure
plot(refScanCart(:,1),refScanCart(:,2),'k.');
hold on
transScanCart = transScan.Cartesian;
plot(transScanCart(:,1),transScanCart(:,2),'r.');
legend('Reference laser scan','Transformed current laser scan','Location','NorthWest');