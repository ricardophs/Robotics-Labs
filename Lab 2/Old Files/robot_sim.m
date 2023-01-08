%% Load map

% Get a map of inly the road boundaries to fill the occupancy matrix
map_bw = imread('Images/map_edges.png');
map_bw = ~map_bw;

% Path p and path m are the estimated paths of the vehicle in pixels and in
% meters
load('DataFiles/path_p.mat');
load('DataFiles/path_m.mat');
% Path p ref and path m ref are the reference paths the vehicle should be following in pixels and in
% meters
load('DataFiles/path_p_ref.mat');
load('DataFiles/path_m_ref.mat');

% Map / Image resolution in pixels / meters
res = 2.86;

%% Simulation using pixel data

% To zoom the map, we are croping the part of the map where the robot can
% never go to
% This ocupancy matrix will have the boundaries of the road
map = binaryOccupancyMap(map_bw(100:1100,100:1000));

% Definition of the cars dimentions
d = 0.64*res;
r = 0.256*res;
Lr = 0.566*res;
L = 2.2*res;
Lf = 0.566*res;

% Start position
x_start = path_p_ref(1, 1)-100;
y_start = 1100-path_p_ref(1, 2);

% Finish position
x_end = path_p_ref(end, 1)-100;
y_end = 1100-path_p_ref(end, 2);

% Map limits
x_min = min(path_p_ref(:, 1)')-100;
x_max = max(path_p_ref(:, 1)')-100;
y_max = 1100-min(path_p_ref(:, 2)');
y_min = 1100-max(path_p_ref(:, 2)');

% Create full screen sized figure
figure('units','normalized','outerposition',[0 0 1 1])
for k = 1:10:size(path_p, 1)
    
    % Current position and orientation
    x = path_p(k, 1)-100;
    y = 1100-path_p(k, 2);
    theta = path_p(k, 3);
   
    show(map)
    
    xlim([x_min-50 x_max+50])
    ylim([y_min-50 y_max+50])
    
    hold on;
    
    % Reference path
    plot(path_p_ref(:, 1)-100, 1100-path_p_ref(:, 2), 'LineStyle', '--', 'LineWidth', 1);
    % Start position
    scatter(x_start, y_start, 'filled', 'MarkerFaceColor', [0 0.75 0])
    % Finish position
    scatter(x_end, y_end, 'filled', 'MarkerFaceColor', [0.75 0 0])
    
    % Car representation
    p = patch([x-Lr x-Lr x+L+Lf x+L+Lf], [y-d y+d y+d y-d], [0 0 1]);
    rotate(p, [0 0 1], theta*180/pi, [x y 0]) 

    light;
    drawnow;
    hold off;
end

%% Simulation using meters data

% Another version of the occupancy matrix in meters
map = binaryOccupancyMap(map_bw(100:1100,100:1000), res);

height = size(map_bw,1);
width = size(map_bw,2);

% Definition of the cars dimentions
d = 0.64;
r = 0.256;
Lr = 0.566;
L = 2.2;
Lf = 0.566;

% Start position
x_start = path_m_ref(1, 1)-100/res;
y_start = path_m_ref(1, 2)-(height-1100)/res;

% Finish position
x_end = path_m_ref(end, 1)-100/res;
y_end = path_m_ref(end, 2)-(height-1100)/res;

% Map limits
x_min = min(path_m_ref(:, 1)')-100/res;
x_max = max(path_m_ref(:, 1)')-100/res;
y_max = max(path_m_ref(:, 2)')-(height-1100)/res;
y_min = min(path_m_ref(:, 2)')-(height-1100)/res;

% Create full screen sized figure
figure('units','normalized','outerposition',[0 0 1 1])
for k = 1:10:size(path_m, 1)
    
    % Current position and orientation
    x = path_m(k, 1)-100/res;
    y = path_m(k, 2)-(height-1100)/res;
    theta = path_m(k, 3);
   
    show(map)
    
    xlim([x_min-50/res x_max+50/res])
    ylim([y_min-50/res y_max+50/res])
    
    hold on;
    
    % Reference path
    plot(path_m_ref(:, 1)-100/res, path_m_ref(:, 2)-(height-1100)/res, 'LineStyle', '--', 'LineWidth', 1);
    % Start position
    scatter(x_start, y_start, 'filled', 'MarkerFaceColor', [0 0.75 0])
    % Finish position
    scatter(x_end, y_end, 'filled', 'MarkerFaceColor', [0.75 0 0])
    
    % Car representation
    p = patch([x-Lr x-Lr x+L+Lf x+L+Lf], [y-d y+d y+d y-d], [0 0 1]);
    rotate(p, [0 0 1], theta*180/pi, [x y 0]) 

    light;
    drawnow;
    hold off;
end
