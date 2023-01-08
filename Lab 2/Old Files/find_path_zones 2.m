%% Load map

% When loading the mao where the robot can go, the user has two options: to
% use the default IST map, or to input a new map. Notice that, when
% inputing a new map, the image must be converted to a binary image. This
% allow us to distinguish between roads and forbiden regions. When inputing
% a new image, the user must also introduce the resolution of the map to be
% possible to convert pixels to meters. A final request if for the user to
% define which color, black or white, defines the roads where the robot can
% move.

prompt = 'For the map, choose 0 for the default IST map or 1 to input a new map:\n';
while(1)
    map_id = input(prompt);
    % Loading IST's defualt map
    if map_id == 0
        filename = 'ist.png';
        map_original = imread(filename);
        % Resolution in pixels / meter
        resolution = 6.9;
        map_original = ~~map_original;
        break;
    % Loading new map
    elseif map_id == 1
        prompt = 'Input the name of the new map:\n';
        while(1)
            filename = input(prompt, 's');
            % Check if the file given by the user exists
            if exist(filename, 'file')
                [X, colormap] = imread(filename);
                if isempty(colormap)
                    map_original = im2bw(X);
                else
                    gray=rgb2gray(colormap);
                    threshold = 128/256;
                    lbw = double(gray > threshold);
                    map_original = im2bw(X,lbw);
                end
                prompt = 'Insert resolution in pixels / meter:\n';
                resolution = input(prompt);
                prompt = 'For road representation, choose 0 for black or 1 for white:\n';
                % Input for the road color (black or white)
                while(1)
                    roads = input(prompt);
                    if roads == 0
                        map_original = ~~map_original;
                        break;
                    elseif roads == 1
                        map_original = ~map_original;
                        break;
                    else
                        prompt = 'Invalid input. You must choose 0 or 1:\n';
                    end
                end
                break;
            else
                 prompt = 'File not found. Try again:\n';
            end
        end
        break;
    else
        prompt = 'Invalid input. You must choose 0 or 1:\n';
    end
end

imshow(map_original);

%% Convert to RGB and process zones

map_im = map_original;
height = size(map_im, 1);
width = size(map_im, 2);

% Add Border
% If the pixels near the edges of the image are meant to be roads, we need
% to define borders that will be used for the definition of the forbiden
% and yellow zones
border = zeros(height,1)+255;
map_im = [border, map_im, border];
border = zeros(1,size(map_im,2))+255;
map_im = [border; map_im; border];
height = size(map_im, 1);
width = size(map_im, 2);

% Convert binary map to RGB
map_rgb = uint8(map_im(:,:,[1 1 1])*255);
% The default map is a map with the default zones. This is used for the
% following: iuf the user inputs very large thicknesses for the forbiden
% and yellow zones, if might be the case that the robot cannot go from
% start to finish. But, maybe, using the default zones, a path might be
% possible. So, if this happens, we can alert the user that there is no
% path for the input zones because these are too big.
map_default_zones = map_rgb;

% Get thickness of the forbiden and yellow zones from the user
% Default thickness for both zones if 0.5 meters.
red_thres_def = round(0.5*resolution);
yellow_thres_def = round(0.5*resolution);
prompt = "For forbiden and yellow zones' thickness, choose 0 for default values of 1 for new values:\n";
while(1)
    zones = input(prompt);
    if zones == 0
        red_thres = red_thres_def;
        yellow_thres = yellow_thres_def;
        break;
    elseif zones == 1
        prompt = 'Insert thickness of the forbiden zone in meters:\n';
        red_thres = input(prompt);
        red_thres = round(red_thres*resolution);
        prompt = 'Insert thickness of the yellow zone in meters:\n';
        yellow_thres = input(prompt);
        yellow_thres = round(yellow_thres*resolution);
        break;
    else
        prompt = 'Invalid input. You must choose 0 or 1:\n';
    end
end

% Find edges
for i=1:height
    for j=1:width
        if map_im(i,j) ~= 0
            % If we are in the white zone (no road) and we have a black
            % pixel around, we are in a border
            if (i > 1 && map_im(i-1,j) == 0) || (i < height && map_im(i+1,j) == 0) || (j > 1 && map_im(i,j-1) == 0) || (j < width && map_im(i,j+1) == 0) || (i > 1 && j > 1 && map_im(i-1,j-1) == 0) || (i > 1 && j < width && map_im(i-1,j+1) == 0) || (i < height && j > 1 && map_im(i+1,j-1) == 0) || (i < height && j < width && map_im(i+1,j+1) == 0)
                for k=-red_thres-yellow_thres:red_thres+yellow_thres
                    for l=-red_thres-yellow_thres:red_thres+yellow_thres
                        if i+k > 0 && i+k < height && j+l > 0 && j+l < width && (is_black(map_rgb(i+k,j+l,:)) || is_green(map_rgb(i+k,j+l,:)) || is_yellow(map_rgb(i+k,j+l,:)))
                            if k >= - red_thres && k <= red_thres && l >= - red_thres && l <= red_thres
                                map_rgb(i+k,j+l,:) = [255,0,0];
                            else
                                map_rgb(i+k,j+l,:) = [255,255,0];
                            end
                        end
                    end
                end
                for k=-red_thres_def-yellow_thres_def:red_thres_def+yellow_thres_def
                    for l=-red_thres_def-yellow_thres_def:red_thres_def+yellow_thres_def
                        if i+k > 0 && i+k < height && j+l > 0 && j+l < width && (is_black(map_default_zones(i+k,j+l,:)) || is_green(map_default_zones(i+k,j+l,:)) || is_yellow(map_default_zones(i+k,j+l,:)))
                            if k >= - red_thres_def && k <= red_thres_def && l >= - red_thres_def && l <= red_thres_def
                                map_default_zones(i+k,j+l,:) = [255,0,0];
                            else
                                map_default_zones(i+k,j+l,:) = [255,255,0];
                            end
                        end
                    end
                end
                map_rgb(i,j,:) = [0,0,255];
                map_default_zones(i,j,:) = [0,0,255];
            end
        % If we are in the road zone, we should fill it with green if the
        % pixels are black
        elseif is_black(map_rgb(i,j,:)) || is_black(map_default_zones(i,j,:))
            if is_black(map_rgb(i,j,:))
                map_rgb(i,j,:) = [0,255,0];
            end
            if is_black(map_default_zones(i,j,:))
                map_default_zones(i,j,:) = [0,255,0];
            end
        end
    end
end

imshow(map_rgb)

%% Start and Finish positions

% Predefined default start and end values
start_default = [208, 256];
end_default = [746, 836];

% If we are in the IST map, we can choose to use the default positions or
% we can enter new ones. If the use a new map the user has to input new
% positions.
if strcmp(filename,'ist.png')
    prompt = 'For initial and final positions, choose 0 for default or 1 for new positions:\n';
    while(1)
        pos = input(prompt);
        if pos == 0
            start_in = start_default;
            end_in = end_default;
            break;
        elseif pos == 1
            imshow(map_im);
            start_in = round(ginput(1));
            end_in = round(ginput(1));
            start_in([1 2]) = start_in([2 1]);
            end_in([1 2]) = end_in([2 1]);
            close all
            break;
        else
            prompt = 'Invalid input. You must choose 0 or 1:\n';
        end
    end
else
    imshow(map_im);
    start_in = round(ginput(1));
    end_in = round(ginput(1));
    start_in([1 2]) = start_in([2 1]);
    end_in([1 2]) = end_in([2 1]);
    close all
end

start_p = zeros(1, 2);
end_p = zeros(1, 2);

% g_input is not very precise. It might be the case where the user clicks
% in a pixel near the green desired one but the selected pixel might be
% another color. Also, the forbiden and yellow regions might be so big that
% there are no green pixels for the robot to move in. Because of this, we
% always find the nearest green pixel for the start and end position.
[start_p(1), start_p(2)] = nearestGreenPixel(map_rgb, start_in(1), start_in(2));

if (start_p(1) == -1) && (start_p(2) == -1)
    fprintf('Error: There is no green zone in the robot path. Try to diminuish the thickness of the forbiden and yellow zones.\n')
    return;
end

[end_p(1), end_p(2)] = nearestGreenPixel(map_rgb, end_in(1), end_in(2));

% The initial and final position cannot be the same
if ((start_p(1) == end_p(1)) && (start_p(2) == end_p(2)))
    fprintf('Error: The start and final position are the same. Try to enter new positions or diminuish the thickness of the forbiden and yellow zones.\n')
    return;
end

imshow(map_rgb)
hold on
scatter(start_in(2), start_in(1), [], [0,0,0], 'filled', 'DisplayName', 'Start - User input')
hold on
scatter(start_p(2), start_p(1), [], [0,0.75,0], 'filled', 'DisplayName', 'Start - Corrected')
hold on
scatter(end_in(2), end_in(1), [], [0,0,0], 'filled', 'DisplayName', 'Finish - User input')
hold on
scatter(end_p(2), end_p(1), [], [0.75,0,0], 'filled', 'DisplayName', 'Finish - Corrected')
hold on
legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');

%% Dijkstra and shortest path

[dist, p] = findShortestPath(map_rgb, start_p, end_p, 1);

% There are two reasons why the path from start to finnish doesn't exist.
% One of them is that the roads are not connected. Another is due to the
% thickness of the forbiden and yellow zones.
if p == 0
    [dist_def, p_def] = findShortestPath(map_default_zones, start_p, end_p, 1);
    if p_def == 0
        fprintf('Error: There if no possible path between the chosen start and finish positions.\n')
    else
        fprintf("Error: The chosen thickness of the forbiden and yellow zones is too big for there fo be a path from the start to the finish. Please choose smaller thickness values of use the default values.\n")
    end
    return;
end

imshow(map_rgb)
hold on
plot(p(:,2), p(:,1), 'black', 'DisplayName', 'Path', 'LineWidth', 3)
hold on
scatter(start_p(2), start_p(1),[],[0,0.75,0],'filled', 'DisplayName', 'Start')
hold on
scatter(end_p(2), end_p(1),[],[0.75,0,0],'filled', 'DisplayName', 'Finish')
hold on
legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');

% Get path in meters instead of pixels
p_meters = p/resolution;
p_meters = fliplr(p_meters);
p_meters(:,2) = height/resolution-p_meters(:,2);

%% Checkpoints

% Default step in meters
step_def = 5;

% Define the step for the checkpoints
prompt = 'For the checkpoint step, choose 0 for default or 1 for a new one:\n';
while(1)
    step_id = input(prompt);
    if step_id == 0
        step = step_def;
        break;
    elseif step_id == 1
        prompt = 'Input checkpoint step in meters:\n';
        step = input(prompt);
        break;
    else
        prompt = 'Invalid input. You must choose 0 or 1:\n';
    end
end

step_p = round(step*resolution);
checkpoints = p(step_p:step_p:end,:);
checkpoints_meters = checkpoints/resolution;

imshow(map_rgb)
hold on
scatter(checkpoints(:,2), checkpoints(:,1), [], [0,0,0], 'filled', 'DisplayName', 'Path', 'LineWidth', 3)
hold on
scatter(start_p(2), start_p(1), [], [0,0.75,0], 'filled', 'DisplayName', 'Start')
hold on
scatter(end_p(2), end_p(1), [], [0.75,0,0], 'filled', 'DisplayName', 'Finish')
hold on
legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');

%% Auxiliary Functions

function val = is_white(rgb)

    if (rgb(1) == 255) && (rgb(2) == 255) && (rgb(3) == 255)
        val = true;
    else
        val = false;
    end

end

function val = is_black(rgb)

    if (rgb(1) == 0) && (rgb(2) == 0) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end

function val = is_green(rgb)

    if (rgb(1) == 0) && (rgb(2) == 255) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end

function val = is_blue(rgb)

    if (rgb(1) == 0) && (rgb(2) == 0) && (rgb(3) == 255)
        val = true;
    else
        val = false;
    end

end

function val = is_red(rgb)

    if (rgb(1) == 255) && (rgb(2) == 0) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end

function val = is_yellow(rgb)

    if (rgb(1) == 255) && (rgb(2) == 255) && (rgb(3) == 0)
        val = true;
    else
        val = false;
    end

end        
        
function [i_out, j_out] = nearestGreenPixel(map, i, j)
    if is_green(map(i,j,:))
        i_out = i;
        j_out = j;
        return;
    end
    height = size(map, 1);
    width = size(map, 2);
    vis = false(height,width);
    dist = Inf(height,width);
    dist(i, j) = 0;
    operations = [1 0; 0 1; -1 0; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
    q = PriorityQueue(1);
    q.insert([dist(i, j) i j]);
    while(q.size() ~= 0)
        curr = q.peek();
        i_curr = curr(2);
        j_curr = curr(3);
        vis(i_curr, j_curr) = true;
        q.remove();
        if is_green(map(i_curr,j_curr,:))
            i_out = i_curr;
            j_out = j_curr;
            return;
        end
        if dist(i_curr, j_curr) < curr(1)
            continue;
        end
        for k=1:8
            new_i = i_curr+operations(k,1);
            new_j = j_curr+operations(k,2);
            if new_i <= 0 || new_i > height || new_j <= 0 || new_j > width
                continue;
            end
            if k < 5
                d = 1;
            else
                d = sqrt(2);
            end
            newDist = dist(i_curr,j_curr) + d;
            if newDist < dist(new_i, new_j)
                dist(new_i, new_j) = newDist;
            end
            if vis(new_i, new_j) == false
                vis(new_i, new_j) = true;
                q.insert([dist(new_i, new_j), new_i, new_j])
            end
        end
    end
    i_out = -1;
    j_out = -1;
end

function [dist_val, path] = findShortestPath(map, start_p, end_p, heur)
    if heur == 1
        [dist_val, prev] = dijkstra_map_heur(map, start_p, end_p);
    else
        [dist_val, prev] = dijkstra_map(map, start_p, end_p);
    end
    if dist_val == Inf
        path = 0;
        return;
    end
    path = zeros(round(dist_val+1)+1,2);
    curr_p = end_p;
    it = 1;
    while curr_p(1) ~= -1
        path(it, :) = curr_p;
        curr_p = prev(curr_p(1), curr_p(2), :);
        it = it + 1;
    end
    path(it:round(dist_val+1)+1,:) = [];
    path = flip(path);
    dist_val = it;
    return;
end

function [dist_val, prev] = dijkstra_map(map, start_p, end_p)
    if is_white(map(start_p(1),start_p(2),:)) || is_white(map(end_p(1),end_p(2),:)) || is_blue(map(start_p(1),start_p(2),:)) || is_blue(map(end_p(1),end_p(2),:)) || is_red(map(start_p(1),start_p(2),:)) || is_red(map(end_p(1),end_p(2),:))
        dist_val = 0;
        prev = 0;
        return;
    end
    height = size(map, 1);
    width = size(map, 2);
    operations = [1 0; 0 1; -1 0; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
    vis = false(height,width);
    for i=1:height
        for j=1:width
            if is_white(map(i,j,:)) || is_blue(map(i,j,:)) || is_red(map(i,j,:))
                vis(i,j) = true;
            end
        end
    end
    prev = -ones(height,width,2);
    dist = Inf(height,width);
    dist(start_p(1), start_p(2)) = 0;
    q = PriorityQueue(3);
    q.insert([start_p(1) start_p(2) 0]);
    while (q.size() ~= 0)
        curr = q.peek();
        i = curr(1);
        j = curr(2);
        value = curr(3);
        q.remove();
        vis(i, j) = true;
        if dist(i, j) < value
            continue;
        end
        for k=1:8
            if k < 5
                penalty = 0;
            else
                penalty = 0.5;
            end
            new_i = i+operations(k,1);
            new_j = j+operations(k,2);
            if new_i <= 0 || new_i > height || new_j <= 0 || new_j > width || vis(new_i, new_j)
                continue;
            end
            if is_green(map(new_i,new_j,:))
                newDist = dist(i,j) + 1 + penalty;
            else
                newDist = dist(i,j) + 2 + penalty;
            end
            if newDist < dist(new_i, new_j)
                prev(new_i, new_j, :) = [i, j];
                dist(new_i, new_j) = newDist;
                q.insert([new_i new_j newDist]);
            end
        end
        if i == end_p(1) && j == end_p(2)
            dist_val = dist(end_p(1), end_p(2));
            return;
        end
    end
    dist_val = Inf;
    return;
end

function [dist_val, prev] = dijkstra_map_heur(map, start_p, end_p)
    if is_white(map(start_p(1),start_p(2),:)) || is_white(map(end_p(1),end_p(2),:)) || is_blue(map(start_p(1),start_p(2),:)) || is_blue(map(end_p(1),end_p(2),:)) || is_red(map(start_p(1),start_p(2),:)) || is_red(map(end_p(1),end_p(2),:))
        dist_val = 0;
        prev = 0;
        return;
    end
    height = size(map, 1);
    width = size(map, 2);
    operations = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 -1; -1 1];
    vis = false(height,width);
    for i=1:height
        for j=1:width
            if is_white(map(i,j,:)) || is_blue(map(i,j,:)) || is_red(map(i,j,:))
                vis(i,j) = true;
            end
        end
    end
    prev = -ones(height,width,2);
    dist = Inf(height,width);
    dist(start_p(1), start_p(2)) = 0;
    q = PriorityQueue(3);
    q.insert([start_p(1) start_p(2) 0]);
    while (q.size() ~= 0)
        curr = q.peek();
        i = curr(1);
        j = curr(2);
        value = curr(3);
        q.remove();
        vis(i, j) = true;
        if dist(i, j) < value - round(sqrt((i-end_p(1))^2+(j-end_p(2))^2))
            continue;
        end
        for k=1:8
            if k < 5
                penalty = 0;
            else
                penalty = 0.5;
            end
            new_i = i+operations(k,1);
            new_j = j+operations(k,2);
            if new_i <= 0 || new_i >= height || new_j <= 0 || new_j >= width || vis(new_i, new_j)
                continue;
            end
            if is_green(map(new_i,new_j,:))
                newDist = dist(i,j) + 1 + penalty;
                newCost = newDist + round(sqrt((new_i-end_p(1))^2+(new_j-end_p(2))^2));
            else
                newDist = dist(i,j) + 2 + penalty;
                newCost = newDist + round(sqrt((new_i-end_p(1))^2+(new_j-end_p(2))^2));
            end
            if newDist < dist(new_i, new_j)
                prev(new_i, new_j, :) = [i, j];
                dist(new_i, new_j) = newDist;
                q.insert([new_i new_j newCost]);
            end
        end
        if i == end_p(1) && j == end_p(2)
            dist_val = dist(end_p(1), end_p(2));
            return;
        end
    end
    dist_val = Inf;
    return;
end