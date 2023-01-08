%% Load map

% Import ist black and white map with two-way streets separated
map_bw = imread('../Images/ist_sep.png');
% Resolution in pixels / meters
resolution = 2.8557;
% Convert map to logical matrix
map_bw = ~~map_bw;

% RGB map 
map_rgb = imread('../Images/ist_rgb.png');

% RGB map with the reference path in black
map_rgb_w_roads = imread('../Images/ist_rgb_black.png');

%% Semaphores Definition

% Hand picked pixels for the semaphores
sem = [[835, 578];
[835, 605];
[753, 487];
[522, 359];
[420, 456];
[281, 538];
[282, 566];
];

imshow(map_bw)
hold on;
scatter(sem(:,2), sem(:,1), 'red', 'filled');

%% Closest points to the semaphores in the reference path

map_input_pixels = ones(size(map_bw));

sem_corr = zeros(size(sem));

% Find the closest pixel to the hand picked semaphores in the reference path
for i = 1:size(sem,1)
    [sem_corr(i,1), sem_corr(i,2)] = nearestBlackPixel(map_rgb_w_roads, sem(i,1), sem(i,2));
end

imshow(map_bw)
hold on
scatter(sem_corr(:,2), sem_corr(:,1), 'red', 'filled');

for i=1:size(sem_corr,1)
    map_input_pixels(sem_corr(i,1), sem_corr(i,2)) = 0;
end

sem = sem_corr;
sem_map = map_input_pixels;

% Save semaphores coordinates and logical map
save('../DataFiles/sem.mat', 'sem');
save('../DataFiles/sem_map.mat', 'sem_map');

%% Auxiliary Functions
        
function [i_out, j_out] = nearestBlackPixel(map, i, j)
    if map(i,j,:) == 0
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
        if map(i_curr,j_curr,:) == 0
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