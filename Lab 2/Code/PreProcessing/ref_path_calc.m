%% Load map

% Black and white ist map with two-way streets separated
map_original = imread('../Images/ist_sep.png');
% Convert map to logical matrix
map_original = ~~map_original;

% Original black and white ist map
map_no_sep = imread('../Images/ist_corr.png');
% Convert map to logical matrix
map_no_sep = ~~map_no_sep;

%% Zone Definition - Global Map

map_im = map_original;
height = size(map_im, 1);
width = size(map_im, 2);

% Get and rgb version of the original separated map
map_rgb = uint8(map_im(:,:,[1 1 1])*255);

for i=1:height
    for j=1:width
        if map_im(i,j) ~= 0 && map_no_sep(i,j) ~= 0
            if (i > 1 && map_im(i-1,j) == 0) || (i < height && map_im(i+1,j) == 0) || (j > 1 && map_im(i,j-1) == 0) || (j < width && map_im(i,j+1) == 0) || (i > 1 && j > 1 && map_im(i-1,j-1) == 0) || (i > 1 && j < width && map_im(i-1,j+1) == 0) || (i < height && j > 1 && map_im(i+1,j-1) == 0) || (i < height && j < width && map_im(i+1,j+1) == 0)
                % The thickness of the zones is different for two-ways
                % streets
                if (i > 168 && i < 425 && j > 124 && j < 362) || (i > 400 && i < 737 && j > 534 && j < 637) || (i > 952 && i < 972 && j > 596 && j < 610) || (i > 188 && i < 208 && j > 540 && j < 550) || (j > 566 && j < 850 && i > 160 && i < 209) || (j > 760 && j < 850 && i > 200 && i < 496) || (j > 623 && j < 900 && i > 917 && i < 970) || (j > 800 && j < 900 && i > 600 && i < 1000)
                    red_thres = 2;
                    yellow_thres = 1;
                else
                    red_thres = 3;
                    yellow_thres = 3;
                end
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
                map_rgb(i,j,:) = [0,0,255];
            end
        elseif is_black(map_rgb(i,j,:))
            map_rgb(i,j,:) = [0,255,0];
        end
    end
end

% Some extra adjustments

i = 190;
for j = 562:572
    if j == 572
        map_rgb(i,j,:) = [0,255,0];
    else
        map_rgb(i,j,:) = [255,255,0];
    end
end
for j = 562:569
	map_rgb(i,j,:) = [255,255,0];
end
for i = 191:194
    for j = 559:572
        map_rgb(i,j,:) = [0,255,0];
    end
end

i = 954;
for j = 620:626
	map_rgb(i,j,:) = [255,255,0];
end

for i = 955:957
    for j = 616:629
        map_rgb(i,j,:) = [0,255,0];
    end
end

imshow(map_rgb)

%% Zone Definition - Road Separation

% Red pixels are one square around an edge pixel
operations_red = [1 0; 0 1; -1 0; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
% Red pixels are two squares around an edge pixel
operations_yellow = [2 0; 0 2; -2 0; 0 -2; 2 2; 2 -2; -2 2; -2 -2; -1 -2; 2 1; -1 2; 2 -1; -2 1; 1 2; -2 -1; 1 -2];
% Get the pixels that separate the two-way streets
p = readmatrix('../DataFiles/two_way_sep.txt');

% Find the forbidden and yellow zones around the roads separation
for s=1:size(p,1)
    i = p(s,1);
    j = p(s,2);
    for m=1:size(operations_red,1)
        k = operations_red(m,1);
        l = operations_red(m,2);
        if is_red(map_rgb(i+k,j+l,:)) == false && is_blue(map_rgb(i+k,j+l,:)) == false
            map_rgb(i+k,j+l,:) = [255,0,0];
        end
    end
    for m=1:size(operations_yellow,1)
        k = operations_yellow(m,1);
        l = operations_yellow(m,2);
        if is_red(map_rgb(i+k,j+l,:)) == false && is_blue(map_rgb(i+k,j+l,:)) == false
            map_rgb(i+k,j+l,:) = [255,255,0];
        end
    end
    map_rgb(i,j,:) = [0,0,255];
end

imwrite(map_rgb, '../Images/ist_rgb.png');

imshow(map_rgb)

%% Node Definition

% Hand placed nodes that will define the traffic rules
nodes = [[476, 810];
[476, 817];
[139, 530];
[136, 555];
[201, 533];
[195, 533];
[201, 557];
[195, 557];
[191, 557];
[191, 533];
[193, 567];
[199, 567];
[206, 533];
[206, 559];
[414, 541];
[404, 575];
[404, 547];
[410, 548];
[416, 551];
[416, 573];
[410, 576];
[430, 557];
[430, 571];
[452, 562];
[452, 571];
[738, 595];
[738, 602];
[746, 594];
[746, 600];
[754, 600];
[748, 571];
[760, 397];
[761, 372];
[761, 364];
[753, 363];
[427, 351];
[426, 360];
[420, 347];
[420, 355];
[219, 164];
[224, 166];
[956, 623];
[963, 623];
[956, 613];
[956, 589];
[970, 615];
[970, 590];
[959, 614];
[965, 615];
[959, 590];
[965, 590];
[644, 827];
[644, 833];
[1027, 619];
[1030, 596];
[901, 374];
[900, 404];
[592, 580];
[596, 588];
[928, 836];
[931, 841];
[195, 787];
[200, 782];
[204, 323];
[212, 320];
[426, 355];
[423, 351];
];

% Save node data
save('../DataFiles/nodes.mat', 'nodes');

imshow(map_rgb)
hold on
scatter(nodes(:,2), nodes(:,1), 'black', 'filled');
for i=1:size(nodes,1)
    text(nodes(i,2), nodes(i,1)+2, int2str(i))
end

%% Adjacency Matrix Definition using Adjacency List

n_nodes = size(nodes, 1);

% Get hand places edges between nodes
ad_list = readmatrix('../DataFiles/ad_list.txt');

ad_matrix = zeros(n_nodes, n_nodes);

% This matrix will have, for each pixels that belong to the referece path,
% the adjacent nodes that it connects (the first node is the "from" node and the second is the "to" node)
near_nodes_matrix = zeros(height, width, 2);

% The indices will the the lines in the file with the reference path where
% each path fragment that represent a directed edge begins
% Each index corresponds to an edge in the file ad_list
indices = 1;
for i = 1:size(ad_list,1)
    if i == 1
        writematrix(indices,'../DataFiles/indices.txt')
    else
        writematrix(indices,'../DataFiles/indices.txt','WriteMode','append')
    end
    [~, p] = findShortestPath(map_rgb, nodes(ad_list(i,1),:), nodes(ad_list(i,2),:), 1);
    ad_matrix(ad_list(i,1), ad_list(i,2)) = size(p,1)-1;
    for j=1:size(p,1)
        map_rgb(p(j,1),p(j,2),:) = [0,0,0];
        near_nodes_matrix(p(j,1),p(j,2),:) = [ad_list(i,1), ad_list(i,2)];
    end
    indices = indices + size(p,1);
    if i == 1
        writematrix(p,'../DataFiles/paths_between_nodes.txt')
    else
        writematrix(p,'../DataFiles/paths_between_nodes.txt','WriteMode','append')
    end
end

imwrite(map_rgb, '../Images/ist_rgb_black.png');

save('../DataFiles/near_nodes_matrix.mat', 'near_nodes_matrix');
save('../DataFiles/ad_matrix.mat', 'ad_matrix');

imshow(map_rgb)

%% Plot of nodes and paths

imshow(map_rgb)
hold on
scatter(nodes(18:end,2), nodes(18:end,1),35,'MarkerEdgeColor',[0 0 0],...
              'MarkerFaceColor',[1 1 1],...
              'LineWidth',1.5);
for i=18:size(nodes(:,1),1)
    text(nodes(i,2)+3, nodes(i,1)+2, int2str(i))
end

imwrite(map_rgb, '../Images/rgb_nodes_path.png');

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
                newDist = dist(i,j) + 1.5 + penalty;
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