% Load the original map
map_original = imread('bw.png');

% Resize
map_im = imresize(map_original, 0.5);

% To rgb
map_rgb = uint8(map_im(:,:,[1 1 1])*255);

imshow(map_rgb)

% Dijkstra

start_p = [104, 128];
end_p = [373, 418];
p = findShortestPath(map_rgb, start_p, end_p);

% Color the original map with the path in red
for i=1:size(p,1)
    map_rgb(p(i,1), p(i,2), :) = [255, 0, 0];
end

imshow(map_rgb)

%% Auxiliary Functions

function val = is_white(rgb)

    if (rgb(1) == 255) && (rgb(2) == 255) && (rgb(3) == 255)
        val = true;
    else
        val = false;
    end

end

function path = findShortestPath(map, start_p, end_p)
    [dist_val, prev] = dijkstra_map(map, start_p, end_p);
    if dist_val == Inf
        path = 0;
        return;
    end
    path = zeros(dist_val+1,2);
    curr_p = end_p;
    it = 1;
    while curr_p(1) ~= -1
        path(it, :) = curr_p;
        curr_p = prev(curr_p(1), curr_p(2), :);
        it = it + 1;
    end
    path = flip(path);
    return;
end

function [dist_val, prev] = dijkstra_map(map, start_p, end_p)
    if is_white(map(start_p(1),start_p(2),:)) || is_white(map(end_p(1),end_p(2),:))
        dist_val = 0;
        prev = 0;
        return;
    end
    height = size(map, 1);
    width = size(map, 2);
    operations = [1 0; -1 0; 0 1; 0 -1];
    vis = false(height,width);
    for i=1:height
        for j=1:width
            if is_white(map(i,j,:))
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
        for k=1:4
            new_i = i+operations(k,1);
            new_j = j+operations(k,2);
            if new_i <= 0 || new_i >= height || new_j <= 0 || new_j >= width || vis(new_i, new_j)
                continue;
            end
            newDist = dist(i,j) + 1;
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