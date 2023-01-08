%% Load map

% Import ist black and white map
map_original = imread('../Images/ist.png');
% Resolution definition in pixels / meters
resolution = 2.8557;
% Convert map to logical matrix
map_original = ~~map_original;

% imtool(map_original);

%% Preprocessing - Separating two way streets

map_im = map_original;
height = size(map_im, 1);
width = size(map_im, 2);

% Convert the original map to a rgb map
map_rgb = uint8(map_original(:,:,[1 1 1])*255);

% Definition of the thickness of the zones
red_thres = 3;
yellow_thres = 3;

% Find the boundaries of the roads and pain the forbiden and yellow zones
for i=1:height
    for j=1:width
        if map_original(i,j) ~= 0
            if (i > 1 && map_original(i-1,j) == 0) || (i < height && map_original(i+1,j) == 0) || (j > 1 && map_original(i,j-1) == 0) || (j < width && map_original(i,j+1) == 0) || (i > 1 && j > 1 && map_original(i-1,j-1) == 0) || (i > 1 && j < width && map_original(i-1,j+1) == 0) || (i < height && j > 1 && map_original(i+1,j-1) == 0) || (i < height && j < width && map_original(i+1,j+1) == 0)
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

% Copy of the original map what will have the separation of the two-way
% streets
map_im = map_original;

% Waypoints where the separations will occur
two_way1 = [[221, 172];
[202, 294];
[205, 314];
[214, 336];
[239, 342];
[262, 344];
[300, 346];
[339, 348];
[371, 349];
[405, 350];
[419, 351]];

two_way2 = [[737, 598];
[709, 597];
[669, 593];
[636, 589];
[592, 584];
[553, 579];
[518, 575];
[485, 571];
[453, 567];
[444, 566]];

% We use a dijkstra algorithm to find the shortest path between the
% waypoints, in a way that this path goes in the middle of the road
% Putting these paths together gives the road separation
for i=1:size(two_way1,1)-1
    start_p = two_way1(i,:);
    end_p = two_way1(i+1,:);
    [~, p1] = findShortestPath(map_rgb, start_p, end_p, 1);
    for j=1:size(p1,1)
        map_im(p1(j,1),p1(j,2)) = 255;
    end
    if i == 1
        writematrix(p1,'../DataFiles/two_way_sep.txt')
    else
        writematrix(p1,'../DataFiles/two_way_sep.txt','WriteMode','append')
    end
end

for i=1:size(two_way2,1)-1
    start_p = two_way2(i,:);
    end_p = two_way2(i+1,:);
    [~, p2] = findShortestPath(map_rgb, start_p, end_p, 1);
    for j=1:size(p2,1)
        map_im(p2(j,1),p2(j,2)) = 255;
    end
    writematrix(p2,'../DataFiles/two_way_sep.txt','WriteMode','append')
end

% Another zone of the map requires different thicknesses for the forbidden
% and yellow zone
map_rgb = uint8(map_original(:,:,[1 1 1])*255);

red_thres = 5;
yellow_thres = 2;

for i=1:height
    for j=1:width
        if map_original(i,j) ~= 0
            if (i > 1 && map_original(i-1,j) == 0) || (i < height && map_original(i+1,j) == 0) || (j > 1 && map_original(i,j-1) == 0) || (j < width && map_original(i,j+1) == 0) || (i > 1 && j > 1 && map_original(i-1,j-1) == 0) || (i > 1 && j < width && map_original(i-1,j+1) == 0) || (i < height && j > 1 && map_original(i+1,j-1) == 0) || (i < height && j < width && map_original(i+1,j+1) == 0)
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

two_way3 = [[647, 830];
[668, 832];
[735, 836];
[779, 838];
[805, 840];
[831, 841];
[858, 843];
[889, 845];
[920, 845];
[930, 838];
[938, 823];
[943, 800];
[946, 775];
[948, 751];
[952, 720];
[955, 688];
[958, 664];
[959, 623]];

for i=1:size(two_way3,1)-1
    start_p = two_way3(i,:);
    end_p = two_way3(i+1,:);
    [~, p3] = findShortestPath(map_rgb, start_p, end_p, 1);
    for j=1:size(p3,1)
        map_im(p3(j,1),p3(j,2)) = 255;
    end
    writematrix(p3,'../DataFiles/two_way_sep.txt','WriteMode','append')
end

two_way4 = [[473, 814];
[425, 810];
[393, 806];
[361, 805];
[329, 802];
[296, 799];
[264, 796];
[232, 793];
[206, 789];
[200, 787];
[197, 784];
[193, 779];
[192, 768];
[193, 730];
[193, 720];
[185, 715];
[182, 711];
[184, 685];
[187, 659];
[189, 631];
[192, 603];
[196, 567]];

for i=1:size(two_way4,1)-1
    start_p = two_way4(i,:);
    end_p = two_way4(i+1,:);
    [~, p4] = findShortestPath(map_rgb, start_p, end_p, 1);
    for j=1:size(p4,1)
        map_im(p4(j,1),p4(j,2)) = 255;
    end
    writematrix(p4,'../DataFiles/two_way_sep.txt','WriteMode','append')
end

two_way6 = [[962, 606];
[962, 598]];

for i=1:size(two_way6,1)-1
    start_p = two_way6(i,:);
    end_p = two_way6(i+1,:);
    [~, p6] = findShortestPath(map_rgb, start_p, end_p, 1);
    for j=1:size(p6,1)
        map_im(p6(j,1),p6(j,2)) = 255;
    end
    writematrix(p6,'../DataFiles/two_way_sep.txt','WriteMode','append')
end

%% Small Adjustments

% These pixels will also be considered as boundaries
pixels = [
[221, 169];
[221, 170];
[221, 171];
[413, 569];
[413, 568];
[413, 567];
[413, 566];
[413, 565];
[413, 564];
[413, 563];
[413, 562];
[413, 561];
[413, 560];
[413, 559];
[413, 558];
[413, 557];
[198, 549];
[198, 548];
[198, 547];
[198, 546];
[198, 545];
[198, 544];
[198, 543];
[198, 542];
];

writematrix(pixels,'../DataFiles/two_way_sep.txt','WriteMode','append')

for i=1:size(pixels,1)
    map_im(pixels(i,1), pixels(i,2)) = 1;
end

map_original(171, 221) = 1;
map_im(171, 221) = 1;

% These pixels will be considered as part of the road
pixels_b = [[420, 564]; [420, 563]; [433, 564]; [433, 565]];

for i=1:size(pixels_b,1)
    map_im(pixels_b(i,1), pixels_b(i,2)) = 0;
    map_original(pixels_b(i,1), pixels_b(i,2)) = 0;
end

imshow(map_im)

%% Saving the final image

% ist_sep is the same as the original map with the two-way street
% separations
imwrite(map_im, '../Images/ist_sep.png');
% ist_corr is the same as the original map with come extra pixels painted
% black
imwrite(map_original, '../Images/ist_corr.png');

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