%% Load map

% RGB map with the reference path painted in black and edges in blue
map = imread('../Images/ist_rgb_black.png');
map_bw = imread('../Images/ist_corr.png');

%% Get borders

sep = readmatrix('../DataFiles/two_way_sep.txt');

height = size(map,1);
width = size(map,2);

map_edges = zeros(height, width);

% The edges are painted blue
for i = 1:height
    for j = 1:width
        if is_blue(map(i,j,:)) && map_bw(i,j) == 1
            map_edges(i,j) = 1;
        end
    end
end

map_edges = ~map_edges;

imshow(map_edges)

% Save the final result
imwrite(map_edges, '../Images/map_edges.png');

%% Save the borders as list

resolution = 2.8557;

n_zeros = sum(map_edges(:)==0);

borders_list = zeros(n_zeros,2);

counter = 1;
for i = 1:height
    for j = 1:width
        if map_edges(i,j) == 0
            borders_list(counter,1) = j/resolution;
            borders_list(counter,2) = (height - i) / resolution;
            counter = counter + 1;
        end
    end
end

save('../DataFiles/borders.mat', 'borders_list');

%% Auxiliary Functions

function val = is_blue(rgb)

    if (rgb(1) == 0) && (rgb(2) == 0) && (rgb(3) == 255)
        val = true;
    else
        val = false;
    end

end