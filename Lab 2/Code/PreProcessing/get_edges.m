%% Load map

% RGB map with the reference path painted in black and edges in blue
map = imread('../Images/ist_rgb_black.png');

%% Get edges

height = size(map,1);
width = size(map,2);

map_edges = zeros(height, width);

% The edges are painted blue
for i = 1:height
    for j = 1:width
        if is_blue(map(i,j,:))
            map_edges(i,j) = 1;
        end
    end
end

map_edges = ~map_edges;

imshow(map_edges)

% Save the final result
imwrite(map_edges, '../Images/map_edges.png');

%% Auxiliary Functions

function val = is_blue(rgb)

    if (rgb(1) == 0) && (rgb(2) == 0) && (rgb(3) == 255)
        val = true;
    else
        val = false;
    end

end