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