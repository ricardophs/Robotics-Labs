function [dist_val, path] = findShortestPath_matrix(ad_matrix, start_node, end_node)
    [dist_val, prev] = dijkstra_ad_matrix(ad_matrix, start_node, end_node);
    if dist_val == Inf
        path = 0;
        return;
    end
    path = zeros(size(ad_matrix,1),1);
    curr_node = end_node;
    it = 1;
    while curr_node ~= -1
        path(it, 1) = curr_node;
        curr_node = prev(curr_node);
        it = it + 1;
    end
    path(it:size(ad_matrix,1),:) = [];
    path = flip(path);
    return;
end