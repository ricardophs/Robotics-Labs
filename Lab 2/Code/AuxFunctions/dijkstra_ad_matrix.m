function [dist_val, prev] = dijkstra_ad_matrix(ad_matrix, start_node, end_node)
    n_nodes = size(ad_matrix,1);
    vis = false(n_nodes,1);
    prev = -ones(n_nodes,1);
    dist = Inf(n_nodes,1);
    dist(start_node,1) = 0;
    q = PriorityQueue(2);
    q.insert([start_node 0]);
    while (q.size() ~= 0)
        curr = q.peek();
        node = curr(1);
        value = curr(2);
        q.remove();
        vis(node, 1) = true;
        if dist(node, 1) < value
            continue;
        end
        neighbors = find(ad_matrix(node,:) ~= 0);
        for k=1:size(neighbors,2)
            edge = neighbors(k);
            if vis(edge, 1)
                continue;
            end
            newDist = dist(node, 1) + ad_matrix(node,edge);
            if newDist < dist(edge,1)
                prev(edge, 1) = node;
                dist(edge, 1) = newDist;
                q.insert([edge newDist]);
            end
        end
        if node == end_node
            dist_val = dist(node, 1);
            return;
        end
    end
    dist_val = Inf;
    return;
end