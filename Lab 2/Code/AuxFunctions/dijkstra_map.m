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