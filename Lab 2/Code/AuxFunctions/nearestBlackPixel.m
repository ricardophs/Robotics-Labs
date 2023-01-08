function [i_out, j_out] = nearestBlackPixel(map, i, j)
    if is_black(map(i,j,:))
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
        if is_black(map(i_curr,j_curr,:))
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