function obstacles = getObstacle(scan, pose, height, res, map)
    theta = pose(3);
    R = [[cos(theta), -sin(theta)];[sin(theta), cos(theta)]];
    B = ~isnan(scan.Cartesian);
    obs_estimate = zeros(nnz(B)/2,2);
    count = 0;
    for i = 1:scan.Count
        if ~isnan(scan.Cartesian(i,1))
            count = count + 1;
            estimate = (-R'*scan.Cartesian(i,:)'+[pose(1);pose(2)])*res;
            estimate([1 2]) = estimate([2 1]);
            estimate(1,1) = height - estimate(1,1);
            obs_estimate(count,:) = estimate;
        end
    end
    obstacles = zeros(size(obs_estimate,1),2);
    for i=1:size(obs_estimate,1)
        [obstacles(i,1), obstacles(i,2)] = nearestStopSemaphore(map, round(obs_estimate(i,1)), round(obs_estimate(i,2)));
    end
    obstacles = unique(obstacles,'rows');
end