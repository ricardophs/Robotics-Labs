%% Load map

map = imread('Images/ist_sep.png');
map_edges = imread('Images/map_edges.png');
map_edges = ~map_edges;

height = size(map,1);
width = size(map,2);

map_semafores = zeros(height, width);

res = 2.86;

occ_map_sem = binaryOccupancyMap(map_semafores, res);
occ_map_edges = binaryOccupancyMap(map_edges, res);

sem_map = [];
load('DataFiles/sem_map.mat');

%%

sem = [];
load('DataFiles/sem.mat');

x = 538/res;
y = (height-250)/res;

figure(1)
show(occ_map_edges)
hold on
scatter(x, y);
hold on
scatter(sem(:,2)/res, (height-sem(:,1))/res);

%%

sem = [];
load('DataFiles/sem.mat');

n_sem = size(sem,1);

avg_signal_time = 5;
avg_wait_time = 3;

signal_time = exprnd(avg_signal_time,n_sem,1);
wait_time = exprnd(avg_wait_time,n_sem,1);
curr_signal_time = zeros(n_sem,1);
curr_wait_time = zeros(n_sem,1);
sem_state = zeros(n_sem,1);

t_init = 0;
t_final = 30;
dt = 0.1;

t = t_init;
figure('units','normalized','outerposition',[0 0 1 1])
while(t < t_final)
    imshow(map)
    hold on;
    for i = 1:n_sem
        if sem_state(i,1) == 0
            scatter(sem(i,2), sem(i,1), 'filled', 'MarkerFaceColor', [0 1 0])
            hold on;
            if curr_signal_time(i,1) > signal_time(i,1)
                sem_state(i,1) = 1;
                curr_signal_time(i,1) = 0;
                signal_time(i,1) = exprnd(avg_signal_time);
            else
                curr_signal_time(i,1) = curr_signal_time(i,1) + dt;
            end
        else
            scatter(sem(i,2), sem(i,1), 'filled', 'MarkerFaceColor', [1 0 0])
            hold on;
            if curr_wait_time(i,1) > wait_time(i,1)
                sem_state(i,1) = 0;
                curr_wait_time(i,1) = 0;
                wait_time(i,1) = exprnd(avg_wait_time);
            else 
                curr_wait_time(i,1) = curr_wait_time(i,1) + dt;
            end
        end
    end
    light;
    drawnow;
    hold off;
    t = t + dt;
end

%%

sem = [];
load('DataFiles/sem.mat');

x = 538/res;
y = (height-250)/res;
theta = -1.6;

pose = [x, y, theta];

lidar = rangeSensor;
lidar.HorizontalAngle = [-pi/2 pi/2];
lidar.Range = [0 60/res];

Vehiclepose = pose;

t_init = 0;
t_final = 30;
dt = 0.1;

avg_signal_time = 5;
avg_wait_time = 3;

signal_time = exprnd(avg_signal_time);
wait_time = exprnd(avg_wait_time);

t = 0;
curr_signal_time = 0;
figure('units','normalized','outerposition',[0 0 1 1])
while(t < t_final)
    [ranges, angles] = lidar(Vehiclepose, occ_map_sem);
    scan = lidarScan(ranges, angles);
    plot(scan);
    light;
    drawnow;
    hold off;
    sem_flag = 0;
    if curr_signal_time >= signal_time
        for i=1:size(sem,1)
            setOccupancy(occ_map_sem, [sem(i,2)/res (height-sem(i,1))/res], 1)
        end
        curr_wait_time = 0;
        while(curr_wait_time < wait_time)
            [ranges, angles] = lidar(Vehiclepose, occ_map_sem);
            scan = lidarScan(ranges, angles);
            R = [[cos(theta), -sin(theta)];[sin(theta), cos(theta)]];
            B = ~isnan(scan.Cartesian);
            if (nnz(B) ~= 0)
                sem_flag = 1;
            else
                sem_flag = 0;
            end
            sem_estimate = zeros(nnz(B)/2,2);
            count = 0;
            for i = 1:scan.Count
                if ~isnan(scan.Cartesian(i,1))
                    count = count + 1;
                    estimate = (-R'*scan.Cartesian(i,:)'+[x;y])*res;
                    estimate([1 2]) = estimate([2 1]);
                    estimate(1,1) = height - estimate(1,1);
                    sem_estimate(count,:) = estimate;
                end
            end
            sem_on = zeros(size(sem_estimate,1),2);
            for i=1:size(sem_estimate,1)
                [sem_on(i,1), sem_on(i,2)] = nearestSemaphore(sem_map, round(sem_estimate(i,1)), round(sem_estimate(i,2)));
            end
            sem_on = unique(sem_on,'rows');
            plot(scan)
            light;
            drawnow;
            hold off;
            curr_wait_time = curr_wait_time + dt;
            t = t + dt;
        end
        for i=1:size(sem,1)
            setOccupancy(occ_map_sem, [sem(i,2)/res (height-sem(i,1))/res], 0)
        end
        signal_time = exprnd(avg_signal_time);
        wait_time = exprnd(avg_wait_time);
        curr_signal_time = 0;
    end
    curr_signal_time = curr_signal_time + dt;
    t = t + dt;
end

%% Auxiliary Functions

function [i_out, j_out] = nearestSemaphore(map, i, j)
    if map(i,j,:) == 0
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
        if map(i_curr,j_curr,:) == 0
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