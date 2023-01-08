% Autonomous Cars
close all
clear
clc

addpath('AuxFunctions/');

% Vehicle parameters
L = 2.2;
L_r = 0.566;
L_f = 0.566;
d = 0.64;
r = 0.256;
length_car = 3.332;
width = 1.508;
mass = 810;

% Default energy budgets

energy_default = [1825376;
1341312;
1629600;
616112;
1640128];

% Load map

map = imread('Images/ist_sep.png');
height_map = size(map,1);
width_map = size(map,2);
res = 2.8557;

% Users choise

prompt = "Choose between a simulation with arbitrary initial and final positions with semaphores and stops (enter 0) or a simulation with a default path with integrated energy budget and noise (enter a number between 1 and 4). If you want to view the default paths before chosing one, check the folder 'Images'.\n";
while(1)
    sim_flag = round(input(prompt));
    if sim_flag >= 1 && sim_flag <= 4
        number = sim_flag;
        sim_flag = 1;
        break;
    elseif sim_flag == 0
        break;
    else
        prompt = 'Invalid input. You must a number between 0 and 4:\n';
    end
end

% Get path

if sim_flag == 0
    [path, path_pixels] = path_planning(false, res);
else
    load(sprintf('DataFiles/path_%d.mat', number));
    load(sprintf('DataFiles/final_path_%d.mat', number));
    path = path_m_ref;
    path_pixels = final_path;
    
    % Draw reference path
    map_sep = imread('Images/map_edges.png');
    fig1 = figure(1);
    imshow(map_sep)
    hold on
    plot(path_m_ref(:,1)*res, height_map - path_m_ref(:,2)*res, 'blue', 'LineWidth', 2, 'DisplayName', 'Path')
    hold on
    scatter(final_path(1,2), final_path(1,1), [], [0,0.75,0], 'filled', 'DisplayName', 'Start')
    hold on
    scatter(final_path(end,2), final_path(end,1), [], [0.75,0,0], 'filled', 'DisplayName', 'Finish')
    hold on
    legend('interpreter', 'latex', 'fontsize', 20, 'Location', 'northeast');
    hold off
    try
        w = waitforbuttonpress;
        while w ~= 1
            w = waitforbuttonpress;
        end
        close(fig1)
        disp('Figure closed due to pressed button')
    catch
        disp('Figure closed')
    end
end

% Traffic Lights for sim_flag = 0

if sim_flag == 0
    
    sem_select = get_sem(false);

    load('DataFiles/sem_map.mat')
    n_sem = size(sem_select,1);

    sem_traj = [];
    for i = 1:n_sem
        if ismember(sem_select(i,:), path_pixels, 'rows')
            sem_traj = [sem_traj; sem_select(i,:)];
        end
    end

    avg_signal_time = 7;
    avg_wait_time = 5;

    signal_time = normrnd(avg_signal_time,1,n_sem,1);
    wait_time = normrnd(avg_wait_time,1,n_sem,1);
    curr_signal_time = zeros(n_sem,1);
    curr_wait_time = zeros(n_sem,1);
    sem_state = zeros(1,n_sem);
    sem_states = sem_state;

    map_semafores = zeros(height_map, width_map);
    occ_map_sem = binaryOccupancyMap(map_semafores, res);

    %%% Semaphores LIDAR
    lidar_sem = rangeSensor;
    lidar_sem.HorizontalAngle = [-pi/2 pi/2];
    lidar_sem.Range = [0 30/res];
    
end

% Stops for sim_flag = 0

if sim_flag == 0
    
    stops_select = get_stops(false);

    load('DataFiles/stops_map.mat')
    n_stops = size(stops_select,1);

    stops_traj = [];
    for i = 1:n_stops
        if ismember(stops_select(i,:), path_pixels, 'rows')
            stops_traj = [stops_traj; stops_select(i,:)];
        end
    end

    map_stops = zeros(height_map, width_map);
    occ_map_stops = binaryOccupancyMap(map_stops, res);
    for i=1:size(stops_select,1)
        setOccupancy(occ_map_stops, [stops_select(i,2)/res (height_map-stops_select(i,1))/res], 1)
    end

    % Stops LIDAR
    lidar_stops = rangeSensor;
    lidar_stops.HorizontalAngle = [-pi/2 pi/2];
    lidar_stops.Range = [0 30/res];
    
end

% Simulation parameters

% Simulation variables
t_step = 0.02;

% Initial parameters
last_state.x = path(1,1);
last_state.y = path(1,2);
last_state.theta = path(1,3);
last_state.phi = 0;
last_state.V = 0;
last_state.Ws = 0;

% Controller Parameters
K_v = 0.03; % x -> v (prof: 0.03)
K_i = 1; % y -> omega_s (prof: 1)
K_s = 10; % theta -> omega_s (prof: 100)

if sim_flag == 1
    last_state.x_real = path(1,1);
    last_state.y_real = path(1,2);
    last_state.theta_real = path(1,3);
    last_state.dE = 0;
end

if sim_flag == 1
    % Get energy budget from user
    P0 = 1000;

    prompt = 'For the energy budget, choose 0 for default or 1 for user input:\n';
    while(1)
        pos = input(prompt);
        if pos == 0
            energy_budget = energy_default(round(number));
            break;
        elseif pos == 1
            prompt = 'Input the energy budget (J):\n';
            energy_budget = input(prompt);
            break;
        else
            prompt = 'Invalid input. You must choose 0 or 1:\n';
        end
    end

    total_dist = 0;
    for i = 1:size(path,1)-1
        total_dist = total_dist + sqrt((path(i+1,1)-path(i,1))^2+(path(i+1,2)-path(i,2))^2);
    end
    total_N = round(total_dist / t_step);
    dE_budget = energy_budget / total_N;
    v_max = min(dE_budget / t_step / P0, 5.6);
    v_max = max(2.5, v_max);
else
    v_max = 5;
end

% Limitations
v_max_default = v_max;
w_max = pi/4;        %Velocidade angular máxima
a_max = 5;         %Acelaração máxima (m/s^2)
a_max_default = a_max;

path_x = path(:,1);
path_y = path(:,2);

% Main loop

clear log_state;

counter = 1;
stop_count = 0;
log_state(counter) = last_state;

tolerance = 1;

if sim_flag == 0
    indices = zeros(size(length(path),1),1);
    sem_states = zeros(1,n_sem);
    stop_count = 0;

    for i=1:length(path)
        
        indices(i,1) = counter;

        x = path_x(i);
        y = path_y(i);

        post_e = [last_state.x;last_state.y] - [x;y];
        distance_error = norm(post_e);
        distance_error_corected = distance_error;

        if i==length(path)
         tolerance = 0.2;
        end

        while distance_error_corected > tolerance

            pose = [last_state.x, last_state.y, last_state.theta];
            if n_sem ~= 0
                for s = 1:n_sem
                    if sem_state(1,s) == 0
                        if curr_signal_time(s,1) > signal_time(s,1)
                            sem_state(1,s) = 1;
                            setOccupancy(occ_map_sem, [sem_select(s,2)/res (height_map-sem_select(s,1))/res], 1)
                            curr_signal_time(s,1) = 0;
                            %signal_time(s,1) = exprnd(avg_signal_time);
                            signal_time(s,1) = normrnd(avg_signal_time,1);
                        else
                            curr_signal_time(s,1) = curr_signal_time(s,1) + t_step;
                        end
                    else
                        if curr_wait_time(s,1) > wait_time(s,1)
                            sem_state(1,s) = 0;
                            setOccupancy(occ_map_sem, [sem_select(s,2)/res (height_map-sem_select(s,1))/res], 0)
                            curr_wait_time(s,1) = 0;
                            %wait_time(s,1) = exprnd(avg_wait_time);
                            wait_time(s,1) = normrnd(avg_wait_time,1);
                        else
                            curr_wait_time(s,1) = curr_wait_time(s,1) + t_step;
                        end
                    end
                end
                sem_states = [sem_states; sem_state];
                % Get On Semaphores
                if ~isempty(sem_traj)
                    [ranges, angles] = lidar_sem(pose, occ_map_sem);
                    scan_sem = lidarScan(ranges, angles);
                    sem_on = getObstacle(scan_sem, pose, height_map, res, sem_map);
                    sem_on = intersect(sem_on,sem_traj,'rows');
                    sem_on = [sem_on(:,2)/res (height_map - sem_on(:,1))/res];
                    if ~isempty(sem_on)
                        sem_flag = true;
                    else
                        sem_flag = false;
                    end
                else
                    sem_flag = false;
                end
            else
                sem_flag = false;
            end
            if n_stops ~= 0
                % Get Stops
                if ~isempty(stops_traj)
                    [ranges, angles] = lidar_stops(pose, occ_map_stops);
                    scan_stops = lidarScan(ranges, angles);
                    stops_path = getObstacle(scan_stops, pose, height_map, res, stops_map);
                    stops_path = intersect(stops_path, stops_traj, 'rows');
                    stops_path = [stops_path(:,2)/res (height_map - stops_path(:,1))/res];
                    if ~isempty(stops_path)
                        stops_flag = true;
                    else
                        stops_flag = false;
                    end
                else
                    stops_flag = false;
                end
            else
                stops_flag = false;
            end

            if(sem_flag == true)
                dist2sem = 100000;
                for j=1:size(sem_on,1)
                    aux_dist = norm([last_state.x;last_state.y]-[sem_on(j,1);sem_on(j,2)]);
                    if aux_dist < dist2sem
                       dist2sem = aux_dist;
                    end
                end

                sec2sem = dist2sem/last_state.V;

                if(dist2sem > 2)
                    a_max = last_state.V/sec2sem;
                    v_max = 0;
                end

            elseif a_max ~= a_max_default
                a_max = a_max_default;
                v_max = v_max_default;
            end

            if(stops_flag == true)
                dist2stop = 100000;
                for j=1:size(stops_path,1)
                    aux_dist = norm([last_state.x;last_state.y]-[stops_path(j,1);stops_path(j,2)]);
                    if aux_dist < dist2stop
                       dist2stop = aux_dist;
                       x_stop = stops_path(j,1);
                       y_stop = stops_path(j,2);
                    end
                end

                sec2stop = dist2stop/last_state.V;

                if (last_state.V == 0 && v_max == 0)
                  v_max = v_max_default;
                  RowIdx = find(ismember(stops_traj, [round(height_map-y_stop*res) round(x_stop*res)],'rows'));
                  stops_traj(RowIdx,:) = [];
                  stops_flag = false;
                  stop_count = 1;
                elseif a_max*sec2stop - last_state.V < 0
                  v_max = 0;
                end
            end

            counter = counter + 1;

            if stop_count > 0
                log_state(counter) = last_state;
                if stop_count == 100
                    stop_count = 0;
                else
                    stop_count = stop_count + 1;
                end
            else
                % Orientação 'ideal'
                beta = atan2(-post_e(2),-post_e(1));

                % Erro de orientação
                alpha = beta-(last_state.theta+last_state.phi);
                if alpha > pi
                    alpha = alpha - 2*pi;
                elseif alpha < -pi
                    alpha = alpha + 2*pi;
                end
                K_v=exp(-abs(alpha)*v_max*2);

                next_state.V=v_max*tanh(K_v*distance_error_corected);

                diff_V = (next_state.V - last_state.V)/t_step; % derivada velocidade
                % Restrição da acelaração
                if (a_max < abs(diff_V))
                    next_state.V = last_state.V + sign(diff_V)*a_max*t_step;
                end

                next_state.Ws=w_max*tanh(((1+(K_i*(beta/alpha)))*(tanh(K_v*distance_error_corected)/distance_error_corected)*sin(alpha)+K_s*tanh(alpha)));

                next_state.phi =   last_state.phi+t_step/2*(next_state.Ws+last_state.Ws);
                next_state.theta = last_state.theta+t_step/2*((next_state.V*tan(next_state.phi)/L)+(last_state.V*tan(last_state.phi)/L));
                next_state.x =     last_state.x+t_step/2*((next_state.V*cos(next_state.theta))+(last_state.V*cos(last_state.theta)));
                next_state.y =     last_state.y+t_step/2*((next_state.V*sin(next_state.theta))+(last_state.V*sin(last_state.theta)));

                post_e = [next_state.x;next_state.y]-[x;y];
                distance_error_corected = norm(post_e);

                log_state(counter) = next_state;
                last_state = next_state;
            end

        end

    end
else
    for i=1:length(path)
        
        indices(i,1) = counter;

        x = path_x(i);
        y = path_y(i);

        post_e = [last_state.x;last_state.y] - [x;y];
        distance_error = norm(post_e);
        distance_error_corected = distance_error;

        if i==length(path)
            tolerance = 0.2;
        end

        while distance_error_corected > tolerance

            counter = counter + 1;

            % Orientação 'ideal'
            beta = atan2(-post_e(2),-post_e(1));

            % Erro de orientação
            alpha = beta-(last_state.theta+last_state.phi);
            if alpha > pi
                alpha = alpha - 2*pi;
            elseif alpha < -pi
                alpha = alpha + 2*pi;
            end
            K_v=exp(-abs(alpha)*v_max*2);

            next_state.V=v_max*tanh(K_v*distance_error_corected);

            diff_V = (next_state.V - last_state.V)/t_step; % derivada velocidade
            % Restrição da acelaração
            if (a_max < abs(diff_V))
                next_state.V = last_state.V + sign(diff_V)*a_max*t_step;
            end

            next_state.Ws=w_max*tanh(((1+(K_i*(beta/alpha)))*(tanh(K_v*distance_error_corected)/distance_error_corected)*sin(alpha)+K_s*tanh(alpha)));

            next_state.phi =   last_state.phi+t_step/2*(next_state.Ws+last_state.Ws);
            next_state.theta = last_state.theta+t_step/2*((next_state.V*tan(next_state.phi)/L)+(last_state.V*tan(last_state.phi)/L));
            next_state.x =     last_state.x+t_step/2*((next_state.V*cos(next_state.theta))+(last_state.V*cos(last_state.theta)));
            next_state.y =     last_state.y+t_step/2*((next_state.V*sin(next_state.theta))+(last_state.V*sin(last_state.theta)));
            a = (next_state.V - last_state.V)/t_step;
            next_state.dE = (mass*abs(a)*abs(next_state.V) + P0)*t_step;

            next_state.theta_real = last_state.theta_real+t_step/2*((next_state.V*tan(next_state.phi)/L)+(last_state.V*tan(last_state.phi)/L));
            next_state.x_real =     last_state.x_real+t_step/2*((next_state.V*cos(next_state.theta))+(last_state.V*cos(last_state.theta)));
            next_state.y_real =     last_state.y_real+t_step/2*((next_state.V*sin(next_state.theta))+(last_state.V*sin(last_state.theta)));

            next_state.x = normrnd(next_state.x_real,sqrt(5.6e-2));
            next_state.y = normrnd(next_state.y_real,sqrt(5.6e-2));
            next_state.theta = normrnd(next_state.theta_real,sqrt(5.6e-4));

            post_e = [next_state.x;next_state.y]-[x;y];
            distance_error_corected = norm(post_e);

            log_state(counter) = next_state;
            last_state = next_state;

        end

    end
end

mean_V = mean([log_state.V]);
max_V = max([log_state.V]);
min_V = min([log_state.V]);
fprintf('Mean V = %.3f m/s; Max V = %.3f m/s; Min V = %.3f m/s\n', mean_V, max_V, min_V);
fprintf('Seconds it takes for the car to go through the trajectory: %.2f seconds = %.2f minutes\n', counter*t_step, counter*t_step/60);

if sim_flag == 1
    totalE = sum([log_state.dE]);
    fprintf('Total energy spent: %.2f J\n', totalE);
    if totalE > energy_budget
        fprintf('Energy budget exceeded by %.2f J\n', totalE-energy_budget);
    end
end

% Real time Simulation

n_points = size([log_state.x]',1);

path_p = zeros(n_points,3);

res_resize = 3.9376;
res_total = res*res_resize;

if sim_flag == 0
    path_p(:,1) = [log_state.x]'*res_total;
    path_p(:,2) = [log_state.y]'*res_total;
    path_p(:,3) = [log_state.theta]';
else
    path_p(:,1) = [log_state.x_real]'*res_total;
    path_p(:,2) = [log_state.y_real]'*res_total;
    path_p(:,3) = [log_state.theta_real]';
end

% Load Map
map_bw = imread('Images/map.png');
map_bw = ~map_bw;

map = binaryOccupancyMap(map_bw);

% Start position
x_start = path(1, 1)*res_total;
y_start = path(1, 2)*res_total;

% Finish position
x_end = path(end, 1)*res_total;
y_end = path(end, 2)*res_total;

% Map limits
x_min = min(path(:, 1)')*res_total;
x_max = max(path(:, 1)')*res_total;
y_max = max(path(:, 2)')*res_total;
y_min = min(path(:, 2)')*res_total;

%% Create full screen sized figure
fig2 = figure('units','normalized','outerposition',[0 0 1 1]);
show(map)
axis off
xlim([x_min-70 x_max+70])
ylim([y_min-70 y_max+70])
bar_len_x = (x_max-x_min+140)/10;
bar_len_y = (y_max-y_min+140)/10;
hold on;
offset_x = (x_max-x_min+140)/10;
offset_y = (x_max-x_min+140)/20;
plot([x_min-70+offset_x x_min-70+offset_x x_min-70+offset_x+bar_len_x],[y_min-70+offset_y+bar_len_y y_min-70+offset_y y_min-70+offset_y], 'black', 'LineWidth', 1, 'HandleVisibility', 'off');
hold on;
l1 = sprintf('%1.f m', bar_len_x/res_total);
text(x_min-70+offset_x+offset_y/2,y_min-70+offset_y/2, l1);
hold on;
l2 = sprintf('%1.f m', bar_len_y/res_total);
text(x_min-70+offset_x-offset_x/2,y_min-70+offset_x/2+bar_len_y/2, l2);
hold on;
title('Simulation of the Autonomous Vehicle')
hold on;
% Reference path
plot(path(:, 1)*res_total, path(:, 2)*res_total, 'blue', 'LineWidth', 1, 'DisplayName', 'Reference path');
% Start position
scatter(x_start, y_start, 50, 'x', 'LineWidth', 2, 'MarkerFaceColor', [0 0.75 0], 'MarkerEdgeColor', [0 0.75 0], 'DisplayName', 'Start')
% Finish position
scatter(x_end, y_end, 50, 'x', 'LineWidth', 2, 'MarkerFaceColor', [0.75 0 0], 'MarkerEdgeColor', [0.75 0 0], 'DisplayName', 'Finish')
if sim_flag == 0
    % Draw stop signs
    for s=1:n_stops
        scale = 25;
        t = (1/16:1/8:1)'*2*pi;
        x_pos = cos(t);
        y_pos = sin(t);
        x_pos = scale*[x_pos; x_pos(1)];
        y_pos = scale*[y_pos; y_pos(1)];
        x_pos = stops_select(s,2)*res_resize + x_pos;
        y_pos = (height_map-stops_select(s,1))*res_resize + y_pos;
        fill(x_pos, y_pos, [1 0 0], 'HandleVisibility', 'off');
        text(stops_select(s,2)*res_resize, (height_map-stops_select(s,1))*res_resize, 'S', 'HorizontalAl', 'center', 'color', 'black');
    end
end
legend('interpreter', 'latex', 'fontsize', 20, 'Location', 'Best');
for k = 1:2:size(path_p, 1)

    % Current position and orientation
    x = path_p(k, 1);
    y = path_p(k, 2);
    theta = path_p(k, 3);

    if sim_flag == 0
        % Draw semaphores
        for s=1:n_sem
            if sem_states(k, s) == 1
                scatter(sem_select(s,2)*res_resize, (height_map-sem_select(s,1))*res_resize, 200, 'filled', 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', [0 0 0], 'LineWidth', 0.5, 'HandleVisibility', 'off')
            else
                scatter(sem_select(s,2)*res_resize, (height_map-sem_select(s,1))*res_resize, 200, 'filled', 'MarkerFaceColor', [0 1 0], 'MarkerEdgeColor', [0 0 0], 'LineWidth', 0.5, 'HandleVisibility', 'off')
            end
        end
    end

    % Car representation
    p = patch([x-L_r*res_total-L*res_total/2 x-L_r*res_total-L*res_total/2 x+L*res_total/2+L_f*res_total x+L*res_total/2+L_f*res_total], [y-d*res_total y+d*res_total y+d*res_total y-d*res_total], [0 0 1], 'HandleVisibility', 'off');
    rotate(p, [0 0 1], theta*180/pi, [x y 0])
    
    drawnow;
    delete(p);

end
try
    w = waitforbuttonpress;
    while w ~= 1
        w = waitforbuttonpress;
    end
    close(fig2)
    disp('Figure closed due to pressed button')
catch
    disp('Figure closed')
end

%% Path following

% Create full screen sized figure
fig3 = figure('units','normalized','outerposition',[0 0 1 1]);
show(map)
axis off
xlim([x_min-70 x_max+70])
ylim([y_min-70 y_max+70])
bar_len_x = (x_max-x_min+140)/10;
bar_len_y = (y_max-y_min+140)/10;
hold on;
offset_x = (x_max-x_min+140)/10;
offset_y = (x_max-x_min+140)/20;
plot([x_min-70+offset_x x_min-70+offset_x x_min-70+offset_x+bar_len_x],[y_min-70+offset_y+bar_len_y y_min-70+offset_y y_min-70+offset_y], 'black', 'LineWidth', 1, 'HandleVisibility', 'off');
hold on;
l1 = sprintf('%1.f m', bar_len_x/res_total);
text(x_min-70+offset_x+offset_y/2,y_min-70+offset_y/2, l1);
hold on;
l2 = sprintf('%1.f m', bar_len_y/res_total);
text(x_min-70+offset_x-offset_x/2,y_min-70+offset_x/2+bar_len_y/2, l2);
hold on;
title('Simulation of the Autonomous Vehicle')
hold on;
% Reference path
plot(path(:, 1)*res_total, path(:, 2)*res_total, 'blue', 'LineWidth', 1, 'DisplayName', 'Reference path');
% Start position
scatter(x_start, y_start, 50, 'x', 'LineWidth', 2, 'MarkerFaceColor', [0 0.75 0], 'MarkerEdgeColor', [0 0.75 0], 'DisplayName', 'Start')
% Finish position
scatter(x_end, y_end, 50, 'x', 'LineWidth', 2, 'MarkerFaceColor', [0.75 0 0], 'MarkerEdgeColor', [0.75 0 0], 'DisplayName', 'Finish')
% Total path
plot(path_p(:, 1), path_p(:, 2), 'red', 'LineWidth', 1, 'DisplayName', "Vehicle's path")
hold on;
if sim_flag == 0
    % Draw stop signs
    for s=1:n_stops
        scale = 25;
        t = (1/16:1/8:1)'*2*pi;
        x_pos = cos(t);
        y_pos = sin(t);
        x_pos = scale*[x_pos; x_pos(1)];
        y_pos = scale*[y_pos; y_pos(1)];
        x_pos = stops_select(s,2)*res_resize + x_pos;
        y_pos = (height_map-stops_select(s,1))*res_resize + y_pos;
        fill(x_pos, y_pos, [1 0 0], 'HandleVisibility', 'off');
        text(stops_select(s,2)*res_resize, (height_map-stops_select(s,1))*res_resize, 'S', 'HorizontalAl', 'center', 'color', 'black');
    end
    for s=1:n_sem
        scatter(sem_select(s,2)*res_resize, (height_map-sem_select(s,1))*res_resize, 200, 'filled', 'MarkerFaceColor', [0 1 0], 'MarkerEdgeColor', [0 0 0], 'LineWidth', 0.5, 'HandleVisibility', 'off')
    end
end
legend('interpreter', 'latex', 'fontsize', 20, 'Location', 'best');
hold off;
try
    w = waitforbuttonpress;
    while w ~= 1
        w = waitforbuttonpress;
    end
    close(fig3)
    disp('Figure closed due to pressed button')
catch
    disp('Figure closed')
end
    
% Plot Results

% Path error and theta dot
if sim_flag == 1
    
    x_real = [log_state.x_real]';
    y_real = [log_state.y_real]';
    theta = [log_state.theta_real]';
    x_est = [log_state.x]';
    y_est = [log_state.y]';
    dE = [log_state.dE]';

    energy = zeros(size(path,1),1);

    for i = 1:size(path,1)-1
        energy(i,1) = sum(dE(indices(i,1):indices(i+1,1)))/(indices(i+1,1)-indices(i,1)+1);
    end
    
    dist_point = zeros(size(x_real,1),1);
    for i = 1:size(x_real,1)
        dist_point(i,1) = Inf;
        for j = 1:size(path,1)
            dist_temp = norm([x_real(i,1);y_real(i,1)] - [path(j,1);path(j,2)]);
            if dist_temp < dist_point(i,1)
                dist_point(i,1) = dist_temp;
            end
        end
    end
    
    dist_point_est = zeros(size(x_est,1),1);
    for i = 1:size(x_est,1)
        dist_point_est(i,1) = Inf;
        for j = 1:size(path,1)
            dist_temp = norm([x_est(i,1);y_est(i,1)] - [path(j,1);path(j,2)]);
            if dist_temp < dist_point_est(i,1)
                dist_point_est(i,1) = dist_temp;
            end
        end
    end

    pp_deriv = spline(1:size(x_real,1),theta);
    p_der = fnder(pp_deriv,1);
    deriv = @(t) ppval(p_der,t);
    theta_dot = abs(deriv(1:size(x_real,1)));
    theta_dot = theta_dot';

    fig0 = figure('units','normalized','outerposition',[0 0 1 1]);
    ax(1) = subplot(2,1,1);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(dist_point_est, 'LineWidth', 1);
    ax(2) = subplot(2,1,2);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(dist_point, 'LineWidth', 1)
    set(ax,'Xlim',[1 counter])
    xlabel(ax(1),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    xlabel(ax(2),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(1),'Trajectory error using estimated positions(m)', 'interpreter', 'latex', 'fontsize', 15);
    ylabel(ax(2),'Trajectory error using real positions(m)', 'interpreter', 'latex', 'fontsize', 15);
    arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
    hold off;
    try
        w = waitforbuttonpress;
        while w ~= 1
            w = waitforbuttonpress;
        end
        close(fig0)
        disp('Figure closed due to pressed button')
    catch
        disp('Figure closed')
    end
    
    fig4 = figure('units','normalized','outerposition',[0 0 1 1]);
    ax(1) = subplot(2,1,1);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(dist_point, 'LineWidth', 1);
    ax(2) = subplot(2,1,2);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(theta_dot, 'LineWidth', 1)
    set(ax,'Xlim',[1 counter])
    xlabel(ax(1),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    xlabel(ax(2),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(1),'Trajectory error using real positions(m)', 'interpreter', 'latex', 'fontsize', 15);
    ylabel(ax(2),'$|\dot{\theta}|(rad/s)$', 'interpreter', 'latex', 'fontsize', 20);
    arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
    hold off;
    try
        w = waitforbuttonpress;
        while w ~= 1
            w = waitforbuttonpress;
        end
        close(fig4)
        disp('Figure closed due to pressed button')
    catch
        disp('Figure closed')
    end

    fig5 = figure('units','normalized','outerposition',[0 0 1 1]);
    ax(1) = subplot(2,1,1);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(indices, energy, 'LineWidth', 1);
    ax(2) = subplot(2,1,2);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(theta_dot, 'LineWidth', 1)
    set(ax,'Xlim',[1 counter])
    xlabel(ax(1),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    xlabel(ax(2),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(1), 'Average Energy per Reference Point (J)', 'interpreter', 'latex', 'fontsize', 15);
    ylabel(ax(2),'$|\dot{\theta}|(rad/s)$', 'interpreter', 'latex', 'fontsize', 20);
    arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
    hold off
    try
        w = waitforbuttonpress;
        while w ~= 1
            w = waitforbuttonpress;
        end
        close(fig5)
        disp('Figure closed due to pressed button')
    catch
        disp('Figure closed')
    end
end

% Velocity and theta dot
if sim_flag == 0

    x = [log_state.x]';
    y = [log_state.y]';
    theta = [log_state.theta]';
    v = [log_state.V]';
    
    dist_point = zeros(size(x,1),1);
    for i = 1:size(x,1)
        dist_point(i,1) = Inf;
        for j = 1:size(path,1)
            dist_temp = norm([x(i,1);y(i,1)] - [path(j,1);path(j,2)]);
            if dist_temp < dist_point(i,1)
                dist_point(i,1) = dist_temp;
            end
        end
    end
    
    pp_deriv = spline(1:size(theta,1),theta);
    p_der = fnder(pp_deriv,1);
    deriv = @(t) ppval(p_der,t);
    theta_dot = abs(deriv(1:size(theta,1)));
    theta_dot = theta_dot';
    
    velocity = zeros(size(path,1),1);

    for i = 1:size(path,1)-1
        velocity(i,1) = sum(v(indices(i,1):indices(i+1,1)))/(indices(i+1,1)-indices(i,1)+1);
    end

    fig6 = figure('units','normalized','outerposition',[0 0 1 1]);
    ax(1) = subplot(2,1,1);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(dist_point, 'LineWidth', 1);
    ax(2) = subplot(2,1,2);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(theta_dot, 'LineWidth', 1)
    set(ax,'Xlim',[1 counter])
    xlabel(ax(1),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    xlabel(ax(2),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(1),'Trajectory error (m)', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(2),'$|\dot{\theta}|(rad/s)$', 'interpreter', 'latex', 'fontsize', 20);
    arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
    hold off;
    try
        w = waitforbuttonpress;
        while w ~= 1
            w = waitforbuttonpress;
        end
        close(fig6)
        disp('Figure closed due to pressed button')
    catch
        disp('Figure closed')
    end
    
    fig7 = figure('units','normalized','outerposition',[0 0 1 1]);
    ax(1) = subplot(2,1,1);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(indices, velocity, 'LineWidth', 1);
    ax(2) = subplot(2,1,2);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(theta_dot, 'LineWidth', 1)
    set(ax,'Xlim',[1 counter])
    xlabel(ax(1),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    xlabel(ax(2),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(1), 'Average Velocity per Reference Point (m/s)', 'interpreter', 'latex', 'fontsize', 15);
    ylabel(ax(2),'$|\dot{\theta}|(rad/s)$', 'interpreter', 'latex', 'fontsize', 20);
    arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
    hold off
    try
        w = waitforbuttonpress;
        while w ~= 1
            w = waitforbuttonpress;
        end
        close(fig7)
        disp('Figure closed due to pressed button')
    catch
        disp('Figure closed')
    end

    fig8 = figure('units','normalized','outerposition',[0 0 1 1]);
    ax(1) = subplot(2,1,1);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(v, 'LineWidth', 1);
    ax(2) = subplot(2,1,2);
    set(gca, 'TickLabelInterpreter', 'latex');
    plot(theta_dot, 'LineWidth', 1)
    set(ax,'Xlim',[1 counter])
    xlabel(ax(1),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    xlabel(ax(2),'Interations', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(1), 'Velocity (m/s)', 'interpreter', 'latex', 'fontsize', 20);
    ylabel(ax(2),'$|\dot{\theta}|(rad/s)$', 'interpreter', 'latex', 'fontsize', 20);
    arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))
    hold off
    try
        w = waitforbuttonpress;
        while w ~= 1
            w = waitforbuttonpress;
        end
        close(fig8)
        disp('Figure closed due to pressed button')
    catch
        disp('Figure closed')
    end
end

% Check colisions

map_borders = imread('Images/ist_corr.png');
map_borders = ~map_borders;
occ_borders = binaryOccupancyMap(map_borders);

height_map = size(map_borders,1);
width_map = size(map_borders,2);

if sim_flag == 0
    x = [log_state.x]';
    y = [log_state.y]';
    theta = [log_state.theta]';
else
    x = [log_state.x_real]';
    y = [log_state.y_real]';
    theta = [log_state.theta_real]';
end

n_points = size(x,1);

height = 2*d;
width = L + L_f + L_r;
colisions = 0;
for i = 1:n_points
    c1_x = x(i,1) + ((width / 2) * cos(theta(i,1))) - ((height / 2) * sin(theta(i,1)));
    c1_y = y(i,1) + ((width / 2) * sin(theta(i,1))) + ((height / 2) * cos(theta(i,1)));
    c2_x = x(i,1) - ((width / 2) * cos(theta(i,1))) - ((height / 2) * sin(theta(i,1)));
    c2_y = y(i,1) - ((width / 2) * sin(theta(i,1))) + ((height / 2) * cos(theta(i,1)));
    c3_x = x(i,1) - ((width / 2) * cos(theta(i,1))) + ((height / 2) * sin(theta(i,1)));
    c3_y = y(i,1) - ((width / 2) * sin(theta(i,1))) - ((height / 2) * cos(theta(i,1)));
    c4_x = x(i,1) + ((width / 2) * cos(theta(i,1))) + ((height / 2) * sin(theta(i,1)));
    c4_y = y(i,1) + ((width / 2) * sin(theta(i,1))) - ((height / 2) * cos(theta(i,1)));
    c1_x = c1_x*res;
    c1_y = c1_y*res;
    c2_x = c2_x*res;
    c2_y = c2_y*res;
    c3_x = c3_x*res;
    c3_y = c3_y*res;
    c4_x = c4_x*res;
    c4_y = c4_y*res;
    if getOccupancy(occ_borders, [round(c1_x) round(c1_y)]) == 0
        colisions = colisions + 1;
    end
end

if colisions > 0
    disp('%d colisions happend!', colisions)
else
    disp('No colisions!')
end