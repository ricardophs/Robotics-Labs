function [path_m_ref, final_path] = test(draw, resolution)

    addpath('AuxFunctions/');

    %%% Load map

    % Black and white ist map with two-way streets separated
    map_bw = imread('Images/ist_sep.png');
    % Convert map to logical matrix
    map_bw = ~~map_bw;

    % RGB map 
    map_rgb = imread('Images/ist_rgb.png');

    % RGB map with the reference path in black
    map_rgb_w_roads = imread('Images/ist_rgb_black.png');

    %%% Start and Finish positions

    % Predefined default start and end values

    start_default = [728, 361];
    end_default = [789, 401];

    % The user can use the default initial and final positions or they can
    % introduce new ones
    prompt = 'For initial and final positions, choose 0 for default or 1 for new positions:\n';
    while(1)
        pos = input(prompt);
        if pos == 0
            start_in = start_default;
            end_in = end_default;
            break;
        elseif pos == 1
            fig1 = figure(1);
            imshow(map_bw);
            start_in = round(ginput(1));
            end_in = round(ginput(1));
            start_in([1 2]) = start_in([2 1]);
            end_in([1 2]) = end_in([2 1]);
            close all
            break;
        else
            prompt = 'Invalid input. You must choose 0 or 1:\n';
        end
    end

    % We need to find the nearest pixels in the reference path to the users
    % inputs since that's where the vehicle should go through
    start_p = zeros(1, 2);
    end_p = zeros(1, 2);

    % The reference path is represented in black in the rgb map
    [start_p(1), start_p(2)] = nearestBlackPixel(map_rgb_w_roads, start_in(1), start_in(2));
    [end_p(1), end_p(2)] = nearestBlackPixel(map_rgb_w_roads, end_in(1), end_in(2));

    if ((start_p(1) == end_p(1)) && (start_p(2) == end_p(2)))
        fprintf('Error: The start and final position are the same. Enter new positions.\n')
        return;
    end

    % Plot the users choises and the corrested initial and final positions
    if draw == true
        fig2 = figure(1);
        imshow(map_rgb)
        hold on
        scatter(start_in(2), start_in(1), [], [0,0,0], 'filled', 'DisplayName', 'Start - User input')
        hold on
        scatter(start_p(2), start_p(1), [], [0,0.75,0], 'filled', 'DisplayName', 'Start - Corrected')
        hold on
        scatter(end_in(2), end_in(1), [], [0,0,0], 'filled', 'DisplayName', 'Finish - User input')
        hold on
        scatter(end_p(2), end_p(1), [], [0.75,0,0], 'filled', 'DisplayName', 'Finish - Corrected')
        hold on
        legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
        hold off
        % The figure must be closed or a button must be pressed for the scrip to
        % move on
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
    end

    %%% Load adjacency matrix and adjacent nodes to each black pixel

    % Load reference path data
    near_nodes_matrix = [];
    load('DataFiles/near_nodes_matrix.mat');
    ad_matrix = [];
    load('DataFiles/ad_matrix.mat');

    %%% Transform initial and final position into new nodes

    % Load reference path data
    nodes = [];
    load('DataFiles/nodes.mat');
    indices = readmatrix('DataFiles/indices.txt');
    paths_between_nodes = readmatrix('DataFiles/paths_between_nodes.txt');
    ad_list = readmatrix('DataFiles/ad_list.txt');

    % Check if the start pixel is one of the hand picked nodes
    % If it is not we find the edge it belongs to and the adjacent nodes to it
    [tf_start, index_start]= ismember(start_p, nodes, 'rows');
    if tf_start == 1
        start_node = index_start;
    else
        start_node = size(nodes,1) + 1;
        nodes = [nodes; start_p];
        from_start = near_nodes_matrix(start_p(1), start_p(2), 1);
        to_start = near_nodes_matrix(start_p(1), start_p(2), 2);
    end

    % Check if the end pixel is one of the hand picked nodes
    % If it is not we find the edge it belongs to and the adjacent nodes to it
    [tf_end, index_end]= ismember(end_p, nodes, 'rows');
    if tf_end == 1
        end_node = index_end;
    else
        end_node = size(nodes,1) + 1;
        nodes = [nodes; end_p];
        from_end = near_nodes_matrix(end_p(1), end_p(2), 1);
        to_end = near_nodes_matrix(end_p(1), end_p(2), 2);
    end

    % If the start pixel is a hand picked node and the end pixel is not
    if tf_start == 1 && tf_end ~= 1
        % If the start node is the the same node from where the edge that
        % contains the end node begins, the path is just the portion of that
        % edge from the begining to the end node
        if from_end == start_node
            dijk = 0;
            [~, ad_list_index] = ismember([from_end, to_end], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == end_p(1) && paths_between_nodes(i,2) == end_p(2)
                    new_idx = i;
                    break;
                end
            end
            final_path = paths_between_nodes(idx_1:new_idx,:);
        % If it is not, we still need to process the end new as a new node and
        % use dijstra to finde the shortest path
        else
            dijk = 1;
            ad_list = [ad_list; [from_end, end_node]];
            [~, ad_list_index] = ismember([from_end, to_end], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)-1
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == end_p(1) && paths_between_nodes(i,2) == end_p(2)
                    new_idx = i;
                    break;
                end
            end
            indices = [indices; size(paths_between_nodes,1)+1];
            ad_matrix = [ad_matrix zeros(size(ad_matrix,1),1)];
            ad_matrix = [ad_matrix ; zeros(1,size(ad_matrix,2))];
            ad_matrix(from_end, end_node) = new_idx - idx_1;
            paths_between_nodes = [paths_between_nodes; paths_between_nodes(idx_1:new_idx,:)];
        end
    % This the oposite case to the first one
    elseif tf_start ~= 1 && tf_end == 1
        if to_start == end_node
            dijk = 0;
            [~, ad_list_index] = ismember([from_start, to_start], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == start_p(1) && paths_between_nodes(i,2) == start_p(2)
                    new_idx = i;
                    break;
                end
            end
            final_path = paths_between_nodes(new_idx:idx_2-1,:);
        else
            dijk = 1;
            ad_list = [ad_list; [start_node, to_start]];
            [~, ad_list_index] = ismember([from_start, to_start], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)-1
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == start_p(1) && paths_between_nodes(i,2) == start_p(2)
                    new_idx = i;
                    break;
                end
            end
            indices = [indices; size(paths_between_nodes,1)+1];
            ad_matrix = [ad_matrix zeros(size(ad_matrix,1),1)];
            ad_matrix = [ad_matrix ; zeros(1,size(ad_matrix,2))];
            ad_matrix(start_node,to_start) = idx_2 - new_idx - 1;
            paths_between_nodes = [paths_between_nodes; paths_between_nodes(new_idx:idx_2-1,:)];
        end
    % If both the start and end pixels are not hand picked nodes, we have two
    % options:
    % 1 - The start and end pixel belong to the same edge and the start pixel
    % is in a position before the end pixel - in this case we only need to
    % extract part of that edges
    % 2 - The pixels are not in the same edge or they are and the end pixel is
    % before the start pixel - we need to process the new nodes and use dijkstra
    elseif tf_start ~= 1 && tf_end ~= 1
        if from_start == from_end && to_start == to_end
            dijk = 0;
            [~, ad_list_index] = ismember([from_start, to_start], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            new_idx_1 = 0;
            new_idx_2 = 0;
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == start_p(1) && paths_between_nodes(i,2) == start_p(2)
                    new_idx_1 = i;
                elseif paths_between_nodes(i,1) == end_p(1) && paths_between_nodes(i,2) == end_p(2)
                    new_idx_2 = i;
                end
                if new_idx_1 == 0 && new_idx_2 ~= 0
                    dijk = 2;
                    break;
                elseif new_idx_1 ~= 0 && new_idx_2 ~= 0
                    break;
                end
            end
            if dijk ~= 2
                final_path = paths_between_nodes(new_idx_1:new_idx_2,:);
            end
        else
            dijk = 1;
            ad_list = [ad_list; [start_node, to_start]];
            [~, ad_list_index] = ismember([from_start, to_start], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)-1
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == start_p(1) && paths_between_nodes(i,2) == start_p(2)
                    new_idx = i;
                    break;
                end
            end
            indices = [indices; size(paths_between_nodes,1)+1];
            ad_matrix = [ad_matrix zeros(size(ad_matrix,1),1)];
            ad_matrix = [ad_matrix ; zeros(1,size(ad_matrix,2))];
            ad_matrix(start_node,to_start) = idx_2 - new_idx - 1;
            paths_between_nodes = [paths_between_nodes; paths_between_nodes(new_idx:idx_2-1,:)];
            %%% Process the end node
            ad_list = [ad_list; [from_end, end_node]];
            [~, ad_list_index] = ismember([from_end, to_end], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)-1
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == end_p(1) && paths_between_nodes(i,2) == end_p(2)
                    new_idx = i;
                    break;
                end
            end
            indices = [indices; size(paths_between_nodes,1)+1];
            ad_matrix = [ad_matrix zeros(size(ad_matrix,1),1)];
            ad_matrix = [ad_matrix ; zeros(1,size(ad_matrix,2))];
            ad_matrix(from_end, end_node) = new_idx - idx_1;
            paths_between_nodes = [paths_between_nodes; paths_between_nodes(idx_1:new_idx,:)];
        end
    elseif tf_start == 1 && tf_end == 1
        dijk = 1;
    end

    %%% Find the path

    % Use dijstra to find the shortest paths in some of the cases discussed
    % before
    if dijk ~= 0
        if dijk == 2
            ad_list = [ad_list; [start_node, to_start]];
            [~, ad_list_index] = ismember([from_start, to_start], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)-1
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == start_p(1) && paths_between_nodes(i,2) == start_p(2)
                    new_idx = i;
                    break;
                end
            end
            indices = [indices; size(paths_between_nodes,1)+1];
            ad_matrix = [ad_matrix zeros(size(ad_matrix,1),1)];
            ad_matrix = [ad_matrix ; zeros(1,size(ad_matrix,2))];
            ad_matrix(start_node,to_start) = idx_2 - new_idx - 1;
            paths_between_nodes = [paths_between_nodes; paths_between_nodes(new_idx:idx_2-1,:)];
            %%% Process the end node
            ad_list = [ad_list; [from_end, end_node]];
            [~, ad_list_index] = ismember([from_end, to_end], ad_list, 'rows');
            if ad_list_index ~= size(ad_list,1)-1
                idx_1 = indices(ad_list_index, 1);
                idx_2 = indices(ad_list_index+1, 1);
            else
                idx_1 = indices(ad_list_index, 1);
                idx_2 = size(paths_between_nodes,1);
            end
            for i = idx_1+1:idx_2-1
                if paths_between_nodes(i,1) == end_p(1) && paths_between_nodes(i,2) == end_p(2)
                    new_idx = i;
                    break;
                end
            end
            indices = [indices; size(paths_between_nodes,1)+1];
            ad_matrix = [ad_matrix zeros(size(ad_matrix,1),1)];
            ad_matrix = [ad_matrix ; zeros(1,size(ad_matrix,2))];
            ad_matrix(from_end, end_node) = new_idx - idx_1;
            paths_between_nodes = [paths_between_nodes; paths_between_nodes(idx_1:new_idx,:)];
        end

        [dist, path_nodes] = findShortestPath_matrix(ad_matrix, start_node, end_node);

        if dist == Inf
            fprintf('Error: There if no possible path between the chosen start and finish positions.\n')
            return;
        end

        final_path = zeros(dist,2);

        j = 1;
        for i=1:size(path_nodes,1)-1
            index_ad_list = find(ismember(ad_list, [path_nodes(i,1), path_nodes(i+1,1)], 'rows'));
            if index_ad_list == size(ad_list,1)
                path_index_1 = indices(index_ad_list,1);
                path_index_2 = size(paths_between_nodes,1)+1;
            else
                path_index_1 = indices(index_ad_list,1);
                path_index_2 = indices(index_ad_list+1,1);
            end
            final_path(j:j+path_index_2-path_index_1-1,:) = paths_between_nodes(path_index_1:path_index_2-1,:);
            j = j + path_index_2 - path_index_1 - 1;
        end
    end

    %%% Draw path - Draw the start to finish path, pixel by pixel
    if draw == true
        fig3 = figure(1);
        imshow(map_rgb)
        hold on
        plot(final_path(:,2), final_path(:,1), 'black', 'DisplayName', 'Path', 'LineWidth', 3)
        hold on
        scatter(start_p(2), start_p(1),[],[0,0.75,0],'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), end_p(1),[],[0.75,0,0],'filled', 'DisplayName', 'Finish')
        hold on
        legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
        hold off
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
    end

    t = linspace(0,1,size(final_path(1:5:end,1),1));
    pp = spline(t,[final_path(1:5:end,2),final_path(1:5:end,1)].');
    xy = @(t) ppval(pp,t);

    % This the time values that correspond to the start and end pixels
    t_init = 0;
    t_final = 1;

    n_points = size(final_path,1)/2;

    x = linspace(t_init,t_final,n_points);
    y = xy(x);
    y = y';

    if draw == true
        fig5 = figure(5);
        imshow(map_rgb)
        hold on
        plot(y(:,1), y(:,2), 'black', 'LineWidth', 3, 'DisplayName', 'Spline Interpolator')
        hold on
        scatter(start_p(2), start_p(1), [], [0,0.75,0], 'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), end_p(1), [], [0.75,0,0], 'filled', 'DisplayName', 'Finish')
        hold on
        legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
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

    %%% Checkpoints

    % The checkpoints will the used for the spline interpolation
    % We are using a 10 in 10 pixel discretization
    step_p = 10;
    checkpoints = final_path(1:step_p:end,:);

    first_check = checkpoints(2,:);
    last_check = checkpoints(end,:);

    % We also use two extra checkpoints in the ends to be able to get the
    % spline derivative in the ends
    dif_1 = first_check(1)-start_p(1);
    dif_2 = first_check(2)-start_p(2);

    extra_checkpoints_start = [[start_p(1)-2*dif_1 start_p(2)-2*dif_2]; [start_p(1)-dif_1 start_p(2)-dif_2]];

    dif_1 = last_check(1)-end_p(1);
    dif_2 = last_check(2)-end_p(2);

    extra_checkpoints_end = [[end_p(1)-dif_1 end_p(2)-dif_2]; [end_p(1)-2*dif_1 end_p(2)-2*dif_2]];

    %%% Extended path for spline interpolation
    if draw == true
        fig4 = figure(1);
        imshow(map_rgb)
        hold on
        scatter(checkpoints(:,2), checkpoints(:,1), [], [1,0,0], 'filled', 'DisplayName', 'Checkpoints', 'LineWidth', 3)
        hold on
        scatter(extra_checkpoints_start(:,2), extra_checkpoints_start(:,1), [], [0,1,0], 'filled', 'DisplayName', 'Extra Start Checkpoints', 'LineWidth', 3)
        hold on
        scatter(extra_checkpoints_end(:,2), extra_checkpoints_end(:,1), [], [1,0,0], 'filled', 'DisplayName', 'Extra Finish Checkpoints', 'LineWidth', 3)
        hold on
        scatter(start_p(2), start_p(1), [], [0,0.75,0], 'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), end_p(1), [], [0.75,0,0], 'filled', 'DisplayName', 'Finish')
        hold on
        legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
        hold off
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
    end

    %%% Spline plot

    checkpoints_spline = [extra_checkpoints_start; checkpoints; extra_checkpoints_end];

    % Because a path in 2D is not a function, we need to use a parametrization
    % of the path, using for that a variable t
    t = linspace(0,1,size(checkpoints_spline,1));
    pp = spline(t,[checkpoints_spline(:,2),checkpoints_spline(:,1)].');
    xy = @(t) ppval(pp,t);

    % This the time values that correspond to the start and end pixels
    t_init = find_time_s(xy, [start_p(2), start_p(1)]);
    t_final = find_time_f(xy, [end_p(2), end_p(1)]);

    n_points = size(final_path,1);

    x = linspace(t_init,t_final,n_points);
    y = xy(x);
    y = y';

    if draw == true
        fig5 = figure(5);
        imshow(map_rgb)
        hold on
        scatter(checkpoints(:,2), checkpoints(:,1), [], [1,0,0], 'filled', 'DisplayName', 'Checkpoints', 'LineWidth', 3)
        hold on
        plot(y(:,1), y(:,2), 'black', 'LineWidth', 3, 'DisplayName', 'Spline Interpolator')
        hold on
        scatter(start_p(2), start_p(1), [], [0,0.75,0], 'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), end_p(1), [], [0.75,0,0], 'filled', 'DisplayName', 'Finish')
        hold on
        legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
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

    %%% Theta calculation

    % Use the spline derivaive to get the theta values
    pp_deriv = spline(t,[checkpoints_spline(:,2), -checkpoints_spline(:,1)].');
    p_der = fnder(pp_deriv,1);
    deriv = @(t) ppval(p_der,t);
    deriv_pixels = deriv(x);
    deriv_pixels = deriv_pixels';
    theta = zeros(size(deriv_pixels,1),1);
    for i = 1:size(deriv_pixels,1)
        theta(i,1) = atan2((deriv_pixels(i,2)),deriv_pixels(i,1));
    end

    theta = unwrap(theta);
    if draw == true
        fig6 = figure(1);
        scatter(x, theta, [], [1,0,0], 'filled')
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
    end

    %%% Apply an average filter to the theta values
    
    n_filter = 100;
    filterWindow = ones(n_filter,1) / n_filter;
    theta_avg = imfilter(theta, filterWindow, 'symmetric');

    theta_avg = unwrap(theta_avg);
    if draw == true
        fig7 = figure(1);
        scatter(x, theta_avg, [], [1,0,0], 'filled')
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
    end

    %%% Get spline of averaged data 
    
    pp_deriv = spline(x,theta_avg);
    p_der = fnder(pp_deriv,1);
    deriv = @(t) ppval(p_der,t);
    theta_dot = deriv(x);
    theta_dot = theta_dot';

    if draw == true
        fig7 = figure(1);
        scatter(x, theta_dot, [], [1,0,0], 'filled')
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
    end

    %%% Get average of the theta derivative
    
    total_time = t_final - t_init;
    samples_avg = round(size(final_path,1)/16);
    theta_dot_avg = zeros(samples_avg,1);

    intervals = t_init:total_time/samples_avg:t_final;
    for i=1:samples_avg
        interval_sampled = intervals(i):total_time/samples_avg/samples_avg:intervals(i+1);
        theta_dot_avg(i,1) = trapz(interval_sampled,deriv(interval_sampled))/(total_time/samples_avg);
    end

    if draw == true
        fig7 = figure(1);
        scatter(intervals(1:samples_avg), theta_dot_avg, [], [1,0,0], 'filled')
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
    end

    %%% Sample rate calculation

    samp_rate = zeros(size(theta_dot_avg));
    scale = 1;
    for i = 1:size(theta_dot_avg)
        if abs(theta_dot_avg(i,1)) < size(final_path,1)/60
            samp_rate(i,1) = 3 + round(20*scale*abs(theta_dot_avg(i,1))/max(abs(theta_dot_avg)));
        else
            samp_rate(i,1) = round(20*scale*abs(theta_dot_avg(i,1))/max(abs(theta_dot_avg)));
        end
    end

    if draw == true
        fig7 = figure(1);
        scatter(intervals(1:samples_avg), theta_dot_avg, [], [1,0,0], 'filled')
        hold on
        scatter(intervals(1:samples_avg), samp_rate, [], [0,0,1], 'filled')
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
    end

    %%% Resample and recalculate theta

    x = [];
    for i=1:samples_avg
        interval_sampled = intervals(i):total_time/samples_avg/samp_rate(i):intervals(i+1);
        x = [x; interval_sampled'];
    end

    y = xy(x);
    y = y';

    pp_deriv = spline(t,[checkpoints_spline(:,2), checkpoints_spline(:,1)].');
    p_der = fnder(pp_deriv,1);
    deriv = @(t) ppval(p_der,t);
    deriv_pixels = deriv(x);
    deriv_pixels = deriv_pixels';
    theta = zeros(size(deriv_pixels,1),1);
    for i = 1:size(deriv_pixels,1)
        theta(i,1) = atan2((deriv_pixels(i,2)),deriv_pixels(i,1));
    end

    % theta = unwrap(theta);
    % if draw == true
    %     fig6 = figure(1);
    %     scatter(x, theta, [], [1,0,0], 'filled')
    %     try
    %         w = waitforbuttonpress;
    %         while w ~= 1
    %             w = waitforbuttonpress;
    %         end
    %         close(fig6)
    %         disp('Figure closed due to pressed button')
    %     catch
    %         disp('Figure closed')
    %     end
    % end

    %%% Save trajectory (meters)

    y2 = xy(intervals);
    y2 = y2';

    height = size(map_bw,1);

    if draw == true
        fig7 = figure(1);
        scatter(start_p(2), height - start_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), height - end_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Finish')
        hold on
        scatter(y(:,1), height - y(:,2), [], [1,0,0], 'filled')
        hold on
        scatter(y2(:,1), height - y2(:,2), [], [0,0,0.75], 'filled', 'DisplayName', 'Start')
        hold on
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
    end

    %%% Savitzky-Golay Filter

    windowWidth = 5;
    polynomialOrder = 3;
    X = [];
    Y = [];

    interval_sampled1 = intervals(1):total_time/samples_avg/samp_rate(1):intervals(2);
    interval_sampled2 = intervals(2):total_time/samples_avg/samp_rate(2):intervals(3);
    % interval_sampled1 = interval_sampled1(1:2:end);
    % interval_sampled2 = interval_sampled2(1:2:end);
    interval_sampled = [interval_sampled1'; interval_sampled2'];
    y = xy(interval_sampled);
    y = y';
    smoothX = sgolayfilt(y(:,1), polynomialOrder, windowWidth);
    smoothY = sgolayfilt(y(:,2), polynomialOrder, windowWidth);
    i_init = 1;
    i_end = size(interval_sampled1, 2);
    X = [X; smoothX(i_init:i_end,1)];
    Y = [Y; smoothY(i_init:i_end,1)];

    for i=2:samples_avg-1
        interval_sampled1 = intervals(i-1):total_time/samples_avg/samp_rate(i-1):intervals(i);
        interval_sampled2 = intervals(i):total_time/samples_avg/samp_rate(i):intervals(i+1);
        interval_sampled3 = intervals(i+1):total_time/samples_avg/samp_rate(i+1):intervals(i+2);
    %     interval_sampled1 = interval_sampled1(1:2:end);
    %     interval_sampled2 = interval_sampled2(1:2:end);
    %     interval_sampled3 = interval_sampled3(1:2:end);
        interval_sampled = [interval_sampled1'; interval_sampled2'; interval_sampled3'];
        y = xy(interval_sampled);
        y = y';
        smoothX = sgolayfilt(y(:,1), polynomialOrder, windowWidth);
        smoothY = sgolayfilt(y(:,2), polynomialOrder, windowWidth);
        i_init = size(interval_sampled1, 2) + 1;
        i_end = size(interval_sampled1, 2) + size(interval_sampled2, 2);
        X = [X; smoothX(i_init:i_end,1)];
        Y = [Y; smoothY(i_init:i_end,1)];
    end

    interval_sampled2 = intervals(end-2):total_time/samples_avg/samp_rate(end-2):intervals(end-1);
    interval_sampled3 = intervals(end-1):total_time/samples_avg/samp_rate(end-1):intervals(end);
    % interval_sampled2 = interval_sampled2(1:2:end);
    % interval_sampled3 = interval_sampled3(1:2:end);
    interval_sampled = [interval_sampled2'; interval_sampled3'];
    y = xy(interval_sampled);
    y = y';
    smoothX = sgolayfilt([y(:,1); end_p(2)], polynomialOrder, windowWidth);
    smoothY = sgolayfilt([y(:,2); end_p(1)], polynomialOrder, windowWidth);
    i_init = size(interval_sampled2, 2) + 1;
    i_end = size(interval_sampled2, 2) + size(interval_sampled3, 2);
    X = [X; smoothX(i_init:i_end,1)];
    Y = [Y; smoothY(i_init:i_end,1)];

    if mod(X,2) == 0
        X = [X(1:2:end); X(end)];
        Y = [Y(1:2:end); Y(end)];
    else
        X = X(1:2:end);
        Y = Y(1:2:end);
    end

    if draw == true
        fig7 = figure(1);
        scatter(start_p(2), height - start_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), height - end_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Finish')
        hold on
        scatter(X, height - Y, [], [1,0,0], 'filled')
        hold on
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
    end

    %%% 

    checkpoints = [Y, X];

    first_check = checkpoints(2,:);
    last_check = checkpoints(end,:);

    dif_1 = first_check(1)-start_p(1);
    dif_2 = first_check(2)-start_p(2);

    extra_checkpoints_start = [[start_p(1)-2*dif_1 start_p(2)-2*dif_2]; [start_p(1)-dif_1 start_p(2)-dif_2]];

    dif_1 = last_check(1)-end_p(1);
    dif_2 = last_check(2)-end_p(2);

    extra_checkpoints_end = [[end_p(1)-dif_1 end_p(2)-dif_2]; [end_p(1)-2*dif_1 end_p(2)-2*dif_2]];

    checkpoints_spline = [extra_checkpoints_start; checkpoints; extra_checkpoints_end];

    dist = zeros(size(checkpoints_spline,1),1);
    dist(1,1) = 0;
    sum = dist(1,1);
    for i = 2:size(checkpoints_spline,1)
        dist(i,1) = sqrt((checkpoints_spline(i,1)-checkpoints_spline(i-1,1))*(checkpoints_spline(i,1)-checkpoints_spline(i-1,1))+(checkpoints_spline(i,2)-checkpoints_spline(i-1,2))*(checkpoints_spline(i,2)-checkpoints_spline(i-1,2)));
        sum = sum + dist(i,1);
    end

    t = zeros(size(checkpoints_spline,1),1);
    t(1,1) = 0;
    for i = 2:size(checkpoints_spline,1)
        t(i,1) = t(i-1,1) + dist(i,1)/sum;
    end

    % Because a path in 2D is not a function, we need to use a parametrization
    % of the path, using for that a variable t
    pp = spline(t,[checkpoints_spline(:,2),checkpoints_spline(:,1)].');
    xy = @(t) ppval(pp,t);

    % This the time values that correspond to the start and end pixels
    t_init = find_time_s(xy, [start_p(2), start_p(1)]);
    t_final = find_time_f(xy, [end_p(2), end_p(1)]);

    n_points = 2*size(final_path,1);

    x = linspace(t_init,t_final,n_points);
    y = xy(x);
    y = y';

    if draw == true
        fig5 = figure(5);
        imshow(map_rgb)
        hold on
    %     scatter(checkpoints_spline(:,2), checkpoints_spline(:,1), [], [1,0,0], 'filled', 'DisplayName', 'Checkpoints', 'LineWidth', 3)
    %     hold on
    %     plot(y(:,1), y(:,2), 'black', 'LineWidth', 3, 'DisplayName', 'Spline Interpolator')
    %     hold on
        scatter(y(:,1), y(:,2), [], [1,0,0], 'filled', 'DisplayName', 'Checkpoints', 'LineWidth', 3)
        hold on
        scatter(start_p(2), start_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), end_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Finish')
        hold on
        legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
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

    %%% Theta calculation

    % Use the spline derivaive to get the theta values
    pp_deriv = spline(t,[checkpoints_spline(:,2), -checkpoints_spline(:,1)].');
    p_der = fnder(pp_deriv,1);
    deriv = @(t) ppval(p_der,t);
    deriv_pixels = deriv(x);
    deriv_pixels = deriv_pixels';
    theta = zeros(size(deriv_pixels,1),1);
    for i = 1:size(deriv_pixels,1)
        theta(i,1) = atan2((deriv_pixels(i,2)),deriv_pixels(i,1));
    end

    theta = unwrap(theta);
    if draw == true
        fig6 = figure(1);
        scatter(x, theta, [], [1,0,0], 'filled')
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
    end

    %%% Save trajectory (meters)

    path_m_ref = zeros(size(x,2),3);
    height = size(map_rgb,1);

    if draw == true
        fig5 = figure(5);
        imshow(map_rgb)
        hold on
        scatter(y(:,1), y(:,2), [], [1,0,0], 'filled', 'DisplayName', 'Checkpoints', 'LineWidth', 3)
        hold on
        scatter(start_p(2), start_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Start')
        hold on
        scatter(end_p(2), end_p(1), [], [0,0,0.75], 'filled', 'DisplayName', 'Finish')
        hold on
        legend('interpreter', 'latex', 'fontsize', 15, 'Location', 'northeast');
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
    
    for i = 1:size(x,2)
        path_m_ref(i,:) = [y(i,1) / resolution, (height - y(i,2)) / resolution, theta(i,1)];
    end

    if draw == true
        fig5 = figure(5);
        scatter(path_m_ref(:, 1), path_m_ref(:, 2), [], [1,0,0], 'filled', 'DisplayName', 'Checkpoints', 'LineWidth', 3)
        hold on
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

    save('DataFiles/path_m_ref.mat', 'path_m_ref');

end