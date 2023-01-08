function stops_select = get_stops(draw)

    % Black and white ist map with two-way streets separated
    map_bw = imread('Images/ist_sep.png');
    % Convert map to logical matrix
    map_bw = ~~map_bw;
    
    % Load stop signs data
    load('DataFiles/stops.mat', 'stops');
    load('DataFiles/stops_map.mat', 'stops_map');

    stop_x_in = [];
    stop_y_in = [];

    fig1 = figure(1);
    imshow(map_bw);
    hold on
    scatter(stops(:,2), stops(:,1), 'red', 'filled');

    disp('Select the desired stops signs. After you select all of them, press the space key.');

    button = 1;
    k = 1;
    while button==1
        [stop_x_in(k), stop_y_in(k), button] = ginput(1);
        k = k + 1;
    end

    % The last value is selected when a button is pressed and it should be
    % ignored
    stop_x_in(end) = [];
    stop_y_in(end) = [];

    close all

    %%% Get closest stop points to the users inputs

    % Find the nearest stop sign to the users inputs
    stops_select = zeros(size(stop_x_in,2),2);

    for i=1:size(stop_x_in,2)
        [stops_select(i,1), stops_select(i,2)] = nearestStopSemaphore(stops_map, round(stop_y_in(i)), round(stop_x_in(i)));
    end

    % If two inputs map to the same stop sigs, eliminate duplicates
    stops_select = unique(stops_select,'rows');

    % Plot the origninal stop signs, the users inputs and the selected stop
    % signs
    if draw == true
        fig2 = figure(1);
        imshow(map_bw)
        hold on
        scatter(stops(:,2), stops(:,1), 'red', 'filled');
        hold on
        scatter(stop_x_in, stop_y_in, 'blue', 'filled');
        hold on
        scatter(stops_select(:,2), stops_select(:,1), 'green', 'filled');
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
    
end