function sem_select = get_sem(draw)

    % Black and white ist map with two-way streets separated
    map_bw = imread('Images/ist_sep.png');
    % Convert map to logical matrix
    map_bw = ~~map_bw;
    
    % Load stop signs data
    load('DataFiles/sem.mat', 'sem');
    load('DataFiles/sem_map.mat', 'sem_map');

    sem_x_in = [];
    sem_y_in = [];

    fig1 = figure(1);
    imshow(map_bw);
    hold on
    scatter(sem(:,2), sem(:,1), 'red', 'filled');

    disp('Select the desired stops signs. After you select all of them, press the space key.');

    button = 1;
    k = 1;
    while button==1
        [sem_x_in(k), sem_y_in(k), button] = ginput(1);
        k = k + 1;
    end

    % The last value is selected when a button is pressed and it should be
    % ignored
    sem_x_in(end) = [];
    sem_y_in(end) = [];

    close all

    %%% Get closest stop points to the users inputs

    % Find the nearest stop sign to the users inputs
    sem_select = zeros(size(sem_x_in,2),2);

    for i=1:size(sem_x_in,2)
        [sem_select(i,1), sem_select(i,2)] = nearestStopSemaphore(sem_map, round(sem_y_in(i)), round(sem_x_in(i)));
    end

    % If two inputs map to the same stop sigs, eliminate duplicates
    sem_select = unique(sem_select,'rows');

    % Plot the origninal stop signs, the users inputs and the selected stop
    % signs
    if draw == true
        fig2 = figure(1);
        imshow(map_bw)
        hold on
        scatter(sem(:,2), sem(:,1), 'red', 'filled');
        hold on
        scatter(sem_x_in, sem_y_in, 'blue', 'filled');
        hold on
        scatter(sem_select(:,2), sem_select(:,1), 'green', 'filled');
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