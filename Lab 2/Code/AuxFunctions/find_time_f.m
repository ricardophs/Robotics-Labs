function zero = find_time_f(func, pixel)
    t = linspace(0.7,1.2,3000);
    y = func(t);
    y = y';
    dif = abs(y-pixel);
    n = vecnorm(dif,2,2);
    idx = find(n == min(n));
    left = 10;
    right = 10;
    for i=1:5
        if idx == 1
            zero = t(idx);
            return;
        end
        while idx - left <= 0
            left = round(left / 2);
        end
        while idx + right > 3000
            right = round(right / 2);
        end
        t = linspace(t(idx-left),t(idx+right),3000);
        y = func(t);
        y = y';
        dif = abs(y-pixel);
        n = vecnorm(dif,2,2);
        idx = find(n == min(n)); 
        if size(idx,1) ~= 1
            dt = zeros(size(idx,1),1);
            for j = 1:size(idx,1)
                dt(j,1) = abs(t(idx(j,1))-1);
            end
            idx = find(dt == min(dt));
        end
        left = 10;
        right = 10;
    end
    zero = t(idx);
    return;
end