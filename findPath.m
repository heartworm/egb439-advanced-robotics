function path = findPath(distance_transform, start, goal)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    distance_transform = padarray(distance_transform, [1,1], inf);    
    
%     goal = fliplr(goal);
    history = [start];
    location = start;
    while ~isequal(location, goal)
        disp(location);
        row = location(1);
        col = location(2);
        window = distance_transform(row:row+2, col:col+2);
        window = window + [
            inf 0 inf
            0 0 0 
            inf 0 inf
        ];
        [min_col_val, min_row_inds] = min(window);
        [~, min_col_ind] = min(min_col_val);
        min_row_ind = min_row_inds(min_col_ind);
        movement = [min_row_ind, min_col_ind] - 2;
        
        location = location + movement;
        history = [history; location];
    end
    
    path = history;
    
end

