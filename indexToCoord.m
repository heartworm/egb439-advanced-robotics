function [ output ] = indexToCoord(indices)
    indices = fliplr(indices);
    indices(:,2) = 500 - indices(:,2);
    output = (indices / 500 * 1.96) - (1.96/2);
%     output(2) = output(2) * -1;
end

