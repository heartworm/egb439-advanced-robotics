function [ output ] = indexToCoord(indices)
    output = fliplr((indices / 49 * 1.96) - (1.96/2));
end

