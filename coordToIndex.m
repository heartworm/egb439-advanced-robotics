function [ output ] = coordToIndex(coord)
    halfway = 1.96 / 2;
    coord = coord + halfway;
    output = fliplr(coord / 1.96 * 500);
    output(:,2) = 500 - output(:,2);
    output = round(output);
end

