function [ coord ] = UVToCoord(uv, maxIndex, maxCoord)
    assert(size(uv,2) == 2, 'Coordinates not two columns wide');
    
    cell = uv;
    cell(:,2) = maxIndex - cell(:,2);
    
    halfway = maxCoord / 2;
    cellNorm = cell / maxIndex;
    coordFromBl = cellNorm * maxCoord;
    coord = coordFromBl - halfway;
end

