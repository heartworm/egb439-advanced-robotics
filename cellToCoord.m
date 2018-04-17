function [ coord ] = cellToCoord(cell, maxIndex, maxCoord)
    assert(size(cell,2) == 2, 'Coordinates not two columns wide');
    
    halfway = maxCoord / 2;
    cellNorm = cell / maxIndex;
    coordFromBl = cellNorm * maxCoord;
    coord = coordFromBl - halfway;
end

