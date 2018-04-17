function [ cell ] = coordToCell(coord, maxIndex, maxCoord)
    assert(size(coord,2) == 2, 'Coordinates not two columns wide');

    halfway = maxCoord / 2;
    coordFromBl = coord + halfway;
    coordNorm = coordFromBl / maxCoord;
    cell = round(coordNorm * maxIndex);
end

