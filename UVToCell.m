function [ cell ] = UVToCell( uv, imageDim )
%UVTOCELL Summary of this function goes here
%   Detailed explanation goes here

    cell = uv;
    cell(:, 2) = imageDim - cell(:,2);


end

