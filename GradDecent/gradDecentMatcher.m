function [ T ] = gradDecentMatcher( guess, scan, map, searchRadius, pixelSize, lidarStd, varargin )
%GRADDECENT This function finds the gradient decent matchup for a scan and
%a map and returns the transformation T

T = [0 0 0];

%first we need to calculate the lookup table
[lookupTable_d, RangeX, RangeY, minX, minY, maxX, maxY] =  genLookupTable(scan, searchRadius, lidarStd, pixelSize);
scanPix = ptToPx(scan, pixelSize, minX, minY, maxX, maxY);

for idx=1:size(scan,1)
    p = ptToPx(scan(idx,:), pixelSize,  minX, minY, maxX, maxY)
    region = findFourNearest( scan(idx,:), pixelSize, RangeX, RangeY, minX, minY, maxX, maxY)
end

end