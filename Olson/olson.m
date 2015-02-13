function [ T, mapTable] = olson(guess, scan, map, varargin)


p = inputParser;
p.addParameter('searchRadius', .3, @(x)isnumeric(x));
p.addParameter('lidarStd', .01, @(x)isnumeric(x));
p.addParameter('pixelSize', .03, @(x)isnumeric(x));
p.addParameter('xRange',  .5, @(x)isnumeric(x));
p.addParameter('yRange', .5, @(x)isnumeric(x));
p.addParameter('thetaRange', deg2rad(.9), @(x)isnumeric(x));
p.addParameter('dX', .01, @(x)isnumeric(x));
p.addParameter('dY',  .01, @(x)isnumeric(x));
p.addParameter('dTheta',  deg2rad(.25), @(x)isnumeric(x));
p.parse(varargin{:})

searchRadius = p.Results.searchRadius;
lidarStd = p.Results.lidarStd;
pixelSize = p.Results.pixelSize;
xRange = p.Results.xRange;
yRange = p.Results.yRange;
thetaRange = p.Results.thetaRange;
dX = p.Results.dX;
dY = p.Results.dY;
dTheta = p.Results.dTheta;

N = size(scan,1);

fprintf('OLSON: Building LookupTable\n')
[mapTable, mapRangeX, mapRangeY, mapMinX, mapMinY, mapMaxX, mapMaxY] = genLookupTable(map, searchRadius, lidarStd, pixelSize);

maxProb = 0;
T = [0 0];

%TODO
%need to make it so that I don't check zero multiple times
fprintf('OLSON: Find best fit\n')
fprintf('OLSON: Initial Guess\n')
fprintf(['\t' repmat('%g\t', 1, size(guess, 2)) '\n'], guess')
for theta = [0, -thetaRange/2:dTheta:thetaRange/2] + guess(3)
%for theta = 0
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    tempTheta = scan * R;
    
    for x = [0, -xRange/2:dX:xRange/2]  + guess(1)
        for y = [0, -yRange/2:dY:yRange] + guess(2)
%             prob = 0;

            temp = tempTheta + repmat([x,y],N,1);
            scanLookup = ptToPx(temp, pixelSize, mapRangeX, mapRangeY, mapMinX, mapMinY, mapMaxX, mapMaxY);
%           outOfBounds = scanLookup(scanLookup(:,1) > size(mapTable,1) | scanLookup(:,2) > size(mapTable,2) | scanLookup(:,1) <= 0 | scanLookup(:,2) <= 0, :);
%             outOfBounds = scanLookup(scanLookup(:,1) <= 0 | scanLookup(:,2) <= 0, :);
%             if ~isempty(outOfBounds)
%                outOfBounds
%                fprintf('Array out of bounds\n')
%             end
            scanLookup(scanLookup(:,1) <= 0 | scanLookup(:,2) <= 0 | scanLookup(:,1) > size(mapTable,1) | scanLookup(:,2) > size(mapTable,2), :) = [];
            scanInd = sub2ind(size(mapTable), scanLookup(:,1), scanLookup(:,2));

            prob = sum(mapTable(scanInd));

            if prob > maxProb
                maxProb = prob;
                T = [x y theta];
                %fprintf('found %d and %d\n',idxX,idxY);
            end

            %fprintf('\n', idxX,idxY);
        end
    end
end

if abs(xRange/2 * 0.9) <= (T(1))
    warning('nearing edge of x bound, considering resizing');
end

if abs(yRange/2 * 0.9) <= (T(2))
    warning('nearing edge of y bound, considering resizing');
end

if abs(thetaRange/2 * 0.9) <= (T(3))
    warning('nearing edge of theta bound, considering resizing');
end