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

lidarStd = p.Results.lidarStd;
searchRadius = p.Results.searchRadius * lidarStd;
pixelSize = p.Results.pixelSize;
xRange = p.Results.xRange;
yRange = p.Results.yRange;
thetaRange = p.Results.thetaRange;
dX = pixelSize;
dY = pixelSize;
dTheta = p.Results.dTheta;

%  [mapP(:,1) mapP(:,2)] = cart2pol(map(:,1),map(:,2));
%  map(mapP(:,2) > 10, :) = [];
%  
%  [scanP(:,1) scanP(:,2)] = cart2pol(scan(:,1),scan(:,2));
%  scan(scanP(:,2) > 10, :) = [];


N = size(scan,1);

fprintf('OLSON: Building LookupTable\n')
[mapTable, mapRangeX, mapRangeY, mapMinX, mapMinY, mapMaxX, mapMaxY] = genLookupTable(map, searchRadius, lidarStd, pixelSize);




%TODO
%need to make it so that I don't check zero multiple times
fprintf('OLSON: Find best fit\n')
fprintf('OLSON: Initial Guess\n')
tmp = guess;
tmp(3) = rad2deg(tmp(3));
fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')
thetas = [0, -thetaRange:dTheta:thetaRange] + guess(3);
data = zeros(size(thetas,2),4);
parfor i = 1:size(thetas,2)
    maxProb = 0;
    T = [0 0];
    theta = thetas(i);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    tempTheta = scan * R;
    
    for x = [0, -xRange:dX:xRange]  + guess(1)
        for y = [0, -yRange:dY:yRange] + guess(2)

            temp = tempTheta + repmat([x,y],N,1);
            scanLookup = ptToPx(temp, pixelSize, mapRangeX, mapRangeY, mapMinX, mapMinY, mapMaxX, mapMaxY);
            scanLookup(scanLookup(:,1) <= 0 | scanLookup(:,2) <= 0 | scanLookup(:,1) > size(mapTable,1) | scanLookup(:,2) > size(mapTable,2), :) = []; scanInd = sub2ind(size(mapTable), scanLookup(:,1), scanLookup(:,2));

            prob = sum(mapTable(scanInd));

            if prob > maxProb
                maxProb = prob;
                
                data(i,:) = [prob x y theta];
            end
        end
    end
end

[p, i] = max(data(:,1));
T = data(i,2:4);

if abs(xRange/2 * 0.9) <= (T(1))
    warning('nearing edge of x bound, considering resizing');
end

if abs(yRange/2 * 0.9) <= (T(2))
    warning('nearing edge of y bound, considering resizing');
end

if abs(thetaRange/2 * 0.9) <= (T(3))
    warning('nearing edge of theta bound, considering resizing');
end