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



N = size(scan,1);

% Generate lookup table
fprintf('OLSON: Building LookupTable... \n')
lookupTableTic = tic;

[mapTable, mapRangeX, mapRangeY, mapMinX, mapMinY, mapMaxX, mapMaxY] = genLookupTable(map, searchRadius, lidarStd, pixelSize);

fprintf('OLSON: LookupTable generation took %4.1f seconds. \n', toc(lookupTableTic))



% Center search around initial guess
fprintf('OLSON: Initial Guess\n')
tmp = guess;
tmp(3) = rad2deg(tmp(3));
fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')


% Exahusive search for best score
fprintf('OLSON: Searching for Solution... \n')
searchTic = tic;

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

[~, i] = max(data(:,1));
T = data(i,2:4);


fprintf('OLSON: Search took %4.1f seconds. \n', toc(lookupTableTic))


% Misc Warnings
if abs(xRange * 0.9) <= T(1)
    warning('OLSON: Nearing edge of x bound, considering resizing');
end

if abs(yRange * 0.9) <= T(2)
    warning('OLSON: Nearing edge of y bound, considering resizing');
end

if abs(thetaRange * 0.9) <= T(3)
    warning('OLSON: Nearing edge of theta bound, considering resizing');
end
