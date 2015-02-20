function [ T, mapTable] = olson(guess, scan, map, varargin)


    p = inputParser;
    p.addParameter('searchRadius', .3, @(x)isnumeric(x));
    p.addParameter('lidarStd', .01, @(x)isnumeric(x));
    p.addParameter('pixelSize', .03, @(x)isnumeric(x));
    p.addParameter('xRange',  .5, @(x)isnumeric(x));
    p.addParameter('yRange', .5, @(x)isnumeric(x));
    p.addParameter('thetaRange', deg2rad(.9), @(x)isnumeric(x));
    p.addParameter('dTheta',  deg2rad(.25), @(x)isnumeric(x));
    p.parse(varargin{:})

    lidarStd = p.Results.lidarStd;
    searchRadius = p.Results.searchRadius * lidarStd;
    pixelSize = p.Results.pixelSize;
    xRange = p.Results.xRange;
    yRange = p.Results.yRange;
    thetaRange = p.Results.thetaRange;
    dTheta = p.Results.dTheta;


 
    N = size(scan,1);

    % Generate lookup table
    fprintf('OLSON: Building LookupTable... \n')
    lookupTableTic = tic;

    [mapTable, mapRangeX, mapRangeY, mapMinX, mapMinY, mapMaxX, mapMaxY] = genLookupTable(map, searchRadius, lidarStd, pixelSize);

    fprintf('OLSON: LookupTable generation took %4.1f seconds. \n', toc(lookupTableTic))



    % Center search around initial guess
    %fprintf('OLSON: Initial Guess\n')
    %tmp = guess;
    %tmp(3) = rad2deg(tmp(3));
    %fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')


    % Exahusive search for best score
    fprintf('OLSON: Searching for solution... \n')
    searchTic = tic;

    X0 = guess(1);
    Y0 = guess(2);
    
    thetas = [0, -thetaRange:dTheta:thetaRange] + guess(3);
    data = zeros(size(thetas,2), 4);
    parfor i = 1:size(thetas,2)
        maxScore = 0;
        theta = thetas(i);
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        tempTheta = scan * R;

        for x = [0, -xRange:pixelSize:xRange]  + X0
            for y = [0, -yRange:pixelSize:yRange] + Y0

                temp = tempTheta + repmat([x,y],N,1);
                scanLookup = ptToPx(temp, pixelSize, mapRangeX, mapRangeY, mapMinX, mapMinY, mapMaxX, mapMaxY);
                
                % Remove out of bounds points
                scanLookup(scanLookup(:,1) <= 0 | scanLookup(:,2) <= 0 | scanLookup(:,1) > size(mapTable,1) | scanLookup(:,2) > size(mapTable,2), :) = []; 
                
                % Sum all points from lookup table
                scanInd = sub2ind(size(mapTable), scanLookup(:,1), scanLookup(:,2));
                prob = sum(mapTable(scanInd));

                % Keep the highest score 
                if prob > maxScore
                    maxScore = prob;
                    data(i,:) = [prob x y theta];
                end
            end
        end
    end

    [~, i] = max(data(:,1));
    T = data(i,2:4);

    fprintf('OLSON: Search took %4.1f seconds. \n', toc(searchTic))


