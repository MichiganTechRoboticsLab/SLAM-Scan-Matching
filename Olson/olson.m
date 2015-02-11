function [ T, mapTable] = olson(map, scan, pixSrch, moveRange, lidarStd, pixRadius, lidarRange, xrange, dx, yrange, dy, thetaRange, dTheta, guess)
N = size(scan,1);

mapTable = genLookupTable(map, pixSrch, lidarRange, moveRange, lidarStd, pixRadius);

maxProb = 0;
T = [0 0];

%TODO
%need to make it so that I don't check zero multiple times

for theta = [0, -thetaRange/2:dTheta:thetaRange/2] + guess(3)
%for theta = 0
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    tempTheta = scan * R;
    
    for x = [0, -xrange/2:dx:xrange/2]  + guess(1)
        for y = [0, -yrange/2:dy:yrange] + guess(2)
            prob = 0;

            temp = tempTheta + repmat([x,y],N,1);
            scanLookup = ptToPx(temp, pixSrch, lidarRange, moveRange, pixRadius);
            scanLookup(scanLookup(:,1) > size(mapTable,1) | scanLookup(:,2) > size(mapTable,2)) = [];
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

if abs(xrange/2 * 0.9) <= (T(1))
    warning('nearing edge of x bound, considering resizing');
end

if abs(yrange/2 * 0.9) <= (T(2))
    warning('nearing edge of y bound, considering resizing');
end

if abs(thetaRange/2 * 0.9) <= (T(3))
    warning('nearing edge of theta bound, considering resizing');
end