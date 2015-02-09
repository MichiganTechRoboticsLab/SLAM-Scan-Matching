function T = olson(map, scan, pixSrch, moveRange, lidarStd, pixRadius, lidarRange, xrange, dx, yrange, dy, thetaRange, dTheta)
N = size(scan,1);

mapTable = lookupTable(map, pixSrch, lidarRange, moveRange, lidarStd, pixRadius);

maxProb = 0;
T = [0 0];

%TODO
%need to make it so that I don't check zero multiple times

for theta = [0, -thetaRange/2:dTheta:thetaRange/2]
%for theta = 0
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    tempTheta = scan * R;
    
    for x = [0, -xrange/2:dx:xrange/2];
        for y = [0, -yrange/2:dy:yrange];
            prob = 0;

            temp = tempTheta + repmat([x,y],N,1);
            scanLookup = ptToPx(temp, pixSrch, lidarRange, moveRange, pixRadius);


            %need to optomize
            for ii=1:size(scanLookup,1)
                prob = prob + mapTable(scanLookup(ii,1),scanLookup(ii,2));
            end

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