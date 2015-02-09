clear
clc
close all

lidarRange = 30;

N = 50;
r = rand(N,1)*lidarRange;
theta = rand(50,1)*2*pi;

map = [r.*cos(theta) r.*sin(theta)];
scan = map + repmat([4,-1],N,1);

mapTable = betterLookupTable(map);

maxProb = 0;
T = [];
for idxX = -5:0.25:5;
    for idxY = -5:0.25:5;
        fprintf('testing %d and %d\n', idxX,idxY);
        
        temp = scan + repmat([idxX,idxY],N,1);
        scanTable = lookupTable(temp);
        
        prob = sum(sum(scanTable.*mapTable));
        if prob > maxProb
            maxProb = prob;
            T = [idxX idxY];
            fprintf('found %d and %d\n',idxX,idxY);
        end
        
        fprintf('\n', idxX,idxY);
    end
end