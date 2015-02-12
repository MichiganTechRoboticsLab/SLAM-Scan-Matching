function [lookupTable_d, totalRangeX, totalRangeY, minX, minY, maxX, maxY] = genLookupTable(data, searchRadius, lidarStd, pixelSize)
    
    
    N = size(data,1);
    
    %range in meters + extra layer for search pixels
    totalRangeX = (max(data(:,1)) - min(data(:,1))) + searchRadius * 2;
    totalRangeY = (max(data(:,2)) - min(data(:,2))) + searchRadius * 2;
    
    minX = min(data(:,1)) - searchRadius;
    minY = min(data(:,2)) - searchRadius;
    maxX = max(data(:,1)) + searchRadius;
    maxY = max(data(:,2)) + searchRadius;
    
    
    maxIdx = ceil(totalRangeX/pixelSize);
    maxIdy = ceil(totalRangeY/pixelSize);
    lookupTable_d = zeros(maxIdx,maxIdy);
    
    
    if true
        [dataPolar(:,1), dataPolar(:,2)] = cart2pol(data(:,1),data(:,2));
        simga = [deg2rad(.05), lidarStd];
        
        NUM_OF_POINTS = 5000;
        
        for p = 1:N
            randP = mvnrnd(dataPolar(p,:), simga, NUM_OF_POINTS);
            weights = mvnpdf(randP, dataPolar(p,:), simga);
            [ randC(:,1), randC(:,2) ] = pol2cart(randP(:,1), randP(:,2));
            
            randPx = ptToPx(randC, pixelSize, totalRangeX, totalRangeY, minX, minY, maxX, maxY);
            removePx = randPx(:,1) <= 0 | randPx(:,2) <= 0 | randPx(:,1) > maxIdx | randPx(:,2) > maxIdy;
            weights(removePx) = [];
            randPx(removePx, :) = [];
            localLookup = accumarray(randPx, weights, [], [], [], true);
            ind = [];
            [ ind(:,1), ind(:,2), ind(:,3) ] = find(localLookup);
            lookupInd = sub2ind(size(lookupTable_d),ind(:,1), ind(:,2));
            lookupTable_d(lookupInd) = lookupTable_d(lookupInd) + ind(:,3);
        end
    else
        dataP = ptToPx(data, pixelSize, totalRangeX, totalRangeY, minX, minY, maxX, maxY);
        for idx=1:N
            ctrRow = dataP(idx,1);
            ctrCol = dataP(idx,2);
            
            for ii=-searchRadius:pixelSize:searchRadius
                for jj=-searchRadius:pixelSize:searchRadius
                    row = ctrRow + round(ii/pixelSize);
                    col = ctrCol + round(jj/pixelSize);
                    
                    if(row > maxIdx || col > maxIdy)
                        continue
                    end
                    
                    
                    
                    if(row <= 0 || col <= 0)
                        continue
                    end
                    
                    dist = sqrt(ii^2 + jj^2);
                    weight = normpdf(dist,0,lidarStd);
                    
                    lookupTable_d(row, col) = lookupTable_d(row,col) + weight;
                end
            end
        end
    end 
end