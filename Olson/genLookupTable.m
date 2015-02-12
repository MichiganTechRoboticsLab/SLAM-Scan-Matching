function [lookupTable_d, totalRangeX, totalRangeY, minX, minY, maxX, maxY] = genLookupTable(data, searchRadius, lidarStd, pixelSize)
    N = size(data,1);
    
    %range in meters + extra layer for search pixels
    totalRangeX = (max(data(:,1)) - min(data(:,1))) + searchRadius * 2;
    totalRangeY = (max(data(:,2)) - min(data(:,2))) + searchRadius * 2;
    
    minX = min(data(:,1)) - searchRadius;
    minY = min(data(:,2)) - searchRadius;
    maxX = max(data(:,1)) + searchRadius;
    maxY = max(data(:,2)) + searchRadius;
    
    dataP = ptToPx(data, pixelSize, totalRangeX, totalRangeY, minX, minY, maxX, maxY);
    
    maxIdx = ceil(totalRangeX/pixelSize);
    maxIdy = ceil(totalRangeY/pixelSize);
    lookupTable_d = zeros(maxIdx,maxIdy);
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