function [lookupTable_d, totalRangeX, totalRangeY, minX, minY, maxX, maxY] = genLookupTable(data, searchRadius, lidarStd, pixelSize)
    
    if false
        data = fillLidarData(data, 30, 270);
    end
    
    N = size(data,1);
    
    % Window size in pixels 
    windowSize = ceil(searchRadius / pixelSize);
        
    %range in meters + extra layer for search pixels
    totalRangeX = (max(data(:,1)) - min(data(:,1))) + searchRadius * 6 + 1;
    totalRangeY = (max(data(:,2)) - min(data(:,2))) + searchRadius * 6 + 1;
    
    minX = min(data(:,1)) - (searchRadius * 6 + 0.5);
    minY = min(data(:,2)) - (searchRadius * 6 + 0.5);
    maxX = max(data(:,1)) + (searchRadius * 6 + 0.5);
    maxY = max(data(:,2)) + (searchRadius * 6 + 0.5);
    
    
    maxIdx = ceil(totalRangeX/pixelSize);
    maxIdy = ceil(totalRangeY/pixelSize);
    lookupTable_d = zeros(maxIdx,maxIdy);
    
    % Max value for any pixel
    maxPxValue = normpdf(0, 0, lidarStd);

    dataP = ptToPx(data, pixelSize, totalRangeX, totalRangeY, minX, minY, maxX, maxY);
    
    for idx=1:N
        ctrRow = dataP(idx,1);
        ctrCol = dataP(idx,2);

        
        for i = ctrRow - windowSize : ctrRow + windowSize 
            for j = ctrCol - windowSize : ctrCol + windowSize 
                
                % No bounds checking here, there's a problem above if there
                % is a bounds problem here...
                
                dist = sqrt((i - ctrRow)^2 + (j - ctrCol)^2);
                weight = normpdf(dist * pixelSize, 0, lidarStd);

                lookupTable_d(i, j) = min(lookupTable_d(i,j) + weight, maxPxValue);
                %lookupTable_d(i, j) = min(lookupTable_d(i,j) + weight, 5000);
            end 
        end
    end
    

%      for idx=1:N
%         ctrRow = dataP(idx,1);
%         ctrCol = dataP(idx,2);
% 
%         for ii=-searchRadius:pixelSize:searchRadius
%             for jj=-searchRadius:pixelSize:searchRadius
%                 row = ctrRow + ceil(ii/pixelSize);
%                 col = ctrCol + ceil(jj/pixelSize);
% 
%                 if(row > maxIdx || col > maxIdy)
%                     continue
%                 end
% 
%                 if(row <= 0 || col <= 0)
%                     continue
%                 end
% 
%                 dist = sqrt(ii^2 + jj^2);
%                 weight = normpdf(dist, 0, lidarStd);
% 
%                 lookupTable_d(row, col) = min(lookupTable_d(row,col) + weight, maxPxValue);
%                 %lookupTable_d(row, col) = lookupTable_d(row,col) + weight;
%             end
%         end
%     end
%     
    
end