function [lookupTable_d, totalRangeX, totalRangeY, minX, minY, maxX, maxY] = genLookupTable(data, searchRadius, lidarStd, pixelSize)
           
% NOTES:
% I think the probablistic math here could be improved... We are using a
% normpdf and summing them for each hit. the pdf is not a probability in
% itself though, it can be greater than 1 for example. We should probably
% be using some normcdf function over the area of each pixel. We could also
% probably not just center the distibution about the center of the hit
% pixel but offset the center of the distibution so that the resulting blur
% is asymetric. This would probably be a more accurate implementation of
% the underlying mathematics. I'm not sure that it would help or hurt
% though.
%
% Additionally, if the number of points is low, we could probably invert
% the problem and itterate over each pixel, summing the probabilities
% contributed by each point in the map which is parallelizable. The current
% implementation works by summing each hit which has data dependancies that
% are not parallelizable. IT is a trade-off, there are fewer itterations
% required in the current form, but it's not parallizable. If the number of
% points approaches the number of pixels, it might be faster to switch.


    % Range in meters + border for search pixels
    borderSize = 0.75;
    minX = min(data(:,1)) - (searchRadius * 3 + borderSize);
    minY = min(data(:,2)) - (searchRadius * 3 + borderSize);
    maxX = max(data(:,1)) + (searchRadius * 3 + borderSize);
    maxY = max(data(:,2)) + (searchRadius * 3 + borderSize);
    
    totalRangeX = maxX - minX;
    totalRangeY = maxY - minY;
    
    % Intialize an empty lookup table
    maxIdx = ceil(totalRangeX/pixelSize);
    maxIdy = ceil(totalRangeY/pixelSize);
    lookupTable_d = zeros(maxIdx,maxIdy);
        
    % Blur window size in pixels 
    windowSize = ceil(searchRadius / pixelSize);
    
    % Max value for any pixel
    maxPxValue = normpdf(0, 0, lidarStd);
    
    % Convert map points to pixels
    dataP = ptToPx(data, pixelSize, totalRangeX, totalRangeY, minX, minY, maxX, maxY);
    
    % Add a blurred normal distibution to the map at each hit location
    for idx=1:size(dataP,1)
        % Pixel location of the hit
        ctrRow = dataP(idx,1);
        ctrCol = dataP(idx,2);

        % Loop through a square of pixels centered around the hit
        for i = ctrRow - windowSize : ctrRow + windowSize 
            for j = ctrCol - windowSize : ctrCol + windowSize 
                
                % No bounds checking here! If there is an error here, then
                % you need to fix the image allocation code above.
                
                % For the center of each pixel in the grid, find the
                % probability that there was a hit there.
                dist = sqrt((i - ctrRow)^2 + (j - ctrCol)^2);
                weight = normpdf(dist * pixelSize, 0, lidarStd);

                % Add the value to the map, with a limit.
                lookupTable_d(i, j) = min(lookupTable_d(i,j) + weight, maxPxValue);
            end 
        end
    end
    
end