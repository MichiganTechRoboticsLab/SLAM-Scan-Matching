function lookupTable_d = genLookupTable(data, pixSrch, lidarRange, moveRange, lidarStd, pixRadius)



N = size(data,1);

totalRangeX = (max(data(:,1)) - min(data(:,1))) + pixSrch * 2;
totalRangeY = (max(data(:,2)) - min(data(:,2))) + pixSrch * 2;

dataP = ptToPx(data,pixSrch, lidarRange, moveRange, pixRadius);

maxIdx = ceil(totalRangeX/pixRadius);
maxIdy = ceil(totalRangeY/pixRadius);
lookupTable_d = zeros(maxIdx,maxIdy);
for idx=1:N
%     idx
    ctrRow = dataP(idx,1);
    ctrCol = dataP(idx,2);
    
    for ii=-pixSrch:pixRadius:pixSrch
        for jj=-pixSrch:pixRadius:pixSrch
            row = ctrRow + round(ii/pixRadius);
            col = ctrCol + round(jj/pixRadius);

%             class row
%             class col
            
            if(row > maxIdx || col > maxIdy)
                continue
            end
            
            
            
            if(row <= 0 || col <= 0)
                continue
            end
            
            dist = sqrt(ii^2 + jj^2);
            weight = normpdf(dist,0,lidarStd);
            %weight = 1;

            lookupTable_d(row, col) = lookupTable_d(row,col) + weight;
        end
    end
end

% %normalize to actually make it a probablity distribution
% totalSum = sum(sum(lookupTable_d));
% lookupTable_d = lookupTable_d / totalSum;

end