function lookupTable = betterLookupTable(data, pixSrch, lidarRange, moveRange, lidarStd, pixRadius)

N = size(data,1);

totalRange = lidarRange + (pixSrch + 1) * pixRadius + moveRange;

dataP = betterPtToPx(data,pixSrch, lidarRange, moveRange, pixRadius);

maxIdx = ceil(totalRange/pixRadius) * 2;
lookupTable = zeros(maxIdx,maxIdx);
for idx=1:N
    ctrRow = dataP(idx,1);
    ctrCol = dataP(idx,2);
    
    for ii=-pixSrch:pixSrch
        for jj=-pixSrch:pixSrch
            row = ctrRow + ceil(ii*pixRadius);
            col = ctrCol + ceil(jj*pixRadius);

            %dist = sqrt(ii^2 + jj^2) * pixRadius;
            %weight = normpdf(dist,0,lidarStd);
            weight = 1;

            lookupTable(row, col) = lookupTable(row,col) + weight;
        end
    end
end

%normalize to actually make it a probablity distribution
totalSum = sum(sum(lookupTable));
lookupTable = lookupTable / totalSum;

end