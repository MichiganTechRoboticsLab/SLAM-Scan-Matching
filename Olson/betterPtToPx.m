function dataP = betterPtToPx(data, pixSrch, lidarRange, moveRange, pixRadius)

N = size(data,1);

%range in meters + extra layer for search pixels
totalRange = lidarRange + (pixSrch + 1) * pixRadius + moveRange;

maxPt = ones(1,2) * ceil(totalRange/pixRadius);%*pixRadius;
minPt = -maxPt;

dataP = ceil(data/pixRadius);%;*pixRadius;
dataP = dataP - repmat(minPt,N,1);

dataP = ceil(dataP);

end