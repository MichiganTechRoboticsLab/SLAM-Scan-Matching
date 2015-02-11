function dataP = ptToPx(data, pixSrch, lidarRange, moveRange, pixRadius)

N = size(data,1);

%range in meters + extra layer for search pixels
totalRangeX = (max(data(:,1)) - min(data(:,1))) + pixSrch * 2;
totalRangeY = (max(data(:,2)) - min(data(:,2))) + pixSrch * 2;

minX = min(data(:,1)) - pixSrch;
minY = min(data(:,2)) - pixSrch;
maxX = max(data(:,1)) + pixSrch;
maxY = max(data(:,2)) + pixSrch;

scaledData = [ (totalRangeX + pixRadius )*(data(:,1) - minX)/(maxX - minX) + pixRadius (totalRangeY + pixRadius)*(data(:,2) - minY)/(maxY - minY) + pixRadius];
dataP = ceil(scaledData / pixRadius);


end