function dataP = ptToPx(data, pixelSize, minX, minY, maxX, maxY)
% Converts laser hits (meters) to pixel locations

    RangeX = maxX - minX;
    RangeY = maxY - minY;
    
    x = (RangeX + pixelSize )*(data(:,1) - minX)/(RangeX) + pixelSize;
    y = (RangeY + pixelSize )*(data(:,2) - minY)/(RangeY) + pixelSize;
    scaledData = [x, y];
    
    dataP = round(scaledData / pixelSize);
end