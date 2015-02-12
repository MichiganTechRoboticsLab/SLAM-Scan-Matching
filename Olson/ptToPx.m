function dataP = ptToPx(data, pixelSize, RangeX, RangeY, minX, minY, maxX, maxY)

    scaledData = [ (RangeX + pixelSize )*(data(:,1) - minX)/(maxX - minX) + pixelSize, (RangeY + pixelSize)*(data(:,2) - minY)/(maxY - minY) + pixelSize];
    dataP = ceil(scaledData / pixelSize);
end