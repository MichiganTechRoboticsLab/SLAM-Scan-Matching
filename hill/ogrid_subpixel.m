function [ V ] = ogrid_subpixel( ogrid, pts )
%ogrid_subsample get a subsampled pixel value from an ogrid
%   Uses bilinear filtering to achieve subpixel resolution

    % Convert from meters to pixel location
    x = (pts(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1) + ogrid.pixelSize ;
    y = (pts(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2) + ogrid.pixelSize ;
    
%     RangeX = ogrid.maxX - ogrid.minX;
%     RangeY = ogrid.maxY - ogrid.minY;
%     
%     x = (RangeX + ogrid.pixelSize )*(pts(:,1) - ogrid.minX)/(ogrid.maxX - ogrid.minX) + ogrid.pixelSize;
%     y = (RangeY + ogrid.pixelSize )*(pts(:,2) - ogrid.minY)/(ogrid.maxY - ogrid.minY) + ogrid.pixelSize;
    
%     % Occupancy Grid Debug
%     figure(3)
%     cla
%     imagesc(ogrid.grid)
%     axis equal
%     colormap([1 1 1; 0.5 0.5 0.5; 0 0 0]);
%     
%     hold on;
%     plot(pixels(:,2), pixels(:,1), '+r');
    
    % Retreive interploated value at that point
    V = interp2(ogrid.grid, y, x);
end

