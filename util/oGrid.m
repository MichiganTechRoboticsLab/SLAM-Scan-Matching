function [ ogrid ] = oGrid( pointcloud, poses, pixelSize)
%oGrid Generate an occupancy grid from point cloud
%   Detailed explanation goes here

    % Range in meters + border for search pixels
    borderSize = 0.75;
    minX = min(pointcloud(:,1)) - borderSize;
    minY = min(pointcloud(:,2)) - borderSize;
    maxX = max(pointcloud(:,1)) + borderSize;
    maxY = max(pointcloud(:,2)) + borderSize;
        
    % Intialize an empty grid
    RangeX = maxX - minX;
    RangeY = maxY - minY;
    maxIdx = ceil(RangeX/pixelSize);
    maxIdy = ceil(RangeY/pixelSize);
    ogrid.grid = zeros(maxIdx, maxIdy);
        
    
    
%     Ray trace free space
%     beamX = [];
%     beamY = [];
%     beamLength = sqrt( (pointcloud(:,1) - poses(:,1)).^2 + (pointcloud(:,2) - poses(:,2)).^2 );
%     beamDiv    = beamLength / pixelSize;
%         
%     for i = 1:size(pointcloud,1);
%         hit  = pointcloud(i,:); 
%         n = beamDiv(i);
%         
%         beamX = [beamX; linspace(poses(i, 1), hit(1), n)'];
%         beamY = [beamY; linspace(poses(i, 2), hit(2), n)'];
%     end
%     
%     freeSpace = ptToPx([beamX beamY], pixelSize, minX, minY, maxX, maxY);
%     freeSpace = unique(freeSpace, 'rows');
%     
%     i = sub2ind(size(ogrid.grid), freeSpace(:,1), freeSpace(:,2));
%     ogrid.grid(i) = -1;
    
    
    % Fill in small holes
%     H = fspecial('average', [3 3]);
%     freeGrid = imfilter(ogrid.grid,H,'replicate');
%     ogrid.grid(freeGrid < -0.5) = -1;
%     ogrid.grid(i) = -1;
    
    
    % Fill in hits
    hits = ptToPx(pointcloud, pixelSize, minX, minY, maxX, maxY);
    hits = unique(hits, 'rows');
    i = sub2ind(size(ogrid.grid), hits(:,1), hits(:,2));
    ogrid.grid(i) = 1;
    
    
    % Store relivant properties in the ogrid structure
    ogrid.pixelSize = pixelSize;
    ogrid.minX = minX;
    ogrid.minY = minY;
    ogrid.maxX = maxX;
    ogrid.maxY = maxY;

end

