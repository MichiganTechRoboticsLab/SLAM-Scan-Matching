function plotItteration( fig, ogrid, map, scan, T )
%PLOTITTERATION Plot the current hill climbing itteration
%   Pulled out the plotting code for code clarity


    % (eq 7) Current transform between map and scan
    e = T';

    % (eq 8) Scan points transformed to current estimate
    theta = e(3);
    m = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)] ;
    S = m * scan' + repmat( e(1:2), 1, size(scan,1)) ;
    S = S';

    % Convert each point set into image coordinates
    map_x = (map(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1);
    map_y = (map(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2);
    
    scan_x = (scan(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1);
    scan_y = (scan(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2);
    
    S_x = (S(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1);
    S_y = (S(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2);
    
    
    % Plot all the things
    figure(fig)
    clf
    imagesc(imrotate(ogrid.grid,0))
    axis equal
    colormap(bone);
    title('Occupancy Grid');
    hold on;
    plot(map_y, map_x, '.r');
    plot(scan_y, scan_x, '.b');
    plot(S_y, S_x, '.g');    
    legend('Reference', 'Current Scan', 'Registered Scan')

end

