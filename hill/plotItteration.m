function plotItteration( fig, ogrid, map, scan, T, err)
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
    change_current_figure(fig)
    clf
    imagesc(imrotate(ogrid.grid,0))
    axis equal
    colormap(bone);
    title('Occupancy Grid');
    hold on;
    plot(map_y, map_x, '.r');
    plot(scan_y, scan_x, '.b');
    plot(S_y, S_x, '.g');    
    legend('Reference', 'Current Scan', 'Registered Scan', 'location', 'best')

    
    
    
    if 0

        % Subpixel and gradient function debug plots
        divby = 5;

        % Generate a mesh of points that are finer than the original grid
        xi = (ogrid.minX : ogrid.pixelSize/divby : ogrid.maxX);
        yi = (ogrid.minY : ogrid.pixelSize/divby : ogrid.maxY);
        [xj, yj] = meshgrid(xi, yi);

        % convert the mesh into a list of points
        pts = [reshape(xj,size(xj,1) * size(xj,2), 1) reshape(yj,size(yj,1) * size(yj,2), 1)];

        % Filter the grid at a higher resolution
        v        = ogrid_subpixel( ogrid, pts );
        [dx, dy] = ogrid_gradient( ogrid, pts );


        % Put the data back into a matrix
        v  = reshape(v, length(yi), length(xi))';
        dx = reshape(dx, length(yi), length(xi))';
        dy = reshape(dy, length(yi), length(xi))';


        % Convert each point set into image coordinates
        map_x = (map(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize/divby) * size(v, 1);
        map_y = (map(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize/divby) * size(v, 2);

        scan_x = (scan(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize/divby) * size(v, 1);
        scan_y = (scan(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize/divby) * size(v, 2);

        S_x = (S(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize/divby) * size(v, 1);
        S_y = (S(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize/divby) * size(v, 2);

        % Plot the result   

        change_current_figure(fig + 1)
        clf
        imagesc(imrotate(v,0))
        axis equal
        colormap(bone);
        title('Bilinear filter-d Occupancy Grid');
        hold on;
        plot(map_y, map_x, '.r');
        plot(scan_y, scan_x, '.b');
        plot(S_y, S_x, '.g');    
        legend('Reference', 'Current Scan', 'Registered Scan', 'location', 'bestoutside')


        change_current_figure(fig + 2)
        clf
        imagesc(imrotate(dx,0))
        axis equal
        colormap(bone);
        title('dx');
        hold on;
        plot(map_y, map_x, '.r');
        plot(scan_y, scan_x, '.b');
        plot(S_y, S_x, '.g');    
        legend('Reference', 'Current Scan', 'Registered Scan', 'location', 'bestoutside')


        change_current_figure(fig + 3)
        clf
        imagesc(imrotate(dy,0))
        axis equal
        colormap(bone);
        title('dy');
        hold on;
        plot(map_y, map_x, '.r');
        plot(scan_y, scan_x, '.b');
        plot(S_y, S_x, '.g');    
        legend('Reference', 'Current Scan', 'Registered Scan', 'location', 'bestoutside')

    end
    
    change_current_figure(8)
    clf
    plot(err)
    title('Error.')
    
    drawnow();
end
