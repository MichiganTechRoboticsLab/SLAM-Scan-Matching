function [ dx, dy ] = ogrid_gradient( ogrid, pts  )
%ogrid_gradient get a subsampled gradient value from an ogrid
%   Uses bilinear filtering to achieve subpixel resolution

    % Convert from meters to pixel location
    x = (pts(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX) * size(ogrid.grid, 1) + 1;
    y = (pts(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY) * size(ogrid.grid, 2) + 1;
    
    for i = 1:size(pts,1)
        
        % Edge cases (literally)
        if x(i) <  2 || x(i) >  size(ogrid.grid,1) - 1
            dx(i) = 0;
            dy(i) = 0;
            continue
        end
        
        if y(i) <  2 || y(i) >  size(ogrid.grid, 2) - 1
            dx(i) = 0;
            dy(i) = 0;
            continue
        end
        
        
        % Find the closest pixel in the x axis
        if x(i) - floor(x(i)) > 0.5
            x0 = floor(x(i));
            x1 = x0 + 1;
        else
            x1 = floor(x(i));
            x0 = x1 - 1;
        end


        % Find the closest pixel in the y axis
        if y(i) - floor(y(i)) > 0.5
            y0 = floor(y(i));
            y1 = y0 + 1;
        else
            y1 = floor(y(i));
            y0 = y1 - 1;
        end
        

        % Extract those pixels
        P00 = ogrid.grid(x0, y0);
        P01 = ogrid.grid(x0, y1);
        P10 = ogrid.grid(x1, y0);
        P11 = ogrid.grid(x1, y1);

        % Find gradient for each pixel
        a = y(i) - y0 - 0.5;
        dx(i) = a * (P11 - P01) + (1-a) * (P10 - P00);
        
        a = x(i) - x0 - 0.5;
        dy(i) = a * (P11 - P10) + (1-a) * (P01 - P00);
    end
 
    
end

