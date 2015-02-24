function [ dx, dy ] = ogrid_gradient( ogrid, pts  )
%ogrid_gradient get a subsampled gradient value from an ogrid
%   Uses bilinear filtering to achieve subpixel resolution

    % Convert from meters to pixel location
    %x = (pts(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX) * size(ogrid.grid, 1) + 1;
    %y = (pts(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY) * size(ogrid.grid, 2) + 1;
    
    x = (pts(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1);
    y = (pts(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2);
    
    
    width = 1;
    for i = 1:size(pts,1)
        
        % Edge cases (literally)
        if x(i) <  width*2 || x(i) >  size(ogrid.grid,1) - width
            dx(i) = 0;
            dy(i) = 0;
            continue
        end
        
        if y(i) <  width*2 || y(i) >  size(ogrid.grid, 2) - width
            dx(i) = 0;
            dy(i) = 0;
            continue
        end
        
        
        % Find the closest pixel in the x axis
        if x(i) - floor(x(i)) > width/2
            x0 = floor(x(i));
            x1 = x0 + width;
        else
            x1 = floor(x(i));
            x0 = x1 - width;
        end


        % Find the closest pixel in the y axis
        if y(i) - floor(y(i)) > width/2
            y0 = floor(y(i));
            y1 = y0 + width;
        else
            y1 = floor(y(i));
            y0 = y1 - width;
        end
        

        % Extract those pixels
        P00 = ogrid.grid(x0, y0);
        P01 = ogrid.grid(x0, y1);
        P10 = ogrid.grid(x1, y0);
        P11 = ogrid.grid(x1, y1);

        % Find gradient for each pixel
        a = y(i) - y0 - width/2;
        dx(i) = a * (P11 - P01) + (width-a) * (P10 - P00);
        
        a = x(i) - x0 - width/2;
        dy(i) = a * (P11 - P10) + (width-a) * (P01 - P00);
    end
 
    
end

