function [ T, ogrid ] = hcm( guess, scan, map, poses, varargin)
%HCM Hill-climbing scan matcher
%   Detailed explanation goes here


    % Read optional parameters
    p = inputParser;
    p.addParameter('pixelSize', .3, @(x)isnumeric(x));
    p.parse(varargin{:})

    pixelSize = p.Results.pixelSize;

    
    % Initial values
    T = guess;
    
    ogrid = oGrid(map, poses, pixelSize);
    
    for itter = 1:30
        % Gauss-Newton gradient Decent

        % (eq 7) Current transform between map and scan
        e = T';

        % (eq 8) Scan points transformed to current estimate
        theta = e(3);
        m = [cos(theta) -sin(theta);
             sin(theta)  cos(theta)] ;
        S = m * scan' + repmat( e(1:2), 1, size(scan,1)) ;

        % Pixel values at each estimated hit location
        M = ogrid_subpixel(ogrid, S');

        % Fudge hits off the map
        M(isnan(M)) = 0;


        % (eq 9) Error function for current pose
        err(itter) = sum(1 - M);

        % (eq 13) H matrix
        [dx, dy] = ogrid_gradient( ogrid, S' );
        dM = [dx; dy];

        N = size(scan,1);


        for i = 1:N
            x = scan(i,1);
            y = scan(i,2);
            w = T(3);

            dS = [1 0 -sin(w) * x - cos(w)*y;
                  0 1  cos(w) * x - sin(w)*y]; 

            h(i, :) = dM(:,i)' * dS;
        end

        H = h'*h;

        
        % (eq 12) Minimization function
        temp = zeros(1,3);
        for idx = 1:N
            temp = temp + dM(:,idx)'*dS * (1 - M(idx));
        end
        dt = inv(H) * temp';


        % convert to meters
        dt(1) = dt(1) * pixelSize;
        dt(2) = dt(2) * pixelSize;

        
        % Move in the direction of the gradient
        T = T + dt'; 
    
        % Debug plot
        plotItteration( 3, ogrid, map, scan, T )
    end
    
    
    return
  
    % Subpixel and gradient function debug plots
    divby = 10;
    
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
    
    
    % Plot the result
    
    map_x = (map(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize/divby) * size(v, 1);
    map_y = (map(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize/divby) * size(v, 2);
   
    
    figure(4)
    clf
    imagesc(imrotate(v,0))
    axis equal
    colormap(bone);
    title('Bilinear filter-d Occupancy Grid');
    hold on;
    plot(map_y, map_x, '.r');
    
    
    figure(5)
    clf
    imagesc(imrotate(dx,0))
    axis equal
    colormap(bone);
    title('dx');
    hold on;
    plot(map_y, map_x, '.r');
    
    figure(6)
    clf
    imagesc(imrotate(dy,0))
    axis equal
    colormap(bone);
    title('dy');
    hold on;
    plot(map_y, map_x, '.r');


    
end

