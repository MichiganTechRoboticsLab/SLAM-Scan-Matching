function [ T ] = chamferMatch( T, scan, map, varargin)
%CHAMFERMATCH Chamfer distance based scan-matching
%   Detailed explanation goes here

    % Read optional parameters
    p = inputParser;
    p.addParameter('pixelSize' ,         0.05  , @(x)isnumeric(x));
    p.addParameter('xRange'    ,         0.4   , @(x)isnumeric(x));
    p.addParameter('yRange'    ,         0.4   , @(x)isnumeric(x));
    p.addParameter('thetaRange', deg2rad(12    ), @(x)isnumeric(x));
    p.addParameter('dTheta'    , deg2rad(1 ), @(x)isnumeric(x));
    p.parse(varargin{:})

    pixelSize   = p.Results.pixelSize;
    xRange      = p.Results.xRange;
    yRange      = p.Results.yRange;
    thetaRange  = p.Results.thetaRange;
    dTheta      = p.Results.dTheta;
    
    verbose = false;
    
    % Generate Occupancy Grid
    lookupTableTic = tic;
    ogrid = oGrid(map, [], pixelSize);
    
    % Generate chamfer distance map
    [Dmap, ~] = bwdist(ogrid.grid);
    
    if verbose
        fprintf('Chamfer: DistMap generation took %.1f seconds. \n', toc(lookupTableTic))
    end

    
    % Exhaustive Search
    searchTic = tic;
    bestScore = 10e+20;
    for theta = [(-thetaRange:dTheta:thetaRange)] + T(1,3)
        
        % Rotate
        m = [cos(theta) -sin(theta);
             sin(theta)  cos(theta)] ;
        S = (m * scan')';
        
        
        
        for x = ([-xRange:pixelSize:xRange] + T(1,1));
            for y = ([-yRange:pixelSize:yRange] + T(1,2))
                
                s = S + repmat([x y], size(S,1), 1);
                
                % Convert to pixel coords
                Sx0 = (s(:,1) - ogrid.minX) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1);
                Sy0 = (s(:,2) - ogrid.minY) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2);
        
                % Translate
                Sx1 = round(Sx0);  
                Sy1 = round(Sy0);
                                
                % Bounds Checking
                I = (Sx1 < 1) | ...
                    (Sy1 < 1) | ...
                    (Sx1 > size(ogrid.grid, 1)) | ...
                    (Sy1 > size(ogrid.grid, 2)) ;
                Sx2 = Sx1(~I);
                Sy2 = Sy1(~I);
                
                % Fitness
                ind   = sub2ind(size(ogrid.grid), Sx2, Sy2);
                score = sum(Dmap(ind));
                
%                 % Debug Plot
%                 change_current_figure(4)
%                 clf;
%                 imagesc(ogrid.grid);
%                 axis equal;
%                 hold on;
%                 contour(Dmap);
%                 plot(Sy0, Sx0, 'Or');
%                 plot(Sy1, Sx1, '+g');
%                 plot(Sy2, Sx2, '.b');
%                 title(sprintf('Score = %6d, T = %.4f %.4f %.4f\n', ...
%                               score, T(end,1), T(end,2), rad2deg(T(end,3) )));
%                 drawnow;
%                 pause(0.1)
                
                % Keep best score
                if score < bestScore(end)
                    T(end+1,:)       = [x y theta];
                    bestScore(end+1) = score;
                    
%                     fprintf('Score = %6d, T = %.4f %.4f %.4f\n', ...
%                              score, T(end,1), T(end,2), rad2deg(T(end,3) ));
        
                end
           end 
        end
    end
    
    
    if verbose
        fprintf('Chamfer: Search took %.1f seconds. \n', toc(searchTic))
    end
    
%     % Debug plot of D & I maps
%     plotItteration( 5, ogrid, map, scan, T(end,:), bestScore )   
%     change_current_figure(5)
%     contour(Dmap);
        
    % Only return the final result
    T = T(end,:);
end

