function [ T, bestScore ] = chamferMatch( T, scan, map, varargin)
%CHAMFERMATCH Chamfer distance based scan-matching
%   Detailed explanation goes here

    % Read optional parameters
    p = inputParser;
    p.addParameter('pixelSize' ,         0.05  , @(x)isnumeric(x)); 
    p.addParameter('dLinear'   ,         0.4   , @(x)isnumeric(x));  
    p.addParameter('dTheta'    , deg2rad(1    ), @(x)isnumeric(x));    
    p.addParameter('iterations',         5     , @(x)isnumeric(x));   
    p.addParameter('verbose'   , false         , @(x)islogical(x));
    p.parse(varargin{:})

    pixelSize   = p.Results.pixelSize;
    dLinear     = p.Results.dLinear;
    dTheta      = p.Results.dTheta;
    iterations  = p.Results.iterations;
    verbose     = p.Results.verbose;
    
    
    % Generate Occupancy Grid
    if verbose
        lookupTableTic = tic;
    end
    
    ogrid = oGrid(map, pixelSize);
    
    if verbose
        fprintf('Chamfer: oGrid generation took %.4f seconds. \n', toc(lookupTableTic))
    end
    
    
    % Generate chamfer distance map    
    if verbose
        lookupTableTic = tic;
    end;
    
    %[Dmap, ~] = bwdist(ogrid.grid);
    %Dmap = -ogrid.grid;
    
    if verbose
        fprintf('Chamfer: DistMap generation took %.4f seconds. \n', toc(lookupTableTic))
        searchTic = tic;
    end
    
    % Search Results
    bestScore = 0;
    Tbest     = T;
       
    
    % Exhausitve search
    if 1
        tmin = 0.03;
        rmin = deg2rad(.5);

        if dLinear > tmin || dTheta > rmin
            t = tmin * ceil(dLinear/tmin);
            r = rmin * ceil(dTheta /rmin);

            for theta = (-r:rmin:r) + T(1,3)

                % Rotate scan
                m = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)] ;
                S = (m * scan')';


                for x = (-t:tmin:t) + T(1,1);
                    for y = (-t:tmin:t) + T(1,2)

                        % Translate scan
                        %s = S + repmat([x y], size(S,1), 1);

                        % Convert to pixel coords
                        Sx0 = (S(:,1) + (x - ogrid.minX)) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1);
                        Sy0 = (S(:,2) + (y - ogrid.minY)) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2);

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
                        %ind   = sub2ind(size(ogrid.grid), Sx2, Sy2);

                        siz = size(ogrid.grid);
                        ind = Sx2 + (Sy2 - 1).*siz(1);

                        score = sum(ogrid.grid(ind));


                        % Keep best score
                        if score > bestScore
                            Tbest     = [x y theta];
                            bestScore = score;                            
                        end
                   end 
                end
            end
        end
        T = Tbest;
    else
    
    % Hi-Lo Search

    %    t = min(tmin, dLinear);
    %    r = min(rmin, dTheta);

        t = dLinear;
        r = dTheta;

        for i = 1:iterations

            for theta = ([-r 0 r]) + T(1,3)

                % Rotate scan
                m = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)] ;
                S = (m * scan')';


                for x = ([-t 0 t]) + T(1,1);
                    for y = ([-t 0 t]) + T(1,2)

                        % Translate scan
                        %s = S + repmat([x y], size(S,1), 1);

                        % Convert to pixel coords
                        Sx0 = (S(:,1) + (x - ogrid.minX)) / (ogrid.maxX - ogrid.minX + ogrid.pixelSize) * size(ogrid.grid, 1);
                        Sy0 = (S(:,2) + (y - ogrid.minY)) / (ogrid.maxY - ogrid.minY + ogrid.pixelSize) * size(ogrid.grid, 2);

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
                        siz = size(ogrid.grid);
                        ind = Sx2 + (Sy2 - 1).*siz(1);                    
                        score = sum(ogrid.grid(ind));

                        % Keep best score
                        if score > bestScore(end)
                            Tbest     = [x y theta];
                            bestScore = score;                            
                        end
                   end 
                end
            end

            % Init Next Iteration
            r = r/2;
            t = t/2;
            T = Tbest;
        end
    end
    
    if verbose
        fprintf('Chamfer: Search took %.4f seconds. \n', toc(searchTic))
    end
end

