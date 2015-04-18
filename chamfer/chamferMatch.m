function [ T, bestHits ] = chamferMatch( T, scan, map, varargin)
%CHAMFERMATCH Chamfer distance based scan-matching
%   Detailed explanation goes here

    % Read optional parameters
    p = inputParser;
    p.addParameter('pixelSize' ,         0.05  , @(x)isnumeric(x)); 
    p.addParameter('dLinear'   ,         0.4   , @(x)isnumeric(x));  
    p.addParameter('dTheta'    , deg2rad(1    ), @(x)isnumeric(x));    
    p.addParameter('SearchRot' , deg2rad(.25  ), @(x)isnumeric(x));    
    p.addParameter('SearchLin' ,         0.5   , @(x)isnumeric(x));   
    p.addParameter('verbose'   ,         false , @(x)islogical(x));
    p.parse(varargin{:})

    pixelSize   = p.Results.pixelSize;
    dLinear     = p.Results.dLinear;
    dTheta      = p.Results.dTheta;
    SearchRot   = p.Results.SearchRot;
    SearchLin   = p.Results.SearchLin;
    verbose     = p.Results.verbose;
    
    UseChamfer = true;
    
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
    
    if UseChamfer
        [Dmap, ~] = bwdist(ogrid.grid);
    end
    
    if verbose
        fprintf('Chamfer: DistMap generation took %.4f seconds. \n', toc(lookupTableTic))
        searchTic = tic;
    end
    
    % Search Results
    if UseChamfer
        bestScore = 10e20;
    else
        bestScore = 0;
    end
    Tbest     = T;
       
    
    % Exhausitve search
    if 1
        t = SearchLin * ceil(dLinear/SearchLin);
        r = SearchRot * ceil(dTheta /SearchRot);

        for theta = (-r:SearchRot:r) + T(1,3)

            % Rotate scan
            m = [cos(theta) -sin(theta);
                 sin(theta)  cos(theta)] ;
            S = (m * scan')';


            tmpX = ((ogrid.maxX - ogrid.minX + ogrid.pixelSize) / size(ogrid.grid, 1));
            tmpY = ((ogrid.maxY - ogrid.minY + ogrid.pixelSize) / size(ogrid.grid, 2));

            for x = (-t:SearchLin:t) + T(1,1)
                for y = (-t:SearchLin:t) + T(1,2)

                    % Translate points and convert to pixel coords
                    Sx0 = (S(:,1) + (x - ogrid.minX)) / tmpX ;
                    Sy0 = (S(:,2) + (y - ogrid.minY)) / tmpY ;

                    Sx1 = round(Sx0);  
                    Sy1 = round(Sy0);

                    % Bounds Checking
                    I = (Sx1 > 1) & ...
                        (Sy1 > 1) & ...
                        (Sx1 < size(ogrid.grid, 1)) & ...
                        (Sy1 < size(ogrid.grid, 2)) ;
                    Sx2 = Sx1(I);
                    Sy2 = Sy1(I);

                    % Fitness
                    %ind   = sub2ind(size(ogrid.grid), Sx2, Sy2);

                    siz = size(ogrid.grid);
                    ind = Sx2 + (Sy2 - 1).*siz(1);


                    if UseChamfer
                        hits  = (Dmap(ind) <= 0); % & (Dmap(ind) > 10);
                        score = sum(Dmap(ind));
                    else
                        hits  = ogrid.grid(ind);
                        score = sum(hits);
                    end

                    % Keep best score
                    if UseChamfer
                        if score < bestScore(end)
                            Tbest     = [x y theta];
                            bestScore = score;  
                            bestHits  = hits;                          
                        end
                    else
                        if score > bestScore(end)
                            Tbest     = [x y theta];
                            bestScore = score;  
                            bestHits  = hits;                          
                        end
                    end
                end 
            end
        end
        T = Tbest;
    end
    
    % Hi-Lo Search
    if 0
    %    t = SearchLin;
    %    r = SearchRot;

        t = dLinear;
        r = dTheta;

        while r >= SearchRot || t >= SearchLin

            for theta = ([-r 0 r]) + T(1,3)

                % Rotate scan
                m = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)] ;
                S = (m * scan')';


                for x = ([-t 0 t]) + T(1,1)
                    for y = ([-t 0 t]) + T(1,2)

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
                                               
                            
                        if UseChamfer
                            hits  = Dmap(ind) <= 1;
                            score = sum(Dmap(ind));
                        else 
                            hits  = ogrid.grid(ind);
                            score = sum(hits);
                        end

                        % Keep best score
                        if UseChamfer
                            if score < bestScore(end)
                                Tbest     = [x y theta];
                                bestScore = score;  
                                bestHits  = hits;                          
                            end
                        else
                            if score > bestScore(end)
                                Tbest     = [x y theta];
                                bestScore = score;  
                                bestHits  = hits;                          
                            end
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
    
    
    
    
    % Decent walk
    if 1
    %    t = SearchLin;
    %    r = SearchRot;

        t = SearchLin/2;
        r = SearchRot/2;

        imax = 0;
        while imax < 5

            for theta = ([-r 0 r]) + T(1,3)

                % Rotate scan
                m = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)] ;
                S = (m * scan')';


                for x = ([-t 0 t]) + T(1,1)
                    for y = ([-t 0 t]) + T(1,2)

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
                                               
                        if UseChamfer
                            hits  = Dmap(ind) <= 1;
                            score = sum(Dmap(ind));
                        else 
                            hits  = ogrid.grid(ind);
                            score = sum(hits);
                        end

                        % Keep best score
                        if UseChamfer
                            if score < bestScore(end)
                                Tbest     = [x y theta];
                                bestScore = score;  
                                bestHits  = hits;                          
                            end
                        else
                            if score > bestScore(end)
                                Tbest     = [x y theta];
                                bestScore = score;  
                                bestHits  = hits;                          
                            end
                        end
                   end 
                end
            end

            if T == Tbest
              r = r/2;
              t = t/2;
              %break;
            end

            % Init Next Iteration
            T = Tbest;
            imax = imax + 1;
        end
    end
    
    
    
    if verbose
        fprintf('Chamfer: Search took %.4f seconds. \n', toc(searchTic))
    end
end

