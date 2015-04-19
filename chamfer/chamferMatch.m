function [ T, bestHits] = chamferMatch( T, scan, map, ogrid, Dmap, varargin)
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
    
    
    
    % Search Results
    if UseChamfer
        bestScore = 10e20;
    else
        bestScore = 0;
    end
    Tbest     = T;
       
    
    % Exhausitve search
    if 0
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
                    siz = size(ogrid.grid);
                    ind = Sx2 + (Sy2 - 1).*siz(1);

                    if UseChamfer
                        D = Dmap(ind);
                        hits  = (D == 0);
                        score = sum(D);
                    else
                        hits  = ogrid.grid(ind);
                        score = sum(hits);
                    end

                    % Keep best score
                    if UseChamfer
                        if score < bestScore
                            Tbest     = [x y theta];
                            bestScore = score;  
                            bestHits  = hits;                          
                        end
                    else
                        if score > bestScore
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

                        % Fitness function
                        siz = size(ogrid.grid);
                        ind = Sx2 + (Sy2 - 1).*siz(1);   
                            
                        if UseChamfer
                            D = Dmap(ind);
                            hits  = (D == 0);
                            score = sum(D);
                        else 
                            hits  = ogrid.grid(ind);
                            score = sum(hits);
                        end

                        % Keep best score
                        if UseChamfer
                            if score < bestScore
                                Tbest     = [x y theta];
                                bestScore = score;  
                                bestHits  = hits;                          
                            end
                        else
                            if score > bestScore
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
        t = SearchLin;
        r = SearchRot;
        
        if UseChamfer
          grid = Dmap;
        else
          grid = ogrid.grid; 
        end
        
        siz = size(ogrid.grid);
        tmpX = ((ogrid.maxX - ogrid.minX + ogrid.pixelSize) / siz(1));
        tmpY = ((ogrid.maxY - ogrid.minY + ogrid.pixelSize) / siz(2));




        dmax = 0;
        imax = 0;
        while imax < 30 %&& (t > 0.02 | r > deg2rad(0.2))

            for theta = ([-r 0 r]) + T(1,3)

                % Rotate scan
                m = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)] ;
                S = (m * scan')';


                for x = ([-t 0 t]) + T(1,1)
                    for y = ([-t 0 t]) + T(1,2)

                    % Translate points and convert to pixel coords                    
                    Sx1 = round((S(:,1) + (x - ogrid.minX)) / tmpX );  
                    Sy1 = round((S(:,2) + (y - ogrid.minY)) / tmpY );
                    

                        % Bounds Checking
                        I = (Sx1 > 1) & ...
                            (Sy1 > 1) & ...
                            (Sx1 < siz(1)) & ...
                            (Sy1 < siz(2)) ;
                        Sx2 = Sx1(I);
                        Sy2 = Sy1(I);

                        % Fitness
                        ind = Sx2 + (Sy2 - 1).*siz(1);                    
                                               
                        hits  = grid(ind);
                        score = sum(hits);
                        

                        % Keep best score
                        if score < bestScore
                            Tbest     = [x y theta];
                            bestScore = score;  
                            bestHits  = hits;                          
                        end       
                    end 
                end
            end

            if T == Tbest
                r = r/2;
                t = t/2;
                
                dmax = dmax + 1;
                if dmax > 3
                    break
                end
            end

            % Init Next Iteration
            T = Tbest;
            imax = imax + 1;
        end
    end
    
    bestHits = (bestHits == 0);
end

