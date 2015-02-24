function [ T, ogrid ] = hcm( guess, scan, map, varargin)
%HCM Hill-climbing scan matcher
%   Detailed explanation goes here


    % Read optional parameters
    p = inputParser;
    p.addParameter('pixelSize', .3, @(x)isnumeric(x));
    p.parse(varargin{:})

    pixelSize = p.Results.pixelSize;

    
    % Initial values
    T(1,:) = guess;
    
    ogrid = oGrid(map, [], pixelSize);
    
    for iter = 1:50
        % Gauss-Newton gradient Decent

        % (eq 7) Current transform between map and scan
        e = T(iter,:)';

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
        err(iter) = sum(1 - M);


        % (eq 13) H matrix
        [dx, dy] = ogrid_gradient( ogrid, S' );
        dM = [dx; dy];

        N = size(scan,1);

        for i = 1:N
            x = scan(i,1);
            y = scan(i,2);
            w = T(3);

            dS = [1 0 -sin(w) * x + cos(w)*y;
                  0 1  cos(w) * x - sin(w)*y]; 

            h(i, :) = dM(:,i)' * dS;
        end

        H = h'*h;
        
        %check if the matrix is singular
        if rcond(H) < 1e-12
            warning('singular matrix detected')
            break;
        end
            
        % (eq 12) Minimization function
        temp = zeros(1,3);
        for idx = 1:N
            temp = temp + dM(:,idx)'*dS * (1 - M(idx));
        end
        dt = H\temp';


        
        % convert to meters
        dt(1) = dt(1) * pixelSize;
        dt(2) = dt(2) * pixelSize;

        
        % Move in the direction of the gradient
        % T = guess + dt';
        T(iter + 1, :) = T(iter, :) + dt';
        T(iter + 1, 3) = T(iter, 3) - dt(3);
        
        
        % DEBUGGING ONLY %
        plotItteration( 4, ogrid, map, scan, T(iter+1,:), err )
        
        
        % Convergence Criteria
        %if sum(dt) < 0.0001 
        %    break
        %end
    end
    

    % Calculate score for final iteration
    e = T(end,:)';
    theta = e(3);
    m = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)] ;
    S = m * scan' + repmat( e(1:2), 1, size(scan,1)) ;
    M = ogrid_subpixel(ogrid, S');
    M(isnan(M)) = 0;
    err(end) = sum(1 - M);
    
    
    % Select best transform for solution
    [~,I] = min(err);
    T = T(I,:);
    
end

