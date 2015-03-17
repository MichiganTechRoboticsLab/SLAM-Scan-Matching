function [ T, ogrid ] = hcm( guess, scan, map, varargin)
%HCM Hill-climbing scan matcher
%   Detailed explanation goes here


    % Read optional parameters
    p = inputParser;
    p.addParameter('pixelSize'    ,  0.20, @(x)isnumeric(x));
    p.addParameter('maxIterations', 20   , @(x)isnumeric(x));
    p.parse(varargin{:})

    pixelSize     = p.Results.pixelSize;
    maxIterations = p.Results.maxIterations;
    
    % Initial values
    T(1,:) = guess;
    
    %guess = [1 0 0];
    
    ogrid = oGrid(map, [], pixelSize);
    
    % Current Scan @ Initial Guess ( for debug plotting )
    theta = guess(3);
    m = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)] ;
    initialScan = (scan * m' + repmat( guess(1:2), size(scan,1), 1)) ;

   
    for iter = 1:maxIterations
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

        
        % Naive Implemenation (AKA Dereck made it up....)
        if 0
           % dont care about rotation right now.            
            dt = sum(dM .* repmat((1-M)', 2, 1), 2);          
            %dt = sum(dM, 2) ./ sum((dM ~= 0), 2);          
            %dt = sum(dM, 2); 
            dt(1) = dt(1) / sum(dM(1,:) ~= 0) ;
            dt(2) = dt(2) / sum(dM(2,:) ~= 0) ;
            
            
            % Find dTheta ( Currently borken )
            if 0
                % Convert scan points to polar
                [a1, d1]  = cart2pol(S(1,:), S(2,:));

                % Convert gradients to polar
                [a2, d2]  = cart2pol(dM(1,:), dM(2,:));

                % Remove no rotation elements.
                I = d2 < 0.1;
                a1(I) = [];
                d1(I) = [];
                a2(I) = [];
                d2(I) = [];

                % rotate all gradients to one axis.
                d3 = d2;
                a3 = a2 - a1;       

                % Find Rotation forces
                %rf = sin(a3) .* d3 ./ d2 * pixelSize;

                [x, ~] = pol2cart(a3, d3);

                % Find Mean of rotations
                mrf = mean(x / d1 * pixelSize);

                % Set rotation
                dt(3) = asin(mrf) * -0.25;
            else
                dt(3) = 0;
            end
            
            % Fix div zeros
            dt(isnan(dt)) = 0;
            
        end
        
        
        
        
        % Paper Implementation
        if 0
            H = zeros(3,3);
            w = T(3);
            for i = 1:N
                x = scan(i,1);
                y = scan(i,2);

                dS = [1 0 -sin(w) * x - cos(w) * y;
                      0 1  cos(w) * x - sin(w) * y]; 

                h = dM(:,i)' * dS;
                
                H = H + (h' * h);
            end


            %check if the matrix is singular
            if rcond(H) < 1e-12
                warning('singular matrix detected')
                break;
            end

            % (eq 12) Minimization function
            tmp = zeros(3,1);
            for idx = 1:N
                x = scan(i,1);
                y = scan(i,2);

                dS = [1 0 -sin(w) * x - cos(w) * y;
                      0 1  cos(w) * x - sin(w) * y]; 

                h = dM(:,i)' * dS;
                
                %tmp = tmp + dM(:,idx)'*dS * (1 - M(idx));
                tmp = tmp + h' * (1 - M(idx));
            end
            dt = H \ tmp;
            
        end
        
        


        if 1
    %       >> C++ Implementation: http://goo.gl/PLE2Ie

    %       Eigen::Affine2f transform(getTransformForState(pose));
    % 
    %       float sinRot = sin(pose[2]);
            %sinRot = sin(e(3));
            sinRot = sin(guess(3));
            
    %       float cosRot = cos(pose[2]);
            %cosRot = cos(e(3));
            cosRot = cos(guess(3));
    % 
    %       H = Eigen::Matrix3f::Zero();
            H = zeros(3,3);
    %       dTr = Eigen::Vector3f::Zero();
    %       (Column vector per: http://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html)
            dTr = zeros(3,1);

    %       for (int i = 0; i < size; ++i) {   
            for i = 1:N
    % 
    %           const Eigen::Vector2f& currPoint (dataPoints.getVecEntry(i));
                currPoint = scan(i,:);
    % 
    %           Eigen::Vector3f transformedPointData(interpMapValueWithDerivatives(transform * currPoint));
                transformedPointData(1)   = M(i);
                transformedPointData(2:3) = dM(:, i);
    % 
    %           float funVal = 1.0f - transformedPointData[0];
                funVal = 1 - transformedPointData(1);
    % 
    %           dTr[0] += transformedPointData[1] * funVal;
    %           dTr[1] += transformedPointData[2] * funVal;
                dTr(1) = dTr(1) + transformedPointData(2) * funVal;
                dTr(2) = dTr(2) + transformedPointData(3) * funVal;
    % 
    %           float rotDeriv = (-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] 
    %                          + ( cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2] ;
                rotDeriv = (-sinRot * currPoint(1) - cosRot * currPoint(2)) * transformedPointData(2) ...
                         + ( cosRot * currPoint(1) - sinRot * currPoint(2)) * transformedPointData(3) ;
                
    % 
    %           dTr[2] += rotDeriv * funVal;
                dTr(3) = dTr(3) + rotDeriv * funVal;
    % 
    %           H(0, 0) += util::sqr(transformedPointData[1]);
    %           H(1, 1) += util::sqr(transformedPointData[2]);
    %           H(2, 2) += util::sqr(rotDeriv);
                H(1, 1) = H(1, 1) + transformedPointData(2)^2;
                H(2, 2) = H(2, 2) + transformedPointData(3)^2;
                H(3, 3) = H(3, 3) + rotDeriv^2;
    % 
    %           H(0, 1) += transformedPointData[1] * transformedPointData[2];
    %           H(0, 2) += transformedPointData[1] * rotDeriv;
    %           H(1, 2) += transformedPointData[2] * rotDeriv;
                H(1, 2) = H(1, 2) + transformedPointData(2) * transformedPointData(3);
                H(1, 3) = H(1, 3) + transformedPointData(2) * rotDeriv;
                H(2, 3) = H(2, 3) + transformedPointData(3) * rotDeriv;

    %         }
       
        % 
        %       H(1, 0) = H(0, 1);
        %       H(2, 0) = H(0, 2);
        %       H(2, 1) = H(1, 2);
                H(2, 1) = H(1, 2);
                H(3, 1) = H(1, 3);
                H(3, 2) = H(2, 3); 

            end

            dt = H \ dTr;
            
            %dt(3) = dt(3) * 0.3;
            %dt = dt * 0.2;
        end 
        
        
        
        if sum(isnan(dt)) > 0
            break;
        end
        
         dt(3) = dt(3) * 0.5;
         dt(3) = min(dt(3),  deg2rad(1));
         dt(3) = max(dt(3), -deg2rad(1));
         
        % Simulated annealing
        if 1
            temp = (maxIterations-iter)/(maxIterations+1);
            dt = dt * temp;
        end
        
        % Restrict angular search
%         dt(1) = min(dt(1),  0.1);
%         dt(1) = max(dt(1), -0.1);
%         dt(2) = min(dt(2),  0.1);
%         dt(2) = max(dt(2), -0.1);

        % convert to meters
        dt(1) = dt(1) * pixelSize;
        dt(2) = dt(2) * pixelSize;
        
        % Move in the direction of the gradient
        % T = guess + dt';
        T(iter + 1, :) = T(iter, :) + dt';
        
        
        % DEBUGGING ONLY %

        %fprintf('dt = %.4f %.4f %.4f\n', dt(1), dt(2), rad2deg(dt(3)) )
        
        %plotItteration( 4, ogrid, map, initialScan, T(iter+1,:), err )
        
        
        % Convergence Criteria
        %if sum(abs(dt)) > 0.0001
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
    
    
 
    %plotItteration( 4, ogrid, map, scan, T, err )
    
end