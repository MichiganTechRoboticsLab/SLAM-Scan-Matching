function [ T, ogrid ] = hcm( guess, scan, map, varargin)
%HCM Hill-climbing scan matcher
%   Detailed explanation goes here


    % Read optional parameters
    p = inputParser;
    p.addParameter('pixelSize', 1, @(x)isnumeric(x));
    p.parse(varargin{:})

    pixelSize = p.Results.pixelSize;

    
    % Initial values
    T(1,:) = guess;
    
    ogrid = oGrid(map, [], pixelSize);
    
    maxIterations = 20;
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
        if 1
           % dont care about rotation right now.            
            dt = sum(dM, 2); 
            dt(1) = dt(1) / sum(dM(1,:) ~= 0);
            dt(2) = dt(2) / sum(dM(2,:) ~= 0);
            dt(3) = 0;
            
            
            % Simulated annealing
            temp = (maxIterations-(iter/2))/(maxIterations);
            dt = dt * temp;
        end
        
        
        
        
        % Paper Implementation
        if 0
            
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
            dt1 = H\temp';

        
       
        end
        
        if 0


    %         >> C++ Implementation: http://goo.gl/PLE2Ie

    %       Eigen::Affine2f transform(getTransformForState(pose));
    % 
    %       float sinRot = sin(pose[2]);
            sinRot = sin(e(3));
    %       float cosRot = cos(pose[2]);
            cosRot = cos(e(3));
    % 
    %       H = Eigen::Matrix3f::Zero();
            H = zeros(3,3);
    %       dTr = Eigen::Vector3f::Zero();
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
    %           float rotDeriv = ((-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] + (cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);
                rotDeriv = (-sinRot * currPoint(1) - cosRot * currPoint(2)) * transformedPointData(2) + (cosRot * currPoint(1) - sinRot * currPoint(2)) * transformedPointData(3);
                rotDeriv = 500;

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
        
        
        % (eq 12) Minimization function
    %         dt2 = inv(H) * sum(h, 1)' * err(itter);
    %         
    %          temp = zeros(1,3);
    %          for idx = 1:N
    %              temp = temp + dM(:,idx)' * dS * (1 - M(idx));
    %          end        
    %          dt = inv(H) * temp';

            dt2 = inv(H) * dTr;

        end
        
        %dt = dt2 * .25;
        
        
        if isnan(dt)
            break;
        end

        % Restrict angular search
        dt(3) = min(dt(3),  0.2);
        dt(3) = max(dt(3), -0.2);
           
        

        % convert to meters
        dt(1) = dt(1) * pixelSize;
        dt(2) = dt(2) * pixelSize;

        
        % Move in the direction of the gradient
        % T = guess + dt';
        T(iter + 1, :) = T(iter, :) + dt';
        T(iter + 1, 3) = T(iter, 3) + dt(3);
        
        
        % DEBUGGING ONLY %

        %fprintf('dt = %.4f %.4f %.4f\n', dt(1), dt(2), rad2deg(dt(3)) )
        
        %plotItteration( 4, ogrid, map, scan, T(iter+1,:), err )
        
        
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
    
 
    %plotItteration( 4, ogrid, map, scan, T, err )
    
end