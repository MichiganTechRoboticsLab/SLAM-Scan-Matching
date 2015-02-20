function adjscan = fillLidarData(scan, lidarRangeTheta,  ptDist)
    % This function linearly interpolates connected points
    % in effort to eqalize the point density of detected surfaces.

    % scan: points in [x y]
    % lidarRangeTheta: sweep angle (degrees)
    % ptDist: Distance to space new points
    
    
    % Helper Function (distance between a & b)
    mag = @(a,b) sqrt((a(1) - b(1))^2 + (a(2) - b(2))^2);
    
    % Angle between points (WARNING: Assumes no missing points in scan)
    anlgeStep = deg2rad(size(scan,1)/lidarRangeTheta);
    theta = (pi-2*anlgeStep)/2;

    % ??? This is part of the magic that determines if two lines are
    % connected or not....
    normalizer = 1;
    alpha = sin(anlgeStep)/sin(theta)*normalizer;

    % Resulting point set
    adjscan = [];

    
    for idx = 1:size(scan,1) - 1

        % These are the points to compare
        a = scan(idx,:);
        b = scan(idx+1,:);

        % Minimum distance for a point to match up.
        thresh = alpha * mag([0,0], (a + b) / 2);


        % If these points are connected
        if mag(a,b) <= thresh;
            
            % Use linspace to make sure there is at least one point each 
            % ptDist between them.
            n = ceil(mag(a,b)/ptDist);
            x = linspace(a(1), b(1), n)';
            y = linspace(a(2), b(2), n)';

            % Add the new points to the point set
            adjscan = [adjscan; x(1:end-1) y(1:end-1)];
        end
    end
end