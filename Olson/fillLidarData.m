
function adjscan = fillLidarData(scan, lidarRangeZ, lidarRangeTheta)
	mag = @(a,b) sqrt((a(1) - b(1))^2 + (a(2) - b(2))^2);
    pts = [];
    ptDist = 0.01;
    thresh = 0.05;

    res = deg2rad(size(scan,1)/lidarRangeTheta);
    theta = (pi-2*res)/2;

    normalizer = 1;
    alpha = sin(res)/sin(theta)*normalizer;
    d_max = alpha * lidarRangeZ;

    minlen = 0.4;
    found = 0;
    edges = [];

    startpt = [];
    endpt = [];

    adjscan = [];

    for idx = 1:size(scan,1) - 1

        a = scan(idx,:);
        b = scan(idx+1,:);  

        %get minimum distance for a point to match up.
        thresh = alpha * mag([0,0], (a + b) / 2);

        adjscan = [adjscan; a; b;];

        if mag(a,b) <= thresh;

            if found == 0
                edges = [edges; a];
                found = 1;
            end

            n = ceil(mag(a,b)/ptDist);
            x = linspace(a(1),b(1),n)';
            y = linspace(a(2),b(2),n)';

            adjscan = [adjscan; x y];
        else
            if found == 1
                edges = [edges; a];
            end
            found = 0;
        end
    end
end