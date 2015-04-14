function [pts, pol] = getLidarXY(nScan, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex, varargin)

    p = inputParser;
    p.addParameter('LidarRange', 30, @(x)isnumeric(x));
    p.parse(varargin{:})

    LidarRange = p.Results.LidarRange;


    % Retrieve each scan's points
    i = (nScan-1) * 1081 + 1;
    I = i:(i+1080);
    a = Lidar_Angles(I)';
    z = Lidar_Ranges(I)';

    % Remove out of range measurements
    I = (z < 0.5 | z > LidarRange*0.9);
    a(I) = [];
    z(I) = [];

    % Convert to cartisian
    [x,y] = pol2cart(a,z);
    pts = [x' y'];
    
    % Also make the original polar version available
    pol = [a z];

end
