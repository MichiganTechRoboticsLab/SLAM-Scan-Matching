function [pts] = getLidarXY(nScan, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex, varargin)

    % Optional Parameters
    p = inputParser;
    p.addParameter('LidarRange', 30, @(x)isnumeric(x));

    p.parse(varargin{:})

    LidarRange = p.Results.LidarRange;
    
    % Retrieve each scan's points
    nIndex = nScanIndex(nScan);
    I = nIndex == Lidar_ScanIndex;

    a = Lidar_Angles(I,:)';
    z = Lidar_Ranges(I,:)';
    
    %  Remove out of range measurements
    I = (z >= LidarRange*0.9);
    a(I) = [];
    z(I) = [];
    
    % Convert to cartesian
    [x,y] = pol2cart(a,z);

    % Pack the points into a single matrix
    pts = [x' y'];

end