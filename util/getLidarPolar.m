function [pts] = getLidarPolar(nScan, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex, varargin)

%nScanIndex = unique(Lidar_ScanIndex);


p = inputParser;
p.addParameter('LidarRange', 30, @(x)isnumeric(x));

p.parse(varargin{:})

LidarRange = p.Results.LidarRange;

% Retrieve each scan's points
nIndex = nScanIndex(nScan);
I = nIndex == Lidar_ScanIndex;

pts(:,1) =  Lidar_Angles(I,:);
pts(:,2) = Lidar_Ranges(I,:);

end