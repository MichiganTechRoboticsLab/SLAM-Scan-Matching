function [pts] = getLidarXY(nScan, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex)

%nScanIndex = unique(Lidar_ScanIndex);
LidarRange = 30;
  
% Retrieve each scan's points
nIndex = nScanIndex(nScan);
I = nIndex == Lidar_ScanIndex;

a = Lidar_Angles(I,:)';
z = Lidar_Ranges(I,:)';

%  Remove out of range measurements
I = (z >= LidarRange*0.9);
a(I) = [];
z(I) = [];

[x,y] = pol2cart(a,z);

pts = [x' y'];

end