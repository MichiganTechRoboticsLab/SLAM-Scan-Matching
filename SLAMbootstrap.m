%function T = betterOlson(map, scan, pixSrch, moveRange, lidarStd, pixRadius, lidarRange)

%clear
clc
close all

nScanIndex = unique(Lidar_ScanIndex);

lidarRange = 30;

map = [];
pose = [0 0 0];
world = [];

figure(1)

T = [0 0 0];

for scanIdx=800:20:10000
    %return the first scan
    scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    if isempty(map)
        map = scan;
        continue
    end
    
    fprintf('calculating transformation...\n');
    skip = 10;
    T = gicp(T, scan(1:skip:end,:), map(1:skip:end,:), 'minMatchDist', 2, 'costThresh', .00001);

    pose = pose + T
    
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    temp = scan * R;
    temp = temp + repmat([x,y],size(temp,1),1);

    world = [world; temp(:,1), temp(:,2)];
    subplot(1,2,1);
    cla
    hold on
    plot(map(:,1),map(:,2),'r.');
    plot(scan(:,1),scan(:,2),'b.');
    plot(scan(:,1) + T(1),scan(:,2) + T(2),'g.');
    hold off
    subplot(1,2,2);
    plot(world(:,1),world(:,2),'k.');
    
    map = scan;
    
    pause(0.1)
end