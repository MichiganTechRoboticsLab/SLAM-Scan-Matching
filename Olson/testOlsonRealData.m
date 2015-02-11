%function T = betterOlson(map, scan, pixSrch, moveRange, lidarStd, pixRadius, lidarRange)

%clear
clc
%close all

nScanIndex = unique(Lidar_ScanIndex);
%number of meters to radiate out
pixSrch = .3;

%movement range (how much do you think the scans will move between each
%other maximum) in meters
moveRange = 3;

%standard deviation of points
lidarStd = 0.01; %meters

pixRadius = .03;

lidarRange = 30;

map = [];
pose = [0 0 0];
world = [];
T = [0 0 0];




for scanIdx=800:10:10000
    %return the first scan
    scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    if isempty(map)
        map = scan;
        continue
    end
    
    fprintf('calculating olson...\n');
    skip = 1;
    [T, lookupTable] = olson(map, scan(1:skip:end,:), pixSrch, moveRange, lidarStd, pixRadius, lidarRange, ...
        0.5, 0.01, 0.5, 0.01, deg2rad(0.9), deg2rad(0.25), T);
    
    %T(abs(T(1:2)) < 0.05) = 0;
    T   
    pose = pose + T
    
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    temp = scan * R;
    temp = temp + repmat([x,y],size(temp,1),1);
    world = [world; temp(:,1), temp(:,2)];
    %subplot(1,2,1);
    figure(1)
    clf
    hold on
    plot(map(:,1),map(:,2),'r.');
    plot(scan(:,1),scan(:,2),'b.');
    plot(scan(:,1) + T(1),scan(:,2) + T(2),'g.');
    hold off
    axis square
    
    figure(2)
    clf
    plot(world(:,1),world(:,2),'k.');
    axis square
    
    figure(3)
    clf
    imagesc(lookupTable);
    colormap(gray)
    axis equal
    %imshow(mat2grey(lookupTable))
    map = scan;
    drawnow()
    %pause(0.1)
end