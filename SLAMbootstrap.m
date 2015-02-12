%  clc

nScanIndex = unique(Lidar_ScanIndex);

numberOfScans = 4;
start = 800;
step = 10;

stop = start + step * numberOfScans;
skip = 1;

map = [];
pose = [0 0 0];
world = [];

figure(1)

T = [0 0 0];
init_guess = [0 0 0];
usePrevOffsetAsGuess = false;

for scanIdx = start:step:stop
    % Get Current Scan
    scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    if isempty(map)
        % Init map and world
        map = scan;
        world = map;
        continue
    end
    
    %fprintf('calculating transformation...\n');
    
    % Scan Matching Algo
    switch algo
        case 0
             T = gicp(init_guess, scan(1:skip:end,:), map(1:skip:end,:), 'minMatchDist', 2, 'costThresh', .00001);
        case 1
            [ T, lookupTable ] = olson(init_guess, scan(1:skip:end,:), map(1:skip:end,:), 'searchRadius', .3);
            T
            figure(3)
            cla
            axis equal
            imagesc(lookupTable)
    end
    
    % Create Transformation
    pose = pose + T;
    
    dx = pose(1);
    dy = pose(2);
    theta = pose(3);
    
    T = [ cos(theta) -sin(theta) dx;
          sin(theta)  cos(theta) dy;
                   0           0  1];
    
    % Do the Transformation
    temp = [scan ones(size(scan,1),1)] * T';
    
    % Add transformed data to world
    world = [world; temp(:,1:2)];

    
    
    % Plot World
    figure(1)
    cla
    plot(world(:,1), world(:,2), 'k.')
    axis equal

    
    % Plot Transformed and Map and Scans
    figure(2)
    cla
    hold on
    plot(map(:,1),map(:,2),'r.')
    plot(scan(:,1),scan(:,2),'b.')
    plot(temp(:,1),temp(:,2),'g.')
    hold off
    axis equal
    
    if usePrevOffsetAsGuess
        init_guess = T;
    end
    
    map = scan;
    
    pause(.1)
end