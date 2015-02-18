%  clc

nScanIndex = unique(Lidar_ScanIndex);

numberOfScans = 4000;
start = 1900;
step = 50; % Scans
stop = start + step * numberOfScans;

skip = 1; % Points

map = [];
pose = [0 0 0];
world = [];
T = [0 0 0];
init_guess = [0 0 0];
usePrevOffsetAsGuess = true;
useIMUAngle = false;

if useIMUAngle

end

if ~exist('figs','var')
    figs = [];

    figs(1) = figure;
    figs(2) = figure;

    switch algo
        case 1
            figs(3) = figure;
    end
    
    for f = figs
        change_current_figure(f)
        cla
    end
end

drawnow;

for scanIdx = start:step:min(stop,size(nScanIndex,1))
    % Get Current Scan
    scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    if isempty(map)
        % Init map and world
        map = scan;
        world = map;
        continue
    end
    
    %fprintf('calculating transformation...\n');
    
    if useIMUAngle
        
    end
    
    
    % Scan Matching Algo
    switch algo
        case 0
             T = gicp(init_guess, scan(1:skip:end,:), map(1:skip:end,:), 'minMatchDist', 2, 'costThresh', .00001);
        case 1
            [ T, lookupTable ] = olson(init_guess, scan(1:skip:end,:), map(1:skip:end,:), ...
                                       'searchRadius', 2, ...         % 2
                                       'dTheta', deg2rad(.05), ...    % 0.05
                                       'thetaRange', deg2rad(20), ... % 20
                                       'xRange', 1, ...               % 1
                                       'yRange', 1, ...               % 1
                                       'pixelSize', .03);             % 0.03
                                   
            fprintf('OLSON: Final Guess\n')
            T(3) = -T(3);
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')
            change_current_figure(figs(3));
            cla
            imagesc(imrotate(lookupTable, 90))
            axis equal
    end
    
    % Create Transformation
    pose = pose + T;
    
    dx = pose(1);
    dy = pose(2);
    theta = pose(3);
    
    Trans = [ cos(theta) -sin(theta) dx;
              sin(theta)  cos(theta) dy;
                       0           0  1];
    
    % Local Transformation
    dx = T(1);
    dy = T(2);
    theta = T(3);
    
    LTrans = [ cos(theta) -sin(theta) dx;
              sin(theta)  cos(theta) dy;
                       0           0  1];
    
    % Do the Transformation
    temp = [scan ones(size(scan,1),1)] * Trans';
    tempL = [scan ones(size(scan,1),1)] * LTrans';
    
    % Add transformed data to world
    world = [world; temp(:,1:2)];

    
    
    % Plot World
    change_current_figure(figs(1));
    cla
    plot(world(:,1), world(:,2), 'k.')
    axis equal

    
    % Plot Transformed and Map and Scans
    change_current_figure(figs(2));
    cla
    hold on
    plot(map(:,1),map(:,2),'r.')
    plot(scan(:,1),scan(:,2),'b.')
    plot(tempL(:,1),tempL(:,2),'g.')
    hold off
    axis equal
    
    if usePrevOffsetAsGuess
        init_guess(3) = T(3);
    end
    
    map = scan;
    
    
    drawnow
    pause(.1)
end