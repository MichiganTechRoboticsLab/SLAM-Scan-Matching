clc

nScanIndex = unique(Lidar_ScanIndex);

numberOfScans = 100000;
start = 1;
step = 1; % Scans
stop = start + step * numberOfScans;

skip = 1; % Points

map = [];
pose = [0 0 0];
path = [0 0 0];
world = [];
T = [0 0 0];
init_guess = [0 0 0];
usePrevOffsetAsGuess = false;
useScan2World = false;


% Create Figures
if ~exist('figs','var')
    figs = [];

    figs(1) = figure;
    figs(2) = figure;

    switch algo
        case 1
            figs(3) = figure;
            figs(4) = figure;
        case 3
            figs(3) = figure;
    end
    
end

% Clear all figures before running
for f = figs
    change_current_figure(f)
    cla
end
drawnow;
pause(0.1);


% Scan Matching Loop
startTime = tic;
for scanIdx = start:step:min(stop,size(nScanIndex,1))
    
    % Display current scan index
    fprintf('Scan %d\n', scanIdx);
        
    % Get current scan data
    scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    
    % Initialize the map with the first scan
    if isempty(map)
        map = scan;
        world = map;
        continue
    end
    
    % Generate a local map from the world map
    if useScan2World
        
        % Translate current scan to map coordinates
        dx    = pose(1);
        dy    = pose(2);
        theta = -pose(3);

        M = [ cos(theta) -sin(theta) dx;
              sin(theta)  cos(theta) dy;
                      0           0  1];

        scanWorldFrame = [scan(1:skip:end,:) ones(size(scan,1), 1)];
        scanWorldFrame = scanWorldFrame(1:skip:end,:) * M';
        scanWorldFrame = scanWorldFrame(:,[1,2]);

        % extract points around the current scan for a reference map
        borderSize = 1; % (Meters)
        map = map(map(:,1) > min(scanWorldFrame(:,1)) - borderSize, :);
        map = map(map(:,1) < max(scanWorldFrame(:,1)) + borderSize, :);
        map = map(map(:,2) > min(scanWorldFrame(:,2)) - borderSize, :);
        map = map(map(:,2) < max(scanWorldFrame(:,2)) + borderSize, :);

        % Limit number of points in the map
        I = randsample(size(map,1), min(size(map,1), 2000));
        map = map(I,:);
    end      
    
    % Scan Matching Algo
    ScanMatch = tic;
    switch algo
        case 0
             T = gicp(init_guess, scan(1:skip:end,:), map(1:skip:end,:), 'minMatchDist', 2, 'costThresh', .00001);
             
        case 1
            % Linear interpolation of conncect points
            m = min(sqrt(scan(:,1).^2 + scan(:,2).^2));
            d = m * sin(deg2rad(270/1081));
            scan = fillLidarData(scan(1:skip:end,:), 270, d);
            
            % Limit number of points in the map
            I = randsample(size(scan,1), min(size(scan,1), 1000));
            scan = scan(I,:);
            
            
            % Low Resolution 
            [ T, lookupTable_l ] = olson(init_guess, scan(1:skip:end,:), map(1:skip:end,:), ... 
                                        'searchRadius', 4,                     ...
                                        'lidarStd', 0.04,                      ...
                                                                               ...
                                        'thetaRange', deg2rad(30) * step / 50,            ...
                                        'dTheta', deg2rad(1) * step / 50, ...
                                                                               ...
                                        'xRange', 1 * step / 50,         ...
                                        'yRange', 1 * step / 50,         ... 
                                        'pixelSize', 0.1); 
                                    

            % High Resolution           
            [ T, lookupTable_h ] = olson(T, scan(1:skip:end,:), map(1:skip:end,:), ... 
                                        'searchRadius', 4,                     ...
                                        'lidarStd', 0.01,                      ...
                                                                               ...
                                        'thetaRange', deg2rad(2) * step / 50,            ...
                                        'dTheta', deg2rad(.1) * step / 50, ...
                                                                               ...
                                        'xRange', 0.2 * step / 50,         ...
                                        'yRange', 0.2 * step / 50,         ... 
                                        'pixelSize', 0.03);
                                    
            
            fprintf('OLSON: Final Guess\n')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')

        case 2
            [ T, iter, err] = psm(init_guess, scan(1:skip:end,:), map(1:skip:end,:));
            fprintf('PSM: Final Guess\n')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')
            fprintf('PSM: %d iterations with %g error\n', iter, err)
            
        case 3  % Hill- Climbing
            
            % Sensor pose for each point in the map for ray tracing.
            poses = repmat([0 0], size(map, 1), 1);
            
            [ T, ogrid ] = hcm(init_guess, scan(1:end,:), map(1:end,:), poses);

    end    
    fprintf('ScanMatcher: Scan %d matched in %.1f seconds. \n', scanIdx, toc(ScanMatch))
        
    % Update current pose
    if useScan2World  
        pose = T;
    else
        % Rotate translation into map frame
        theta = -pose(3);
        Trans = [ cos(theta) -sin(theta), 0;
                  sin(theta)  cos(theta), 0;
                  0           0           1];
        mapT  = T * Trans;
        
        % Add previous scan to pose
        pose = pose + [mapT(1:2), -T(3)];        
    end
    path(end+1,:) = pose;
    
    
    % Current World Pose Transform
    dx = pose(1);
    dy = pose(2);
    theta = pose(3);
    
    Trans = [ cos(theta) -sin(theta) dx;
              sin(theta)  cos(theta) dy;
                       0           0  1];
    temp = [scan ones(size(scan,1),1)] * Trans';
    
    
    % Current Scan Transformation
    dx = T(1);
    dy = T(2);
    theta = -T(3);
    LTrans = [ cos(theta) -sin(theta) dx;
              sin(theta)  cos(theta) dy;
                       0           0  1];
    tempL = [scan ones(size(scan,1),1)] * LTrans';
    
    
    % Add transformed data to world
    if useScan2World
        world = [world; tempL(:,1:2)];
    else
        world = [world; temp(:,1:2)];
    end

    
    
    % Plot World
    change_current_figure(figs(1));
    cla
    hold on
    plot(world(:,1), world(:,2), 'k.', 'MarkerSize', 1)
    plot(path(:,1), path(:,2), 'r.')
    axis equal
    title(['Scan: ' num2str(scanIdx)]);

    
    % Plot Transformed and Map and Scans
    change_current_figure(figs(2));
    cla
    hold on
    plot(map(:,1),map(:,2),'r.') 
    if useScan2World 
        plot(scanWorldFrame(:,1),scanWorldFrame(:,2),'b.')
    else
        plot(scan(:,1),scan(:,2),'b.')
    end
    plot(tempL(:,1),tempL(:,2),'g.')
    hold off
    axis equal
    title(['Scan: ' num2str(scanIdx)]);
    legend('Reference', 'Current Scan', 'Registered Scan')
    
    
    % Algorithm specific plots      
    switch algo
        case 1  % Olson
            
            % Low-resolution lookup table
            change_current_figure(figs(3));
            cla
            imagesc(imrotate(lookupTable_l,90))
            colormap(bone)
            axis equal
            title(['Low-resolution lookup table, Scan: ' num2str(scanIdx)]);

            % High resolution lookup table
            change_current_figure(figs(4));
            cla
            imagesc(imrotate(lookupTable_h,90))
            colormap(bone)
            axis equal
            title(['High-resolution lookup table, Scan: ' num2str(scanIdx)]);
            

        case 3  % Hill Climbing
            
            % Occupancy Grid
            change_current_figure(figs(3));
            cla
            imagesc(imrotate(ogrid.grid,90))
            axis equal
            colormap([1 1 1; 0.5 0.5 0.5; 0 0 0]);
    end 
    
    
    
    % Select the map for the next scan
    if useScan2World  
        map = world;
    else
        map = scan;
    end
    
    % Set next search starting location
    if useScan2World  
        if usePrevOffsetAsGuess
            init_guess = T + (path(end, :) - path(end-1, :));
        else
            init_guess = T;
        end
    else 
        if usePrevOffsetAsGuess
            init_guess = T;
        end
    end

    
    drawnow
    pause(.1)
end

toc(startTime)