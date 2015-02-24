clc

if (~exist('init','var') || init == false)
    if (exist('useSimWorld','var') && useSimWorld == true)
        Lidar_Ranges = Test_Lidar_Ranges;
        Lidar_ScanIndex = Test_Lidar_ScanIndex;
        Lidar_Angles = Test_Lidar_Angles;
        [Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);
    else
        % Grab range measurements from Lidar data (First Echo)
        I = 3:6:6484;
        Lidar_Ranges = Lidar_Log(:, I);
        
        % System Timestamp
        Lidar_Timestamp_System = Lidar_Log(:, 1);
        
        % Sensor Timestamp
        Lidar_Timestamp_Sensor = Lidar_Log(:, 2) / 1000;
        
        % Combined Timestamp
        Lidar_Timestamp = Lidar_Timestamp_System(1) - Lidar_Timestamp_Sensor(1) + Lidar_Timestamp_Sensor;
        
        % Scan Index
        Lidar_ScanIndex = (1:Lidar_ScanCount)';
        
        % Angles for each measurement
        % Rotated the lidar data to face the Y-Axis
        da = (Lidar_AngleEnd - Lidar_AngleStart) / (Lidar_nPoints - 1);
        Lidar_Angles = (Lidar_AngleStart:da:Lidar_AngleEnd)' + pi/2;
        
        
        % Copy each col vector across for each measurement
        Lidar_Timestamp = repmat(Lidar_Timestamp, 1, Lidar_nPoints);
        Lidar_ScanIndex = repmat(Lidar_ScanIndex, 1, Lidar_nPoints);
        Lidar_Angles    = repmat(Lidar_Angles', Lidar_ScanCount, 1);
        
        % Reformat the data so there is one measurement per row
        Lidar_Ranges = reshape(Lidar_Ranges', [], 1);
        Lidar_Timestamp = reshape(Lidar_Timestamp', [], 1);
        Lidar_ScanIndex = reshape(Lidar_ScanIndex', [], 1);
        Lidar_Angles = reshape(Lidar_Angles', [], 1);
        
        
        % Convert from mm to meters
        Lidar_Ranges = Lidar_Ranges / 1000;
        switch algo
            case 2
                fprintf('Data is ready for PSM\n')
            otherwise
                % Remove invalid range data (Too close or too far)
                I = Lidar_Ranges < 0.75 | Lidar_Ranges > 30;
                Lidar_ScanIndex(I) = [];
                Lidar_Timestamp(I) = [];
                Lidar_Angles(I) = [];
                Lidar_Ranges(I) = [];
        end
        % Convert to Cartisian Coordinates
        % (Rotated data so that the Lidar faces the y-Axis)
        [Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);
    end
    init = true;
end

% Scan Settings
nScanIndex = unique(Lidar_ScanIndex);

numberOfScans = 100000;
start = 1;
step = 10; % Scans
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
    
    % Get Current Scan
    switch algo
        case 2
            scan = getLidarPolar(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
        otherwise
            scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    end
    
    % Init World
    if isempty(map)
        % Init map and world
        map = scan;
        tempMap = [];
        switch algo
            case 2
                I = map(:,2) < 30;
                [ tempMap(:,1), tempMap(:,2) ] = pol2cart(map(I,1), map(I,2));
            otherwise
                tempMap = map;
        end
        world = tempMap;
        path = pose;
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
            T(3) = -T(3);
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
    
    % Make Sure Scan in proper Coordinates
    tempScan = [];
    switch algo
        case 2
            I = scan(:,2) < 30;
            [ tempScan(:,1), tempScan(:,2) ] = pol2cart(scan(I,1), scan(I,2));
        otherwise
            tempScan = scan;
    end
    
    % Current World Pose Transform
    dx = pose(1);
    dy = pose(2);
    theta = pose(3);
    
    Trans = [ cos(theta) -sin(theta) dx;
        sin(theta)  cos(theta) dy;
        0           0  1];
    temp = [tempScan ones(size(tempScan,1),1)] * Trans';
    
    
    % Current Scan Transformation
    dx = T(1);
    dy = T(2);
    theta = -T(3);
    LTrans = [ cos(theta) -sin(theta) dx;
        sin(theta)  cos(theta) dy;
        0           0  1];
    tempL = [tempScan ones(size(tempScan,1),1)] * LTrans';
    
    
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
    
    tempMap = [];
    switch algo
        case 2
            I = map(:,2) < 30;
            [ tempMap(:,1), tempMap(:,2) ] = pol2cart(map(I,1), map(I,2));
        otherwise
            tempMap = map;
    end
    
    plot(tempMap(:,1),tempMap(:,2),'r.')
    if useScan2World
        plot(scanWorldFrame(:,1),scanWorldFrame(:,2),'b.')
    else
        plot(tempScan(:,1),tempScan(:,2),'b.')
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
