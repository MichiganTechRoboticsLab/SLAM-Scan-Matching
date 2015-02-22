clc

%% Proper Coor.
if (~exist('init','var') || init == false)
    switch algo
        
        % Get Full Scans for PSM
        case 2
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
            % Convert to Cartisian Coordinates
            % (Rotated data so that the Lidar faces the y-Axis)
            [Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);
    end
    init = true;
end

%% Scan Settings
nScanIndex = unique(Lidar_ScanIndex);

numberOfScans = 100000;
start = 1;
step = 1; % Scans
stop = start + step * numberOfScans;

skip = 1; % Points

map = [];
pose = [0 0 0];
path = [];
world = [];
T = [0 0 0];
init_guess = [0 0 0];
usePrevOffsetAsGuess = false;
useScan2World = false;

%% Figure Creation
% Create Figures
if ~exist('figs','var')
    global figs;
    figs = [];
    
    figs(1) = figure;
    figs(2) = figure;
    
    switch algo
        case 1
            figs(3) = figure;
            figs(4) = figure;
    end
    
end

% Clear all figures before running
for f = figs
    change_current_figure(f)
    cla
end
drawnow;
pause(0.1);

%% Scan Matching Loop
startTime = tic;
for scanIdx = start:step:min(stop,size(nScanIndex,1))
    fprintf('Scan %d\n',scanIdx);
    
    %% Get Current Scan
    switch algo
        case {0, 1}
            scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
        case 2
            scan = getLidarPolar(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    end
    
    %% Init World
    if isempty(map)
        % Init map and world
        map = scan;
        tempMap = [];
        switch algo
            case {0, 1}
                tempMap = map;
            case 2
                
                I = map(:,2) < 30;
                [ tempMap(:,1), tempMap(:,2) ] = pol2cart(map(I,1), map(I,2));
        end
        world = tempMap;
        path = pose;
        continue
    end
    
    %% Scan Matching Algo
    switch algo
        case 0
            %% GICP
            T = gicp(init_guess, scan(1:skip:end,:), map(1:skip:end,:), 'minMatchDist', 2, 'costThresh', .00001);
            
        case 1
            %% OLSON
            startOlson = tic;
            
            scan = fillLidarData(scan(1:skip:end,:), 30, 270);
            I = randsample(size(scan,1), min(size(scan,1), 600));
            scan = scan(I,:);
            
            if useScan2World
                % Translate current scan to map coordinates
                dx = pose(1);
                dy = pose(2);
                theta = pose(3);
                
                M = [ cos(theta) -sin(theta) dx;
                    sin(theta)  cos(theta) dy;
                    0           0  1];
                
                scan = [scan(1:skip:end,:) ones(size(scan,1), 1)];
                scan = scan(1:skip:end,:) * M';
                scan = scan(:,[1,2]);
                
                % extract points around the current scan for a map
                map = map(map(:,1) > min(scan(:,1)) - 1, :);
                map = map(map(:,1) < max(scan(:,1)) + 1, :);
                map = map(map(:,2) > min(scan(:,2)) - 1, :);
                map = map(map(:,2) < max(scan(:,2)) + 1, :);
                
                % Limit number of points in the map
                I = randsample(size(map,1), min(size(map,1), 2000));
                map = map(I,:);
            end
            
            % Low Resolution
            [ T, lookupTable_l ] = olson(init_guess, scan(1:skip:end,:), map(1:skip:end,:), ...
                'searchRadius', 3,                     ...
                'lidarStd', 0.03,                      ...
                ...
                'thetaRange', deg2rad(25),            ...
                'dTheta', deg2rad(5), ...
                ...
                'xRange', 0.5,         ...
                'yRange', 0.5,         ...
                'pixelSize', 0.05);
            
            
            % High Resolution
            dRotateRange = deg2rad(5);
            dTranslateRange = 0.1;
            nSearchSteps = 20;
            
            [ T, lookupTable_h ] = olson(T, scan(1:skip:end,:), map(1:skip:end,:), ...
                'searchRadius', 3,                     ...
                'lidarStd', 0.03,                      ...
                ...
                'thetaRange', deg2rad(8),            ...
                'dTheta', deg2rad(.1), ...
                ...
                'xRange', 0.1,         ...
                'yRange', 0.1,         ...
                'pixelSize', 0.02);
            
            
            toc(startOlson)
            
            fprintf('OLSON: Final Guess\n')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')
            
            change_current_figure(figs(3));
            cla
            imagesc(imrotate(lookupTable_l,90))
            %imagesc(1:size(lookupTable_l,1),1:size(lookupTable_l,2),imrotate(lookupTable_l,90))
            colormap(bone)
            axis equal
            title(['Scan: ' num2str(scanIdx)]);
            
            change_current_figure(figs(4));
            cla
            imagesc(imrotate(lookupTable_h,90))
            colormap(bone)
            axis equal
            title(['Scan: ' num2str(scanIdx)]);
            
        case 2
            %% PSM
            [ T, iter, err] = psm(init_guess, scan(1:skip:end,:), map(1:skip:end,:));
            T(3) = -T(3);
            fprintf('PSM: Final Guess\n')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')
            fprintf('PSM: %d iterations with %g error\n', iter, err)
            
    end
    
    %% Transforms
    tempScan = [];
    switch algo
        case {0, 1}
            tempScan = scan;
        case 2
            I = scan(:,2) < 30;
            [ tempScan(:,1), tempScan(:,2) ] = pol2cart(scan(I,1), scan(I,2));
    end
    
    % Rotate translation into map frame
    if useScan2World
        theta = pose(3);
    else
        theta = -pose(3);
    end
    Trans = [ cos(theta) -sin(theta), 0;
        sin(theta)  cos(theta), 0;
        0           0           1];
    mapT  = T * Trans;
    
    % Create Transformation
    pose = pose + [mapT(1:2), -T(3)];
    path(end+1,:) = pose;
    
    dx = pose(1);
    dy = pose(2);
    theta = pose(3);
    
    Trans = [ cos(theta) -sin(theta) dx;
        sin(theta)  cos(theta) dy;
        0           0  1];
    
    % Local Transformation
    dx = T(1);
    dy = T(2);
    theta = -T(3);
    
    LTrans = [ cos(theta) -sin(theta) dx;
        sin(theta)  cos(theta) dy;
        0           0  1];
    
    % Do the Transformation
    temp = [tempScan ones(size(tempScan,1),1)] * Trans';
    tempL = [tempScan ones(size(tempScan,1),1)] * LTrans';
    
    % Add transformed data to world
    if useScan2World
        world = [world; tempL(:,1:2)];
    else
        world = [world; temp(:,1:2)];
    end
    
    %% Plots
    % Plot World
    change_current_figure(figs(1));
    cla
    hold on
    plot(world(:,1), world(:,2), 'k.')
    plot(temp(:,1),temp(:,2),'g.')
    plot(path(:,1), path(:,2), 'r.')
    axis equal
    title(['Scan: ' num2str(scanIdx)]);
    legend('World', 'Registered Scan', 'Path')
    
    
    
    tempMap = [];
    switch algo
        case {0, 1}
            tempMap = map;
        case 2
            I = map(:,2) < 30;
            [ tempMap(:,1), tempMap(:,2) ] = pol2cart(map(I,1), map(I,2));
    end
    
    % Plot Transformed and Map and Scans
    change_current_figure(figs(2));
    cla
    hold on
    plot(tempMap(:,1),tempMap(:,2),'r.')
    plot(tempScan(:,1),tempScan(:,2),'b.')
    plot(tempL(:,1),tempL(:,2),'g.')
    hold off
    axis equal
    title(['Scan: ' num2str(scanIdx)]);
    legend('Reference', 'Current Scan', 'Registered Scan')
    
    %% Init Values for next loop
    if usePrevOffsetAsGuess
        init_guess = T;
    end
    
    
    if useScan2World
        map = world;
    else
        map = scan;
    end
    
    %%
    drawnow
    pause(.1)
end
toc(startTime)
