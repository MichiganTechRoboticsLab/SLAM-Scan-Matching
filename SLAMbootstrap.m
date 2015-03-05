clc


% Scan Settings
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
switch algo
    case 2
        connectTheDots = false;
    otherwise
        connectTheDots = false;
end


% Clear all figures before running
for i = 1:8
    if ~ishandle(i);
        figure(i);
    end
    
    change_current_figure(i);
    clf;
end
% drawnow;
pause(0.1);


% Scan Matching Loop
startTime = tic;
for scanIdx = start:step:min(stop,size(nScanIndex,1))
    
    % Display current scan index
    fprintf('ScanMatcher: Scan %d\n', scanIdx);
    
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
        
        
        if connectTheDots
            % Linear interpolation of 'connected' map points
            m = min(sqrt(map(:,1).^2 + map(:,2).^2));
            d = m * sin(deg2rad(270/1081));
            map = fillLidarData(map(1:skip:end,:), 270, d);
            
            % Limit number of points in the map
            I = randsample(size(map,1), min(size(map,1), 300));
            %map = map(I,:);
        end
        
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
    
    if connectTheDots
        % Linear interpolation of 'connected' scan points
        m = min(sqrt(scan(:,1).^2 + scan(:,2).^2));
        d = m * sin(deg2rad(270/1081));
        scan = fillLidarData(scan(1:skip:end,:), 270, d);
        
        % Limit number of points in the map
        I = randsample(size(scan,1), min(size(scan,1), 300));
        %scan = scan(I,:);
    end
    
    % Scan Matching Algo
    ScanMatch = tic;
    switch algo
        case 0
            T = gicp(init_guess, scan(1:skip:end,:), map(1:skip:end,:), 'minMatchDist', 2, 'costThresh', .00001);
            
        case 1
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
            
            
            fprintf('OLSON: Final Guess: ')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g]\n'], tmp')
            
        case 2
            [ T, iter, err, axs, ays, aths, errs, dxs, dys, dths, stoperr ] = psm(init_guess, scan(1:skip:end,:), map(1:skip:end,:), ...
                'PM_STOP_COND', .04,                                               ...
                'PM_MAX_ITER', 30,                                                   ...
                'PM_MAX_RANGE', 3000,                                                  ...
                'PM_MIN_RANGE', 10,                                                  ...
                'PM_WEIGHTING_FACTOR', 30*30,                                        ...
                'PM_SEG_MAX_DIST', 10,                                               ...
                'PM_CHANGE_WEIGHT_ITER', 10,                                         ...
                'PM_MAX_ERR', 10,                                                    ...
                'PM_SEARCH_WINDOW', 200);
            fprintf('PSM: Final Guess: ')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g]\n'], tmp')
            fprintf('PSM: %d iterations with %g error\n', iter, err)
            

        case 3  % Hill- Climbing                        
            searchStep = 0.5;
            maxIterations = 30;
            
            [T, ~    ] = hcm(init_guess, scan, map, 'pixelSize', searchStep      , 'maxIterations', maxIterations);
            [T, ~    ] = hcm(T         , scan, map, 'pixelSize', searchStep * 1/2, 'maxIterations', maxIterations);
            [T, ~    ] = hcm(T         , scan, map, 'pixelSize', searchStep * 1/4, 'maxIterations', maxIterations);
            [T, ~    ] = hcm(T         , scan, map, 'pixelSize', searchStep * 1/8, 'maxIterations', maxIterations);
            [T, ~    ] = hcm(T         , scan, map, 'pixelSize', 0.03            , 'maxIterations', maxIterations);
            
    end
    fprintf('ScanMatcher: Scan %d matched in %.1f seconds. \n', scanIdx, toc(ScanMatch))
    
    % Update current pose
    if useScan2World
        pose = T;
    else
        % Rotate translation into map frame
        theta = pose(3);
        Trans = [ cos(theta) -sin(theta), 0;

                  sin(theta)  cos(theta), 0;
                  0           0           1];
        mapT  = Trans * T';


        % Add previous scan to pose
        pose = pose + [mapT(1:2)', T(3)];
    end
    fprintf('ScanMatcher: Scan %d pose is ', scanIdx);
    tmp = pose;
    tmp(3) = rad2deg(tmp(3));
    fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g]\n'], tmp')
    if(useSimWorld)
        goodPose = LidarPose(scanIdx,1:3);
        tmp = goodPose;
        tmp(3) = rad2deg(tmp(3));
        fprintf('ScanMatcher: Scan %d pose should be ', scanIdx);
        fprintf(['[ ' repmat('%g ', 1, size(goodPose, 2)-1) '%g ]\n'], goodPose')
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
    theta = T(3);
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
    change_current_figure(1);
    cla
    hold on
    plot(world(:,1), world(:,2), 'k.', 'MarkerSize', 1)
    plot(path(:,1), path(:,2), 'r.')
    axis equal
    title(['Scan: ' num2str(scanIdx)]);
    
    
    % Plot Transformed and Map and Scans
    change_current_figure(2);
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
            change_current_figure(3);
            cla
            imagesc(imrotate(lookupTable_l,90))
            colormap(bone)
            axis equal
            title(['Low-resolution lookup table, Scan: ' num2str(scanIdx)]);
            
            % High resolution lookup table
            change_current_figure(4);
            cla
            imagesc(imrotate(lookupTable_h,90))
            colormap(bone)
            axis equal
            title(['High-resolution lookup table, Scan: ' num2str(scanIdx)]);
        case 2  % PSM
            change_current_figure(3);
            cla
            
            iters = 1:iter;
            
            subplot(4,1,1);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, dxs(iters), 'o-');
            title('Evolution of X');
            axis([iters(1),max(2,iters(end)),-1.2*max(abs(dxs(iters)))-.1,1.2*max(abs(dxs(iters)))+.1])
            
            subplot(4,1,2);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, dys(iters), 's-');
            title('Evolution of Y');
            axis([iters(1),max(2,iters(end)),-1.2*max(abs(dys(iters)))-.1,1.2*max(abs(dys(iters)))+.1])
            
            subplot(4,1,3);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, rad2deg(dths(iters)),'x-');
            title('Evolution of Ө');
            axis([iters(1),max(2,iters(end)),-1.2*max(abs(rad2deg(dths(iters))))-.1,1.2*max(abs(rad2deg(dths(iters))))+.1])
            
            subplot(4,1,4);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, stoperr(iters),'.-');
            title('Evolution of Err');
            axis([iters(1),max(2,iters(end)),-1.2*max(abs(stoperr(iters)))-.1,1.2*max(abs(stoperr(iters)))+.1])
%         case 3  % Hill Climbing
% 
%             % Occupancy Grid
%             change_current_figure(3);
%             cla
%             imagesc(imrotate(ogrid.grid,90))
%             axis equal
%             colormap([1 1 1; 0.5 0.5 0.5; 0 0 0]);
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

