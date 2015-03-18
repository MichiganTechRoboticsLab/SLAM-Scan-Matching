clc

% Scan ROI Settings
start         = 1;
step          = 1; % Scans
numberOfScans = 100000;
skip          = 1; % Points


% Framework Options
usePrevOffsetAsGuess = true;
useScan2World = true;
connectTheDots = false;
ConnectDist = 0.1;
plotit = false;

% Chamfer-SLAM
rmin = deg2rad(1);
tmin = 0.05;


% Algorithm Specific
switch algo
    case 2
        connectTheDots = false; % doesn't work with algo 2
end


% Initialize State Variables
nScanIndex = unique(Lidar_ScanIndex);
stop = start + step * numberOfScans;

map = [];
pose = [0 0 0];
path = [0 0 0];
world = [];
T = [0 0 0];
init_guess = [0 0 0];


% Clear all figures before running
for i = 1:8
    if ~ishandle(i);
        figure(i);
    end
    
    change_current_figure(i);
    clf;
end
drawnow;
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
            %m = min(sqrt(map(:,1).^2 + map(:,2).^2));
            %d = m * sin(deg2rad(270/1081));
            map = fillLidarData(map(1:skip:end,:), 270, ConnectDist);
            
            % Limit number of points in the map
            %I = randsample(size(map,1), min(size(map,1), 300));
            %map = map(I,:);
        end
        
        continue
    end
    
    % Generate a local map from the world map
    if useScan2World
        
        % Translate current scan to map coordinates
        dx    = pose(1);
        dy    = pose(2);
        theta = pose(3);
        
        M = [ cos(theta) -sin(theta) dx ;
              sin(theta)  cos(theta) dy ;
              0           0          1  ];
        
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
        I = randsample(size(map,1), min(size(map,1), 8000));
        %map = map(I,:);
    end
    
    if connectTheDots
        % Linear interpolation of 'connected' scan points
        %m = min(sqrt(scan(:,1).^2 + scan(:,2).^2));
        %d = m * sin(deg2rad(270/1081));
        scan = fillLidarData(scan(1:skip:end,:), 270, ConnectDist);
        
        % Limit number of points in the map
        %I = randsample(size(scan,1), min(size(scan,1), 300));
        %scan = scan(I,:);
    end
    
    
    % Initial Guess
    T = init_guess;
    
    
    % Scan Matching Algo
    ScanMatch = tic;
    switch algo
        case 0
            T = gicp(init_guess, scan(1:skip:end,:), map(1:skip:end,:), 'minMatchDist', 2, 'costThresh', .00001);
            
        case 1
%             % Low Resolution
%             [ T, lookupTable_l ] = olson(init_guess, scan(1:skip:end,:), map(1:skip:end,:), ...
%                 'searchRadius', 4,                     ...
%                 'lidarStd', 0.04,                      ...
%                 ...
%                 'thetaRange', deg2rad(30) * step / 50,            ...
%                 'dTheta', deg2rad(1) * step / 50, ...
%                 ...
%                 'xRange', 1 * step / 50,         ...
%                 'yRange', 1 * step / 50,         ...
%                 'pixelSize', 0.1);
            
            
            % High Resolution
            [ T, lookupTable_h ] = olson(T, scan(1:skip:end,:), map(1:skip:end,:), ...
                'searchRadius', 4,          ...
                'lidarStd', 0.01,           ...
                                            ...
                'thetaRange', deg2rad(2),   ...
                'dTheta', deg2rad(.1),      ...
                                            ...
                'xRange', 0.1,              ...
                'yRange', 0.1,              ...
                'pixelSize', 0.05);
            
            
            fprintf('OLSON: Final Guess: ')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g]\n'], tmp')
            
        case 2
            [ T, iter, err, axs, ays, aths, errs, dxs, dys, dths, stoperr ] = psm(init_guess, scan(1:skip:end,:), map(1:skip:end,:), ...
                'PM_STOP_COND', .4,                                               ...
                'PM_MAX_ITER', 30,                                                   ...
                'PM_MAX_RANGE', 10,                                                  ...
                'PM_MIN_RANGE', .1,                                                  ...
                'PM_WEIGHTING_FACTOR', .70*.70,                                        ...
                'PM_SEG_MAX_DIST', .20,                                               ...
                'PM_CHANGE_WEIGHT_ITER', 10,                                         ...
                'PM_MAX_ERR', 1.00,                                                    ...
                'PM_SEARCH_WINDOW', 80);
            fprintf('PSM: Final Guess: ')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g]\n'], tmp')
            fprintf('PSM: %d iterations with %g error\n', iter, err)
            

        case 3  % Hill- Climbing  
            
            %while searchStep > 0.03
            %    T = init_guess;            
            %    [T, ~    ] = hcm(T, scan, map, 'pixelSize', searchStep      , 'maxIterations', maxIterations);
            %    searchStep = searchStep * .5;
            %end
            
            
            searchStep = 0.1;
            maxIterations = 15;
            [T, ~    ] = hcm(T, scan, map, ...
                             'pixelSize'    , searchStep, ...
                             'maxIterations', maxIterations);
        
            
        case 4 % libicp
           
            ti = [ cos(T(3)) -sin(T(3)) T(1) ;
                   sin(T(3))  cos(T(3)) T(2) ;
                   0          0         1    ];
            
            % Often seg faults.....
            t = icpMex(map', scan', ti, 0.2, 'point_to_point');
            
            T(1) = t(1,3);
            T(2) = t(2,3);
            T(3) = atan2(t(2,1), t(1,1));
            
        
        case 5 % ICP1
            
            maxIterations = 30;
            
            tt = [ T(1);      T(2);     0 ];            
            tr = [ cos(T(3)) -sin(T(3)) 0 ;
                   sin(T(3))  cos(T(3)) 0 ;
                   0          0         1 ];
               
            p = [map  zeros(size(map ,1), 1)]';
            q = [scan zeros(size(scan,1), 1)]';
            
            % Kinda slow...
            [tr, tt] = icp1(p, q, maxIterations, tt, tr, ...
                            'Matching', 'kDtree', ...
                            'Minimize', 'point', ...
                            'WorstRejection', 0.1);
                        
            T(1) = tt(1);
            T(2) = tt(2);
            T(3) = atan2(tr(2,1), tr(1,1));
       
            
        case 6
            
            
            % This is equivilant to a hi-lo search
            r = max(deg2rad(4)*(step/20), rmin);
            t = max(0.2       *(step/20), tmin);
            
            while t >= tmin || r >= rmin          
            %for i = 1:3    
                T = chamferMatch(T, scan, map, ...
                    'thetaRange', r,           ...
                    'dTheta', r,               ...
                                               ...
                    'xRange', t,               ...
                    'yRange', t,               ...
                    'pixelSize', t,            ...
                    'verbose', true );
                
                r = r/2;
                t = t/2;
            end
        
            
    end
    fprintf('ScanMatcher: Scan %d matched in %.1f seconds. \n', scanIdx, toc(ScanMatch))
    
    
    
    
    % Update current pose
    if useScan2World
        pose = T;
    else
        % Rotate translation into map frame
        theta = pose(3);
        Trans = [ cos(theta) -sin(theta) 0 ;

                  sin(theta)  cos(theta) 0 ;
                  0           0          1 ];
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
        fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g ]\n'], tmp')
        poseErr = goodPose - pose;
        tmp = poseErr;
        tmp(3) = rad2deg(tmp(3));
        fprintf('ScanMatcher: Scan %d pose err: ', scanIdx);
        fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g ]\n'], tmp')
        
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
    dx    = pose(1);
    dy    = pose(2);
    theta = pose(3);
    
    Trans = [ cos(theta) -sin(theta) dx ;
              sin(theta)  cos(theta) dy ;
              0           0          1  ];
    temp =  [tempScan ones(size(tempScan,1),1)] * Trans';
    
    
    % Current Scan Transformation
    dx    = T(1);
    dy    = T(2);
    theta = T(3);
    
    LTrans = [ cos(theta) -sin(theta) dx ;
               sin(theta)  cos(theta) dy ;
               0           0          1  ];
    tempL = [tempScan ones(size(tempScan,1),1)] * LTrans';
    
    
    % Add transformed data to world
    if useScan2World
        world = [world; tempL(:,1:2)];
    else
        world = [world; temp(:,1:2)];
    end
    
    
    if plotit 
        % Plot World
        change_current_figure(1);
        cla
        hold on
        plot(world(:,1), world(:,2), 'k.', 'MarkerSize', 1)
        plot(path(:,1), path(:,2), 'r.')
        axis equal
        title(['Scan: ' num2str(scanIdx)]);
    end
    
    % Plot Transformed and Map and Scans
    if plotit
        change_current_figure(2);
        cla
        hold on
    end
    
    tempMap = [];
    switch algo
        case 2
            I = map(:,2) < 30;
            [ tempMap(:,1), tempMap(:,2) ] = pol2cart(map(I,1), map(I,2));
        otherwise
            tempMap = map;
    end
    
    if plotit
        plot(tempMap(:,1),tempMap(:,2),'r.', 'MarkerSize', 1)
        if useScan2World
            plot(scanWorldFrame(:,1),scanWorldFrame(:,2),'b.', 'MarkerSize', 1)
        else
            plot(tempScan(:,1),tempScan(:,2),'b.', 'MarkerSize', 1)
        end
        plot(tempL(:,1),tempL(:,2),'g.', 'MarkerSize', 1)
        hold off
        axis equal
        title(['Scan: ' num2str(scanIdx)]);
        legend('Reference', 'Current Scan', 'Registered Scan')
    end
    
    % Algorithm specific plots
    if plotit
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
    %             change_current_figure(3);
    %             cla
    %             
    %             iters = 1:iter;
    %             
    %             subplot(4,1,1);
    %             curticks = get(gca, 'XTick');
    %             set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
    %             plot(iters, dxs(iters), 'o-');
    %             title('Evolution of X');
    %             axis([iters(1),max(2,iters(end)),-1.2*max(abs(dxs(iters)))-.1,1.2*max(abs(dxs(iters)))+.1])
    %             
    %             subplot(4,1,2);
    %             curticks = get(gca, 'XTick');
    %             set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
    %             plot(iters, dys(iters), 's-');
    %             title('Evolution of Y');
    %             axis([iters(1),max(2,iters(end)),-1.2*max(abs(dys(iters)))-.1,1.2*max(abs(dys(iters)))+.1])
    %             
    %             subplot(4,1,3);
    %             curticks = get(gca, 'XTick');
    %             set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
    %             plot(iters, rad2deg(dths(iters)),'x-');
    %             title('Evolution of Ө');
    %             axis([iters(1),max(2,iters(end)),-1.2*max(abs(rad2deg(dths(iters))))-.1,1.2*max(abs(rad2deg(dths(iters))))+.1])
    %             
    %             subplot(4,1,4);
    %             curticks = get(gca, 'XTick');
    %             set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
    %             plot(iters, stoperr(iters),'.-');
    %             title('Evolution of Err');
    %             axis([iters(1),max(2,iters(end)),-1.2*max(abs(stoperr(iters)))-.1,1.2*max(abs(stoperr(iters)))+.1])
    %         case 3  % Hill Climbing
    % 
    %             % Occupancy Grid
    %             change_current_figure(3);
    %             cla
    %             imagesc(imrotate(ogrid.grid,90))
    %             axis equal
    %             colormap([1 1 1; 0.5 0.5 0.5; 0 0 0]);
        end
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
    
    
    if plotit
        drawnow
        %pause(.1)
    end
end

toc(startTime)




% Plot World
change_current_figure(1);
cla
hold on
plot(world(:,1), world(:,2), 'k.', 'MarkerSize', 1)
plot(path(:,1), path(:,2), 'r.')
axis equal
title(['Scan: ' num2str(scanIdx)]);
print('../World','-dpng');


% Plot dT
n = 1;
change_current_figure(2);
clf
subplot(3,1,1);
plot(diff(path(:,1),n), 'r.')
title('X: diff(path(:,1),n)')

subplot(3,1,2);
plot(diff(path(:,2),n), 'g.')
title('Y: diff(path(:,2),n)')

subplot(3,1,3);
plot(diff(path(:,3),n), 'b.')
title('Z: diff(path(:,3),n)')
print('../pathDiff1','-dpng');


% Plot dT2
n = 2;
change_current_figure(3);
clf
subplot(3,1,1);
plot(diff(path(:,1),n), 'r.')
title('X: diff(path(:,1),n)')

subplot(3,1,2);
plot(diff(path(:,2),n), 'g.')
title('Y: diff(path(:,2),n)')

subplot(3,1,3);
plot(diff(path(:,3),n), 'b.')
title('Z: diff(path(:,3),n)')
print('../pathDiff1','-dpng');


