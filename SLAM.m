clc

profile on

% Scan ROI Settings
step          = 1;       % Scans
start         = 1;       % Scan Index
numberOfScans = 500000;  % Scan Index
skip          = 1;       % Points


% Framework Options
verbose              = false;
debugplots           = true;

usePrevOffsetAsGuess = false;
useScan2World        = true;

connectTheDots       = false;
ConnectDist          = 0.1;         % (Meters )

MaxVelocityLin       = 3;           % (Meters  / second   )
MaxVelocityRot       = deg2rad(90); % (Radians / second   )
MaxAccelLin          = 0.5;         % (Meters  / second^2 )
MaxAccelRot          = deg2rad(60); % (Radians / second^2 )

MapBorderSize        = 1;           % (Meters )
MapPixelSize         = 0.1;         % (Meters )

SearchResolutionLin  = 0.1;         % (Meters )
SearchResolutionRot  = deg2rad(0.5);  % (Radians )



% Algorithm Specific
switch algo
    case 2
        connectTheDots = false; % doesn't work with PSM
end


% Initialize State Variables
nScanIndex = unique(Lidar_ScanIndex);
stop       = start + step * numberOfScans;

map        = [];
pose       = [0 0 0];
path       = [0 0 0];
world      = [];
T          = [0 0 0];
init_guess = [0 0 0];
LastMapUpdatePose = [0 0 0];



% Clear all figures before running
for i = 1:3
    clearallplots( i );
end



% Scan Matching Loop
startTime = tic;
stopIdx = min(stop,size(nScanIndex,1));
for scanIdx = start:step:stopIdx
    
    % Display current scan index
    fprintf('ScanMatcher: Scan %d / %d\n', scanIdx, stopIdx);
    
    if verbose
        ScanPreMatch = tic;
    end
    
    % Get Current Scan
    switch algo
        case 2
            scan = getLidarPolar(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
        otherwise
            [scan, pol] = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex, ...
                                     'LidarRange', 30);
    end
    
    % Timestamp (missing data compensation)
    stamp = Lidar_Timestamp_Sensor(scanIdx);
       
    
    % Get the orientation from the IMU for each hit
    %Fusion_Q = interp1(IMU_Timestamp, IMU_Q, stamp);

    % Rotate all points by their orientation
    %Fusion_scan = quatrotate(Fusion_Q, [scan, zeros(size(scan,1),1)]);
    
    % Remove points that are probably on the ground or ceiling
    %I = abs(Fusion_scan(:,3)) < 0.2;
    %scan = scan(I, [1,2]);
    
    % Init World
    if isempty(map)
        % Init map and world
        path = pose;
        map  = scan;
        
        tempMap = [];
        switch algo
            case 2
                I = map(:,2) < 30;
                [ tempMap(:,1), tempMap(:,2) ] = pol2cart(map(I,1), map(I,2));
            otherwise
                tempMap = map;
        end
        world = tempMap;
        
        
        if connectTheDots
            % Linear interpolation of 'connected' map points
            map = fillLidarData(map(1:skip:end,:), 270, ConnectDist);
        end
        
        prev_stamp = stamp;
        
        continue
    end
    
    
    % Generate a local map from the world map
    if useScan2World
        
        % Translate current scan to map coordinates
        dx    = init_guess(1);
        dy    = init_guess(2);
        theta = init_guess(3);
        
        M = [ cos(theta) -sin(theta) dx ;
              sin(theta)  cos(theta) dy ;
              0           0          1  ];
        
        scanWorldFrame = [scan(1:skip:end,:) ones(size(scan,1), 1)];
        scanWorldFrame = scanWorldFrame(1:skip:end,:) * M';
        scanWorldFrame = scanWorldFrame(:,[1,2]);
        
        % extract points around the current scan for a reference map
        map = map(map(:,1) > min(scanWorldFrame(:,1)) - MapBorderSize, :);
        map = map(map(:,1) < max(scanWorldFrame(:,1)) + MapBorderSize, :);
        map = map(map(:,2) > min(scanWorldFrame(:,2)) - MapBorderSize, :);
        map = map(map(:,2) < max(scanWorldFrame(:,2)) + MapBorderSize, :);
        
    end
    
    
    % Linear interpolation of 'connected' scan points
    if connectTheDots
        scan = fillLidarData(scan(1:skip:end,:), 270, ConnectDist);
    end
    
    
    % Search area
    if usePrevOffsetAsGuess
        rmax = max(MaxAccelRot*(stamp - prev_stamp), SearchResolutionRot);
        tmax = max(MaxAccelLin*(stamp - prev_stamp), SearchResolutionLin);
    else
        rmax = max(MaxVelocityRot*(stamp - prev_stamp), SearchResolutionRot);
        tmax = max(MaxVelocityLin*(stamp - prev_stamp), SearchResolutionLin);
    end

  
    
    % Initial Guess
    T = init_guess;
    
    if verbose
        fprintf('ScanMatcher:  PreMatch took %.4f seconds. \n', toc(ScanPreMatch))
        ScanMatch = tic;
    end
    
    % Scan Matching Algo
    switch algo
        case 0 % Generalize ICP
            
            T = gicp(T, scan, map,          ...
                     'minMatchDist', 2,     ...
                     'costThresh'  , .00001 );
            
            
        case 1 % Correlative (Olson) 
            
            T = olson(T, scan, map,                  ...
                'searchRadius', 3,                   ... % Standard Deviations
                'lidarStd'    , 0.03,                ...
                'thetaRange'  , rmax,                ...
                'dTheta'      , SearchResolutionRot, ...
                'xRange'      , tmax,                ...
                'yRange'      , tmax,                ...
                'pixelSize'   , MapPixelSize,        ...
                'verbose'     , verbose              );
                       
            
        case 2 % Polar Scan Matching
            
            [ T, iter, err, axs, ays, aths, errs, dxs, dys, dths, stoperr ] = psm(init_guess, scan, map, ...
                'PM_STOP_COND'         ,  0.4,      ...
                'PM_MAX_ITER'          , 30,        ...
                'PM_MAX_RANGE'         , 10,        ...
                'PM_MIN_RANGE'         ,  0.1,      ...
                'PM_WEIGHTING_FACTOR'  ,  0.70*.70, ...
                'PM_SEG_MAX_DIST'      ,  0.20,     ...
                'PM_CHANGE_WEIGHT_ITER', 10,        ...
                'PM_MAX_ERR'           ,  1.00,     ...
                'PM_SEARCH_WINDOW'     , 80         );
            
            fprintf('PSM: Final Guess: ')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g]\n'], tmp')
            fprintf('PSM: %d iterations with %g error\n', iter, err)
            

        case 3  % Hill- Climbing  
                       
            [T, ~    ] = hcm(T, scan, map,         ...
                             'pixelSize'    , 0.1, ...
                             'maxIterations', 15   );
        
            
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
                            'Matching'      , 'kDtree' , ...
                            'Minimize'      , 'point'  , ...
                            'WorstRejection', 0.1);
            
            T(1) = tt(1);
            T(2) = tt(2);
            T(3) = atan2(tr(2,1), tr(1,1));
       
            
        case 6 % ChamferSLAM
 
            [T, hits] = chamferMatch(T, scan, map,               ...
                                      'dTheta'    , rmax,         ...
                                      'dLinear'   , tmax,         ...
                                      'pixelSize' , MapPixelSize, ...
                                      'SearchRot' , SearchResolutionRot, ...
                                      'SearchLin' , SearchResolutionLin, ...
                                      'verbose'   , verbose );
            score  = sum(hits);
             
    end
    
    if verbose
        fprintf('ScanMatcher:     Match took %.4f seconds. \n', toc(ScanMatch))
        ScanPostMatch = tic;
    end
    
    
    
    
    
    
    
    
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
%     if(useSimWorld)
%         fprintf('ScanMatcher: Scan %d pose is ', scanIdx);
%         tmp = pose;
%         tmp(3) = rad2deg(tmp(3));
%         fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g]\n'], tmp')
%     
%         goodPose = LidarPose(scanIdx,1:3);
%         tmp = goodPose;
%         tmp(3) = rad2deg(tmp(3));
%         fprintf('ScanMatcher: Scan %d pose should be ', scanIdx);
%         fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g ]\n'], tmp')
%         poseErr = goodPose - pose;
%         tmp = poseErr;
%         tmp(3) = rad2deg(tmp(3));
%         fprintf('ScanMatcher: Scan %d pose err: ', scanIdx);
%         fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g ]\n'], tmp')
%         
%     end
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
        
        %world = [world; tempL(:,1:2)];
        
        % Update map after distance traveled. 
        dp = abs(LastMapUpdatePose - path(end, :));
        if (dp(1) > 0.1) || (dp(2) > 0.1) || (dp(3) > deg2rad(5))
           LastMapUpdatePose = path(end, :);
                
            % Only add new points to the map 
            I = ~logical(hits);
            
            % Only add points close to the sensor
            %I = I & (pol(:,2) < 5); %cos(deg2rad(0.25)) * 5m = 0.04m error (1 px)
            
            % Add points to the map
            newpts = tempL(I, 1:2);
            world = [world; newpts];
        end
        
        % Update when new hits are detected
%         if score < 1070
%            world = [world; tempL(:,1:2)];
%         end
        
%         if mod(length(path), 4) == 0 || length(path) < 40           
%             world = [world; tempL(:,1:2)];
%         end
    
        
    else
        world = [world; temp(:,1:2)];
    end
    
    
    
    % Debug Plots
    if debugplots  && mod(length(path), 400) == 0 
  
        % Limit number of points in the map
        MaxMapSize = 100000;
        if size(world,1) > MaxMapSize           
            I = randsample(size(world,1), MaxMapSize);
            %I = (size(map,1)-MaxMapSize):size(map,1);
            tmpW = world(I,:);
        else
            tmpW = world;
        end
        
        % Plot World
        change_current_figure(1);
        clf
        plot(tmpW(:,1), tmpW(:,2), 'k.', 'MarkerSize', 1)
        hold on
        plot(path(:,1), path(:,2), 'r.')
        axis equal
        title(['Scan: ' num2str(scanIdx)]);
        drawnow
  
        %set(gcf,'PaperUnits','inches','PaperPosition', [0 0 8.5 11]);
        print([ OutPath DatasetName '-dbg'],'-dpdf');
    
        tmpW = [];
        
%         % Plot Transformed and Map and Scans
%         tempMap = [];
%         switch algo
%             case 2
%                 I = map(:,2) < 30;
%                 [ tempMap(:,1), tempMap(:,2) ] = pol2cart(map(I,1), map(I,2));
%             otherwise
%                 tempMap = map;
%         end     
%         
%         change_current_figure(2);
%         clf
%         hold on
%         plot(tempMap(:,1),tempMap(:,2),'r.', 'MarkerSize', 1)
%         if useScan2World
%             plot(scanWorldFrame(:,1),scanWorldFrame(:,2),'b.', 'MarkerSize', 1)
%         else
%             plot(tempScan(:,1),tempScan(:,2),'b.', 'MarkerSize', 1)
%         end
%         plot(tempL(:,1),tempL(:,2),'g.', 'MarkerSize', 1)
%         hold off
%         axis equal
%         title(['Scan: ' num2str(scanIdx)]);
%         legend('Reference', 'Current Scan', 'Registered Scan')
    end
    
    
    % Select the map for the next scan
    if useScan2World
        map = world;
        
        %MaxMapSize = 100000;
        %sz = size(world, 1);
        %if sz > MaxMapSize            
        %    I = randsample(size(world,1), MaxMapSize);
        %    map = world(I, :);
        %else
        %    map = world;
        %end
        
        %map = [map; tempL(:,1:2)];
%         
%         if mod(length(path), 40) == 0
%             map = world;
%         else
%             map = [map; tempL(:,1:2)];
%         end
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
    
    % Timestamp 
    prev_stamp = stamp;
    
    
    
    if verbose
        fprintf('ScanMatcher: PostMatch took %.4f seconds. \n', toc(ScanPostMatch))
    end
end


toc(startTime)
realTime = Lidar_Timestamp_Sensor(scanIdx) - Lidar_Timestamp_Sensor(start);
fprintf('ScanMatcher: Logfile length %.4f seconds. \n', realTime)

profile off
%profile viewer
%profsave;

% Plot World

% Limit number of points in the map
MaxMapSize = 100000;
if size(world,1) > MaxMapSize           
    I = randsample(size(world,1), MaxMapSize);
    %I = (size(map,1)-MaxMapSize):size(map,1);
    map = world(I,:);
end
 
change_current_figure(1);
clf
hold on
plot(map(:,1), map(:,2), 'k.', 'MarkerSize', 1)
plot(path(:,1), path(:,2), 'r.', 'MarkerSize', 2);
axis equal
title(['Scan: ' num2str(scanIdx)]);

hgsave([ OutPath DatasetName])
%saveas(gcf, '../World.png');
%print('../World','-dpng', '-r300');

set(gcf,'PaperUnits','inches','PaperPosition', [0 0 8.5 11]);
print([ OutPath DatasetName],'-dpdf');


% Plot dT
fl = 5;
n = 1;
change_current_figure(2);
clf
subplot(3,1,1);
plot(diff(path(:,1),n), 'r.')
hold on
plot( tmax * ones(size(path(:,1))), 'b-')
plot(-tmax * ones(size(path(:,1))), 'b-')
title(['X: diff(path(:,1),' num2str(n) ')'])
tmp  = diff(path(:,1),n);
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-b')

subplot(3,1,2);
plot(diff(path(:,2),n), 'g.')
hold on
plot( tmax * ones(size(path(:,1))), 'b-')
plot(-tmax * ones(size(path(:,1))), 'b-')
title(['Y: diff(path(:,2),' num2str(n) ')'])
tmp  = diff(path(:,2),n);
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-b')

subplot(3,1,3);
plot(rad2deg(diff(path(:,3),n)), 'b.')
hold on
plot( rad2deg(rmax) * ones(size(path(:,1))), 'b-')
plot(-rad2deg(rmax) * ones(size(path(:,1))), 'b-')
title(['Z: diff(path(:,3),' num2str(n) ')'])
tmp  = rad2deg(diff(path(:,3),n));
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-r')

print( [OutPath 'pathDiff1'],'-dpng');



% Plot dT2
n = 2;
change_current_figure(3);
clf
subplot(3,1,1);
plot(diff(path(:,1),n), 'r.')
hold on
plot( tmax * ones(size(path(:,1))), 'b-')
plot(-tmax * ones(size(path(:,1))), 'b-')
title(['X: diff(path(:,1),' num2str(n) ')'])
tmp  = diff(path(:,1),n);
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-b')

subplot(3,1,2);
plot(diff(path(:,2),n), 'g.')
hold on
plot( tmax * ones(size(path(:,1))), 'b-')
plot(-tmax * ones(size(path(:,1))), 'b-')
title(['Y: diff(path(:,2),' num2str(n) ')'])
tmp  = diff(path(:,2),n);
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-b')

subplot(3,1,3);
plot(rad2deg(diff(path(:,3),n)), 'b.')
hold on
plot( rad2deg(rmax) * ones(size(path(:,1))), 'b-')
plot(-rad2deg(rmax) * ones(size(path(:,1))), 'b-')
title(['Z: diff(path(:,3),' num2str(n) ')'])
tmp  = rad2deg(diff(path(:,3),n));
tmp2 = conv(tmp, ones(fl,1) / fl);
plot(tmp2, '-r')

print([OutPath 'pathDiff2'],'-dpng');


