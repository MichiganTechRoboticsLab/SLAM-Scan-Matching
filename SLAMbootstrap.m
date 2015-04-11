clc

profile on

% Scan ROI Settings
step          = 3;       % Scans
start         = 1;       % Scan Index
numberOfScans = 50000;   % Scan Index
skip          = 1;       % Points


% Framework Options
verbose              = false;
debugplots           = true;

usePrevOffsetAsGuess = true;
useScan2World        = true;

connectTheDots       = false;
ConnectDist          = 0.1;         % (Meters)

SensorHz             = 40;          % (Hz)
MaxVelocityLin       = 1;           % (Meters  / second   )
MaxVelocityRot       = deg2rad(45); % (Radians / second   )
MaxAccelLin          = 0.5;         % (Meters  / second^2 )
MaxAccelRot          = deg2rad(15); % (Radians / second^2 )

MapBorderSize        = 1;           % (Meters)
MapPixelSize         = 0.03;        % (Meters)

SearchResolutionRot  = deg2rad(.25); % (Radians)
SearchResolutionLin  = 0.05;        % (Meters)



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
clearallplots( 1 );


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
            scan = getLidarXY(scanIdx, nScanIndex, Lidar_Angles, Lidar_Ranges, Lidar_ScanIndex);
    end
    
    % Timestamp (missing data compensation)
    stamp = Lidar_Timestamp_Sensor(scanIdx);
       
    
    % Init World
    if isempty(map)
        % Init map and world
        path = pose;
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
        
        
        if connectTheDots
            % Linear interpolation of 'connected' map points
            map = fillLidarData(map(1:skip:end,:), 270, ConnectDist);
        end
        
        prev_stamp = stamp;
        
        continue
    end
    
    % Only update map when pose has changed
    if length(path) > 10
      updatemap = sum(path(end-1,:) ~= path(end,:));
    else
      updatemap = true;
    end
    
    % Generate a local map from the world map
    if useScan2World && updatemap
        
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
        map = map(map(:,1) > min(scanWorldFrame(:,1)) - MapBorderSize, :);
        map = map(map(:,1) < max(scanWorldFrame(:,1)) + MapBorderSize, :);
        map = map(map(:,2) > min(scanWorldFrame(:,2)) - MapBorderSize, :);
        map = map(map(:,2) < max(scanWorldFrame(:,2)) + MapBorderSize, :);
        
%         % Limit number of points in the map
%         MaxMapSize = 10000;
%         if size(map,1) > MaxMapSize           
%             I = randsample(size(map,1), MaxMapSize);
%             %I = (size(map,1)-MaxMapSize):size(map,1);
%             map = map(I,:);
%         end
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

    % Search Iterations (ChamferSLAM)
    % Finds the number of searach iterations required to meet the accuracy goal
    iterations = ceil(max(log(rmax/SearchResolutionRot)/log(2),  ...
                          log(tmax/SearchResolutionLin)/log(2) ));
    iterations = max(iterations, 1);
    
    
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

            [T, score] = chamferMatch(T, scan, map,               ...
                                      'dTheta'    , rmax,         ...
                                      'dLinear'   , tmax,         ...
                                      'iterations', iterations,   ...
                                      'pixelSize' , MapPixelSize, ...
                                      'verbose'   , verbose );
            
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
        %dp = abs(LastMapUpdatePose - path(end, :));
        %if (dp(1) > 0.5) || (dp(2) > 0.5) || (dp(3) > deg2rad(10))
        %    LastMapUpdatePose = path(end, :);
        %if score < 1080
            world = [world; tempL(:,1:2)];
        %end
    else
        world = [world; temp(:,1:2)];
    end
    
    
    
    % Debug Plots
    if debugplots && mod(length(path), 200) == 0 
        % Plot World
        change_current_figure(1);
        cla
        hold on
        plot(world(:,1), world(:,2), 'k.', 'MarkerSize', 1)
        plot(path(:,1), path(:,2), 'r.')
        axis equal
        title(['Scan: ' num2str(scanIdx)]);
        drawnow
  
    
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
realTime = Lidar_Timestamp(end) - Lidar_Timestamp(1);
fprintf('ScanMatcher: Logfile length %.4f seconds. \n', realTime)

profile off
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

hgsave([ '../' DatasetName])
%saveas(gcf, '../World.png');
%print('../World','-dpng', '-r300');

set(gcf,'PaperUnits','inches','PaperPosition', [0 0 8.5 11]);
print([ '../' DatasetName],'-dpdf');

% 
% % Plot dT
% n = 1;
% change_current_figure(2);
% clf
% subplot(3,1,1);
% plot(diff(path(:,1),n), 'r.')
% title('X: diff(path(:,1),n)')
% 
% subplot(3,1,2);
% plot(diff(path(:,2),n), 'g.')
% title('Y: diff(path(:,2),n)')
% 
% subplot(3,1,3);
% plot(rad2deg(diff(path(:,3),n)), 'b.')
% title('Z: diff(path(:,3),n)')
% print('../pathDiff1','-dpng');



% % Plot dT2
% n = 2;
% change_current_figure(3);
% clf
% subplot(3,1,1);
% plot(diff(path(:,1),n), 'r.')
% title('X: diff(path(:,1),n)')
% 
% subplot(3,1,2);
% plot(diff(path(:,2),n), 'g.')
% title('Y: diff(path(:,2),n)')
% 
% subplot(3,1,3);
% plot(rad2deg(diff(path(:,3),n)), 'b.')
% title('Z: diff(path(:,3),n)')
% print('../pathDiff2','-dpng');


