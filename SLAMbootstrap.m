clc

nScanIndex = unique(Lidar_ScanIndex);

numberOfScans = 100000;
start = 1;
step = 50; % Scans
stop = start + step * numberOfScans;

skip = 1; % Points

map = [];
pose = [0 0 0];
path = [];
world = [];
T = [0 0 0];
init_guess = [0 0 0];
usePrevOffsetAsGuess = true;
useIMUAngle = false;

if useIMUAngle

end

% Create Figures
if ~exist('figs','var')
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


% Scan Matching Loop
startTime = tic;
for scanIdx = start:step:min(stop,size(nScanIndex,1))
    fprintf('Scan %d\n',scanIdx);
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
            startOlson = tic;
            
            scan = fillLidarData(scan(1:skip:end,:), 30, 270);
            I = randsample(size(scan,1), min(size(scan,1), 600));
            scan = scan(I,:);
            
            
            % Low Resolution 
            [ T, lookupTable_l ] = olson(init_guess, scan(1:skip:end,:), map(1:skip:end,:), ... 
                                        'searchRadius', 3,                     ...
                                        'lidarStd', 0.03,                      ...
                                                                               ...
                                        'thetaRange', deg2rad(17),            ...
                                        'dTheta', deg2rad(5), ...
                                                                               ...
                                        'xRange', 0.5,         ...
                                        'yRange', 0.5,         ... 
                                        'pixelSize', 0.06); 
                                    

            % High Resolution
            dRotateRange = deg2rad(5);
            dTranslateRange = 0.1;
            nSearchSteps = 20;
            
            [ T, lookupTable_h ] = olson(T, scan(1:skip:end,:), map(1:skip:end,:), ... 
                                        'searchRadius', 3,                     ...
                                        'lidarStd', 0.03,                      ...
                                                                               ...
                                        'thetaRange', deg2rad(6),            ...
                                        'dTheta', deg2rad(.25), ...
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
            %imagesc(imrotate(lookupTable_l,90))
            imagesc(1:size(lookupTable_l,1),1:size(lookupTable_l,2),imrotate(lookupTable_l,90))
            colormap(bone)
            axis equal
            title(['Scan: ' num2str(scanIdx)]);
            
%             change_current_figure(figs(4));
%             cla
%             imagesc(imrotate(lookupTable_h,90))
%             axis equal
%             title(['Scan: ' num2str(scanIdx)]);
            
        case 2
            [ T, iter, err] = psm(init_guess, scan(1:skip:end,:), map(1:skip:end,:));
            fprintf('PSM: Final Guess\n')
            tmp = T;
            tmp(3) = rad2deg(tmp(3));
            fprintf(['\t' repmat('%g\t', 1, size(tmp, 2)) '\n'], tmp')
            fprintf('PSM: %d iterations with %g error\n', iter, err)
    end
    
    % Rotate translation into map frame
    theta = -pose(3);
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
    temp = [scan ones(size(scan,1),1)] * Trans';
    tempL = [scan ones(size(scan,1),1)] * LTrans';
    
    % Add transformed data to world
    world = [world; temp(:,1:2)];

    
    
    % Plot World
    change_current_figure(figs(1));
    cla
    hold on
    plot(world(:,1), world(:,2), 'k.')
    plot(path(:,1), path(:,2), 'r.')
    axis equal
    title(['Scan: ' num2str(scanIdx)]);

    
    % Plot Transformed and Map and Scans
    change_current_figure(figs(2));
    cla
    hold on
    plot(map(:,1),map(:,2),'r.')
    plot(scan(:,1),scan(:,2),'b.')
    plot(tempL(:,1),tempL(:,2),'g.')
    hold off
    axis equal
    title(['Scan: ' num2str(scanIdx)]);
    legend('Reference', 'Current Scan', 'Registered Scan')
    
    if usePrevOffsetAsGuess
        init_guess = T;
        %init_guess(3) = -T(3);
    end
    
    map = scan;
    
    %I = randsample(size(world,1), min(size(world,1), 10000));
    %map = world(I,:);
    
    
    drawnow
    pause(.1)
end

toc(startTime)