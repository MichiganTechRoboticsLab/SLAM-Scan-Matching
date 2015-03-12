% Simulate a 2D robot
% Author: Dereck Wonnacott (2014)

%
% Generate the virtual world (Ground Truth Map)
%

% Sensor Parameters
Lidar_AngleStart = -2.3562;
Lidar_AngleEnd   =  2.3562;

% Number of points per scan
Lidar_nPoints = 1081;

% W is an array of objects in the map
% Each object is a polyline stored in a 2xn matrix
% Where n is the number of line endpoints
% the first row is the x coordinates
% the second row is the y coordinates.
World = [];
World{1} = [-3 -3 3 3;
    0 5 5 0];


% Plot the world
figure(1);
clf;
title('World Map');
for i = 1:length(World)
    line(World{i}(1,:), World{i}(2,:))
end
hold on;
axis equal
grid


%
% Generate Robot Path (Ground Truth)
%

% Robot Trajectory
% [x, y, Theta]
Path = [0, 0, deg2rad(0);
    1, -1 , deg2rad(-15)];

PathVelLin = sqrt(2);  % Linear Velocity (Units/Second)
PathVelRot = deg2rad(15); % Rotational velocity (Rad/Second)

% Plot Trajectory
figure(1);
plot(Path(:,1)', Path(:,2)', '-r');


% Generate timestamps for each trajectory vertex
tt = 0;
for i = 1:(size(Path,1)-1)
    dp = sqrt((Path(i,1) - Path(i+1,1))^2 + (Path(i,2) - Path(i+1,2))^2);
    tp = dp/PathVelLin;
    
    dr = Path(i,3) - Path(i+1,3);
    tr = dr/PathVelRot;
    
    t  = max(tp, tr);
    tt = cat(2, tt, t);
end
Path = cat(2, Path, tt');


%
% Generate Lidar Data
%

LidarHz = 1; % Samples Per Second

% lidar poses
LidarPose = [];

for i = 1:(size(Path,1)-1)
    x  = [0; tt(i+1)];
    xx = x(1):1/LidarHz:x(2);
    
    y  = [Path(i,1); Path(i+1,1)];
    px = linterp(x,y,xx);
    
    y  = [Path(i,2); Path(i+1,2)];
    py = linterp(x,y,xx);
    
    y  = [Path(i,3); Path(i+1,3)];
    pr = linterp(x,y,xx);
    
    LidarPose = cat(1, LidarPose, [px' py' pr' xx']);
end

plot(LidarPose(:,1),LidarPose(:,2), '.r');
for i = 1:size(LidarPose,1)
    plot([LidarPose(i,1), LidarPose(i,1) + cos(LidarPose(i,3))], [LidarPose(i,2),LidarPose(i,2)+ sin(LidarPose(i,3))], '.g-');
end

if 1
    % Generate Lidar Sensor measurements
    LidarRange = 30.0;
    da = (Lidar_AngleEnd - Lidar_AngleStart) / (Lidar_nPoints - 1);
    LidarAngles = (Lidar_AngleStart:da:Lidar_AngleEnd) + pi/2;
    LidarScan = zeros(size(LidarPose,1), size(LidarAngles,2));
    
    for n = 1:size(LidarPose,1) % For each pose
        p = LidarPose(n,:); % Current pose
        z = []; % Current scan
        
        for i = 1:length(LidarAngles); % For each beam
            r = LidarRange; % Max Range
            a = LidarAngles(i); % Current Angle
            
            % find endpoints of this beam
            beam = [p(1), p(1)+r*cos(p(3)+a); p(2), p(2)+r*sin(p(3)+a)];
            
            % Search if this range measurement intersects any map line
            for j = 1:length(World)
                [xi, yi] = polyxpoly(World{j}(1,:), World{j}(2,:), beam(1,:), beam(2,:));
                if ~isempty(xi)
                    % Find the closest intersecetion
                    for k = 1:length(xi)
                        d = sqrt((p(1) - xi(k)).^2 + (p(2) - yi(k)).^2);
                        if d < r
                            r = d;
                        end
                    end
                end
            end
            
            % Add this measurement to the scan
            z = cat(2, z, r);
        end
        
        LidarScan(n,:) = z;
    end
    Test_Lidar_Angles = LidarAngles;
    Test_Lidar_ScanCount = size(LidarScan,1);
    Test_Lidar_ScanIndex = (1:Test_Lidar_ScanCount)';
    
    Test_Lidar_ScanIndex = repmat(Test_Lidar_ScanIndex, 1, Lidar_nPoints);
    Test_Lidar_Angles    = repmat(Test_Lidar_Angles, Test_Lidar_ScanCount, 1);
    
    Test_Lidar_Ranges = reshape(LidarScan',[],1);
    Test_Lidar_ScanIndex = reshape(Test_Lidar_ScanIndex',[],1);
    Test_Lidar_Angles = reshape(Test_Lidar_Angles',[],1);
    
end
% Plot the Lidar Measurements
if 1
    figure(2)
    for nScan = 1:1:size(LidarScan,1)
        a = LidarAngles;
        z = LidarScan(nScan, :);
        
        %  Remove out of range measurements
%         I = (z >= LidarRange);
%         a(I) = [];
%         z(I) = [];
        
        % Plot
        set(0, 'CurrentFigure', 2);
        clf
        [x, y] = pol2cart(a,z);
        plot(x,y,'.b')
        axis equal tight
        %polar(a, z, '.b')
        %         hold on
        % Realtime playback
        pause(1/LidarHz);
    end
end

% Clean up workspace
clear a beam d dp dr i j k n p pr px py r t tp tr tt x xi xx y yi z
