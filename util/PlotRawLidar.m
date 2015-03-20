% PlotRawLidar.m
%  Plots the raw Lidar data all on one plot, and one scan at a time.


% Load Data
SetParameters
ReadHokuyoLog


% Plot of all hits on a 2D polar plot
figure(1);
clf;
polar(Lidar_Angles, Lidar_Ranges, '.b');


% Identify missing data / Bad timestamps
figure(2)
clf  
plot(diff(Lidar_Timestamp), '.r')
title('Missing Data Identification')


%% Plot each scan individually along the Z Axis (Pseudo 3D)
n = size(Lidar_X,1);
I = randsample(n,min(n,20000));
l = 750;
z = (0:(l/(n-1)):l)';

figure(3);
plot3(Lidar_X(I), Lidar_Y(I), z(I), '.')
title('Plot of each scan along the Z Axis (Pseudo 3D)')

% View in PCL_Veiwer
% Add library to path if not already present
if ~exist('pclviewer.m', 'file')
    if exist('matpcl', 'dir') 
        addpath('matpcl');
    end
end

% View the Full Pointcloud
pc = [Lidar_X, Lidar_Y, z];
pclviewer(pc') 


%% Plot each scan individually
nScanIndex = unique(Lidar_ScanIndex);

figure(4);
clf
for i = 9000:length(nScanIndex)
   
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    la = Lidar_Angles(I,:);
    lr = Lidar_Ranges(I,:);
    
    % Plot the scan
    set(0, 'CurrentFigure', 4);
    clf;
    polar(0, 25, '.r'); % axis limit hack
    hold on
    polar(la, lr, '.b');
    %view(-90,90);
    drawnow();
    pause(0.01);
end


    
    
    