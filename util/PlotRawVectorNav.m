% PlotRawVectorNav.m
%  Sample code for displaying the raw vector nav data. Currently 8 different plots:
%  GPS path over a google map
%  3D plot of gps path
%  Raw sensor data for the LLA and IMU signals
%  Missing data plot (based on timestamps)
%  A plot of the orientations from the IMU in 3D without translation.
%  A Plot of the flat (metric) gps track interpolated for each IMU message (accounts for short missing GPS locks)
%  A plot of the orientations from the IMU over the GPS track


% Load Data
SetParameters
ReadVectorNavLog


% Add google map plot function to path if not already present
if ~exist('plot_google_map.m', 'file')
    if exist('plot_google_map', 'dir') 
        addpath('plot_google_map');
    end
end


% Plot GPS track over a Google Maps image
figure(1)
clf
plot(GPS_Longitude, GPS_Lattitude, '-r', 'MarkerSize', 1)
hold on;
plot(GPS_Longitude, GPS_Lattitude, '+r', 'MarkerSize', 3) 
plot_google_map('MapType', 'hybrid')  % Dont forget to add to path
title('GPS Track');


% Plot GPS track in Flat Earth (Metric) coodinates
figure(2)
clf
plot(GPS_MetricPose(:,1), GPS_MetricPose(:,2),'.');
title('Flat Earth Coordinates');
axis equal;
grid


% Plot the GPS track in 3D
figure(3)
clf
plot3(IMU_MetricPose(:,1), IMU_MetricPose(:,2), IMU_MetricPose(:,3), '.r', 'MarkerSize', 20) 
title('GPS Track (3D)');
grid
axis equal


% Plot the raw GPS location data
figure(4)
clf
subplot(3,1,1);
plot(GPS_Timestamp - GPS_Timestamp(1), GPS_Longitude, '.r')
title('GPS Longitude');
subplot(3,1,2);
plot(GPS_Timestamp - GPS_Timestamp(1), GPS_Lattitude, '.g')
title('GPS Latitude');
subplot(3,1,3);
plot(GPS_Timestamp - GPS_Timestamp(1), GPS_Altitude, '.b')
title('GPS Altitude');


% Identify any missing data
figure(5)
clf  
hold on  
subplot(2,1,1)
plot(diff(GPS_Timestamp), '.r')
title('Missing Data Identification (GPS)')
subplot(2,1,2)
plot(diff(IMU_Timestamp), '.b')
title('Missing Data Identification (IMU)')


% Plot the raw IMU data
figure(6)
clf
subplot(3,1,1);
plot(IMU_Timestamp - IMU_Timestamp(1), rad2deg(IMU_Pitch), '.r')
title('IMU Pitch');
subplot(3,1,2);
plot(IMU_Timestamp - IMU_Timestamp(1), rad2deg(IMU_Roll), '.g')
title('IMU Roll');
subplot(3,1,3);
plot(IMU_Timestamp - IMU_Timestamp(1), rad2deg(IMU_Yaw), '.b')
title('IMU Yaw');

% Plot all orientations on same spot
figure(7)
clf
zv = zeros(size(IMU_Pitch, 1), 3);
PlotTraj3D(zv, IMU_Q, 1);
title('IMU Orientation')
grid
axis equal
view(63, 24)

% Show the interpolated GPS Location for each IMU message
figure(8)
clf
subplot(3,1,1);
plot(IMU_Timestamp - IMU_Timestamp(1), IMU_MetricPose(:,1), '.r')
title('IMU GPS Metric Pose (X)');
subplot(3,1,2);
plot(IMU_Timestamp - IMU_Timestamp(1), IMU_MetricPose(:,2), '.g')
title('IMU GPS Metric Pose (Y)');
subplot(3,1,3);
plot(IMU_Timestamp - IMU_Timestamp(1), IMU_MetricPose(:,3), '.b')
title('IMU GPS Metric Pose (Z)');


% Plot the trajectory of the IMU & GPS with orientations.
figure(9)
clf
PlotTraj3D(IMU_MetricPose, IMU_Q, 1);
title('IMU Orientation & GPS Pose')
axis equal 
grid  
view(63, 24)
       
% Show the interpolated GPS Location for each IMU message
figure(10)
clf
subplot(2,1,1);
plot( IMU_MetricPose(:,1))
subplot(2,1,2);
plot( IMU_MetricPose(:,2))
title('ROI Selection Plot');


clear zv
