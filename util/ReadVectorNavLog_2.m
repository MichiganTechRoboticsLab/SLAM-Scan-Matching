% ReadVectorNavLog_2.m
%
%  Second version of the read vectornav file
%
%  Reads the vectornav log file into matlab and makes variables for each data type.  
%  Generates a single timestamp value from the two log file columns, converts the 
%  LLA to metric positions, and converts the IMU's RPY  valued into quaternions. 
%  It also filters out all zero LLA values from the gps data. So the IMU and GPS
%  data arrays may not be the same length!



if ~exist('IMU_YawBias', 'var') 
        IMU_YawBias = 0;
end
if ~exist('IMU_RollBias', 'var') 
        IMU_RollBias = 0;
end
if ~exist('IMU_PitchBias', 'var') 
        IMU_PitchBias = 0;
end


% Extract GPS coodinates

% Ignore entries where no GPS data was available
i = VectorNav_Log(:,9) ~= 0;

GPS_Timestamp = VectorNav_Log(i,1);
GPS_Lattitude = VectorNav_Log(i,9);
GPS_Longitude = VectorNav_Log(i,10);
GPS_Altitude  = VectorNav_Log(i,11);



% Extract IMU Orientation

% Ignore entries where no IMU data was available
i = VectorNav_Log(:,5) ~= 0;

IMU_Timestamp = VectorNav_Log(i,1);

% VectorNav Quaternion format: X Y Z W
% Matlab Quaternion format:    W X Y Z
IMU_Q = [VectorNav_Log(i,8) VectorNav_Log(i,5:7)];

% Convert to YPR
[IMU_Yaw, IMU_Pitch, IMU_Roll] = quat2angle(IMU_Q, 'ZYX');

% IMU Bias 
IMU_Yaw   = IMU_Yaw   + deg2rad(IMU_YawBias);
IMU_Roll  = IMU_Roll  + deg2rad(IMU_RollBias);
IMU_Pitch = IMU_Pitch + deg2rad(IMU_PitchBias);


% NOTE: Matlab quaternion functions use a NED coord system, so the usual 
% rotations will produce negitive results, that's why they are negated
% here and the X and Y axis are swapped above.
IMU_Yaw   =  -1 * IMU_Yaw;
IMU_Roll  =  -1 * IMU_Roll;
IMU_Pitch =  -1 * IMU_Pitch;

% Save Coordinate inversions
IMU_Q = angle2quat(IMU_Pitch, IMU_Roll, IMU_Yaw, 'XYZ');


% Convert from LLA to Flat Earth (Metric) coodinates
if size(GPS_Lattitude,1) > 0 
    GPS_MetricPose = lla2flat([GPS_Lattitude GPS_Longitude GPS_Altitude], ... 
                              [GPS_Lattitude(1) GPS_Longitude(1)],  0, -GPS_Altitude(1));

    % Flat earth uses a (Northing, Easting, - Altitude) format, 
    % so X and Y need to be swapped 
    GPS_MetricPose = [GPS_MetricPose(:,2) GPS_MetricPose(:,1) GPS_MetricPose(:,3)];
    GPS_MetricPose(:,3) = GPS_MetricPose(:,3);
else
    % When no GPS signal is available, default to zero for the whole
    % dataset so that the rest of the code doesn't fault on an empty matrix
    GPS_Lattitude  = [0 0 0; 0 0 0];
    GPS_Longitude  = [0 0 0; 0 0 0];
    GPS_Altitude   = [0 0 0; 0 0 0];
    GPS_MetricPose = [0 0 0; 0 0 0];
    GPS_Timestamp  = [IMU_Timestamp(1);  IMU_Timestamp(end)];
end


% Interpolate the GPS pose for each IMU orientation (metric)
IMU_MetricPose = interp1(GPS_Timestamp, GPS_MetricPose, IMU_Timestamp);


% Cleanup workspace
clear i
