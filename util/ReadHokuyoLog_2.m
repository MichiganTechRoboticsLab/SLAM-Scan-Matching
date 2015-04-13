% ReadHokuyoLog.m
%  Reads the Hokuyo sensor log and sets some variables pertaining to the 
%  sensor such as start end angle. Because the log file can take a long time 
%  to read, it only reads the file once per session. If you change data sets 
%  or want to force a reload you must delete the Lidar_Log variable from the 
%  workspace. This feature has saved hours. :)  The result of the file will 
%  be a vector of points, not a matrix. This is because the file automatically 
%  drops all out of range points before passing the data back to the workspace. 
%  This cut 5 million points from the dataset, saving a lot of time. There are 
%  quite a few examples on how to index through the dataset one scan at a time 
%  if needed.  For each point there is a column vector with a timestamp, 
%  angle, range, X, Y, and Scan index, 


% Check that the File Exists
if ~exist(Hokuyo_Logfile, 'file')
    error('Hokuyo Logfile not found')
end

% Check if this file has already been parsed
matfile = strrep(Hokuyo_Logfile, '.csv', '.mat');
if exist(matfile, 'file')
    disp('INFO: Lidar data already parsed, importing from .mat file.')
    load(matfile)
else
    % Sensor Parameters
    Lidar_AngleStart = -2.3562;
    Lidar_AngleEnd   =  2.3562;

    % Read the log file
    Lidar_Log = load(Hokuyo_Logfile);

    % Number of points per scan
    Lidar_nPoints = 1081;
    Lidar_nEchos  = 3;
    Lidar_nIntensities  = 3;

    % Number of scans
    Lidar_ScanCount = size(Lidar_Log, 1);
   
    % Save parsed data to speed up load next time
    save(matfile, 'Lidar_*', '-v7.3');
end


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


% Remove invalid range data (Too close or too far)
% I = Lidar_Ranges < 0.75 | Lidar_Ranges > 30;
% Lidar_ScanIndex(I) = [];
% Lidar_Timestamp(I) = [];
% Lidar_Angles(I) = [];
% Lidar_Ranges(I) = [];


% Convert to Cartisian Coordinates
% (Rotated data so that the Lidar faces the y-Axis)
[Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);


% Cleanup workspace
clearvars matfile I da 





