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
end

% Read the Lidar Data from HDD only when nessisary
if exist('Lidar_Log', 'var')
    disp('INFO: Lidar data already loaded, skipping import from HDD.')
else
    % Sensor Parameters
    Lidar_AngleStart = -2.3562;
    Lidar_AngleEnd   =  2.3562;

    % Column Indexes
    Lidar_Col_Timestamp = 1;
    Lidar_Col_Ranges    = 2;

    % Resulting Logfile
    Lidar_Log = [];

    % Open the file
    fid = fopen(Hokuyo_Logfile);

    % State vars
    nLine = 0;
    nScan = 0;
    nPoint = 0;
    nPoints = [];

    %Read each line
    tLine = fgets(fid);
    while ischar(tLine)
        nLine = nLine + 1;

        if strcmp(tLine(1:2), '-1')
            % Beginning of a new scan
            [items, ~] = sscanf(tLine, '%d,%d,%d');

            % Store the number of ranges for this scan
            if nScan > 0 
                nPoints(nScan) = nPoint;
                nPoint = 0;
            end

            % Start a new scan by storing the timestamp on a new row
            nScan = nScan + 1;
            Lidar_Log(nScan, Lidar_Col_Timestamp) = items(2) + items(3) * 10E-7;

        else
            % New data point
            [items, ~] = sscanf(tLine, '%d,%f');

            % Store this measurement (Convert from mm to m)
            Lidar_Log(nScan, Lidar_Col_Ranges + nPoint) = items(1) * 1e-3;
            nPoint = nPoint + 1;
        end

        % Read next line
        tLine = fgets(fid);

        % Show progress
        if mod(nLine, 10000) == 0
            disp(nLine)
        end
    end

    fclose(fid);

    % Store number of points per scan
    Lidar_nPoints = nPoints(1);

    % Remove incomplete scans
    I = nPoints ~= Lidar_nPoints;
    Lidar_Log(I, :) = [];    

    % Remove blank columns
    if size(Lidar_Log, 2) > Lidar_Col_Ranges + Lidar_nPoints
        Lidar_Log(:, Lidar_Col_Ranges + Lidar_nPoints:end) = [];
    end
    
    % Record number of scans read
    Lidar_ScanCount = nScan;
   
    % Save parsed data to speed up load next time
    save(matfile, 'Lidar_*');
end

% Grab all range measurements from Lidar data
r = Lidar_Log(:, Lidar_Col_Ranges:end);
Lidar_Ranges = reshape(r', [], 1);

% Generate column for a Timestamp for every point
t1 = (1:size( r, 1))';
t2 = repmat(t1', size(r,2), 1);
Lidar_ScanIndex = reshape(t2, [], 1);

% Generate column for a Timestamp for every point
t1 = Lidar_Log(:, Lidar_Col_Timestamp);
t2 = repmat(t1', size(r,2), 1);
Lidar_Timestamp = reshape(t2, [], 1);

% Make a second column of all of the angles for each measurement
% Rotated the lidar data to face the Y-Axis
da = (Lidar_AngleEnd - Lidar_AngleStart) / (Lidar_nPoints - 1);
a = (Lidar_AngleStart:da:Lidar_AngleEnd)' + pi/2;
Lidar_Angles = repmat(a, size(r,1), 1);


% Remove invalid range data (Too close or too far)
I = or(Lidar_Ranges >= 25, Lidar_Ranges <= 0.8);
Lidar_ScanIndex(I) = [];
Lidar_Timestamp(I) = [];
Lidar_Angles(I) = [];
Lidar_Ranges(I) = [];


% Convert to Cartisian Coordinates
% (Rotated data so that the Lidar faces the y-Axis)
[Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);


% Cleanup workspace
clearvars fid nLine nScan nPoint tLine items nPoints I da
clearvars r R a A t1 t2 T t I





