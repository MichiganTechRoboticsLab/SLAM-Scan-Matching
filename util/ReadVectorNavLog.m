% ReadVectorNavLog.m
%  Reads the vectornav log file into matlab and makes variables for each data type.  
%  Generates a single timestamp value from the two log file columns, converts the 
%  LLA to metric positions, and converts the IMU's RPY  valued into quaternions. 
%  It also filters out all zero LLA values from the gps data. So the IMU and GPS
%  data arrays may not be the same length!


% Check that a log file was specified
if ~exist(VectorNav_Logfile, 'file')
    error('VectorNav Logfile not found')
end


% Read the log file 
matfile = strrep(VectorNav_Logfile, '.csv', '.mat');
if exist(matfile, 'file')
    disp('INFO: VectorNav data already parsed, importing from .mat file.')
    load(matfile)
else
    VectorNav_Log = load(VectorNav_Logfile);
    
    % Save parsed data to speed up load next time
    save(matfile, 'VectorNav_*');
end


% Default Parameters
if ~exist('VectorNav_ROI_Start', 'var')
    VectorNav_ROI_Start = 1;
end
if ~exist('VectorNav_ROI_End', 'var')
    VectorNav_ROI_End  = size(VectorNav_Log, 1);
end
if ~exist('IMU_YawBias', 'var')
    IMU_YawBias = 0;
end
if ~exist('IMU_RollBias', 'var')
    IMU_RollBias = 0;
end
if ~exist('VectorNav_LogFormat', 'var')
    VectorNav_LogFormat = 2;
end

% ROI
VectorNav_Log = VectorNav_Log(VectorNav_ROI_Start:VectorNav_ROI_End, :);

% Use the proper file format reader
switch VectorNav_LogFormat
    case 1
        ReadVectorNavLog_1
    case 2
        ReadVectorNavLog_2
end 

