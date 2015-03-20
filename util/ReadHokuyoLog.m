% ReadHokuyoLog.m
% 
% Call the correct read script based on file format 



if ~exist('Lidar_LogFormat', 'var')
    Lidar_LogFormat = 2;
end

% Use the proper file format reader
switch Lidar_LogFormat
    case 1
        ReadHokuyoLog_1
    case 2
        ReadHokuyoLog_2
end 
