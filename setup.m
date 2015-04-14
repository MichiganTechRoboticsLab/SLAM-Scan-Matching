 clear

% Select a scan matching algorithm:
% 0: GICP          (Doens't Work)
% 1: Olsen
% 2: PSM           (Doens't Work)
% 3: Hill-Climbing
% 4: LIBICP
% 5: ICP1
% 6: Chamfer

algo = 6;


% Set up paths
wd = pwd;
addpath( wd, [wd '/GICP'], ...
             [wd '/Olson'], ...
             [wd '/PSM'], ...
             [wd '/hill'], ...
             [wd '/ICP'], ...
             [wd '/ICP/libicp/matlab'], ...
             [wd '/ICP/icp1'], ...
             [wd '/chamfer'], ...
             [wd '/util']);

% Load sensor dataset
%load('../datasets/hallroomvn.mat')
%load('testData/testWorldPSM.mat')
%load('testData/testworld.mat')

DataPath = '../datasets/';


%DatasetName = 'hall_and_room_w_vn';       
%DatasetName = 'eerc_dillman_dow';         
%DatasetName = 'campus1';                   
%DatasetName = 'eerc_dow_dill_inout';  
      
% DatasetName = '2000-01-31-19-01-35';  % EERC817 Multiple passes
% DatasetName = '2000-01-31-19-05-32';  % EERC8F
% DatasetName = '2000-01-31-19-10-14';  % EERC1F (bad)
% DatasetName = '2000-01-31-19-13-51';  % EERC1F small loop   
% DatasetName = '2000-01-31-19-15-35';  % EERC1F large loop
% DatasetName = '2000-01-31-19-21-26';  % EERC_DOW_DIL inout (bad) [1-32000], 
% DatasetName = '2000-01-31-19-47-37';  %  EERC Outside front no motion
% DatasetName = '2000-01-31-19-50-23';  %  Campus {1.5, 16k, 24k, 43k, [], 76k, 90k} 
% DatasetName = '2000-01-31-20-30-59';  % EERC8f elevator to ieee, 817
%DatasetName = '2000-01-31-20-34-36';  % garbage
         
% DatasetName = 'EERC8f handheld';  % EERC 8f IEEE, Lap, stairwells. (good)
%DatasetName = 'EERC_DOW_DIL inout3';  %  handheld {1, 3k, 11k, 24k }



% Load dataset from log files
VectorNav_Logfile = [DataPath DatasetName '/vn.csv'];
Hokuyo_Logfile = [DataPath DatasetName '/lidar_data.csv'];  

ReadHokuyoLog
ReadVectorNavLog


% % Simulated data for PSM
 useSimWorld = false;
% 
% if (~exist('init','var') || init == false)
%     if (exist('useSimWorld','var') && useSimWorld == true)
%         Lidar_Ranges = Test_Lidar_Ranges;
%         Lidar_Ranges = Lidar_Ranges;% + normrnd(0, .001, size(Test_Lidar_Ranges,1),1);
% 
%         Lidar_ScanIndex = Test_Lidar_ScanIndex;
%         Lidar_Angles = Test_Lidar_Angles;
%         [Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);
%     else
%         useSimWorld = false;
%         % Grab range measurements from Lidar data (First Echo)
%         I = 3:6:6484;
%         Lidar_Ranges = Lidar_Log(:, I);
% 
%         % System Timestamp
%         Lidar_Timestamp_System = Lidar_Log(:, 1);
% 
%         % Sensor Timestamp
%         Lidar_Timestamp_Sensor = Lidar_Log(:, 2) / 1000;
% 
%         % Combined Timestamp
%         Lidar_Timestamp = Lidar_Timestamp_System(1) - Lidar_Timestamp_Sensor(1) + Lidar_Timestamp_Sensor;
% 
%         % Scan Index
%         Lidar_ScanIndex = (1:Lidar_ScanCount)';
% 
%         % Angles for each measurement
%         % Rotated the lidar data to face the Y-Axis
%         da = (Lidar_AngleEnd - Lidar_AngleStart) / (Lidar_nPoints - 1);
%         Lidar_Angles = (Lidar_AngleStart:da:Lidar_AngleEnd)' + pi/2;
% 
% 
%         % Copy each col vector across for each measurement
%         Lidar_Timestamp = repmat(Lidar_Timestamp, 1, Lidar_nPoints);
%         Lidar_ScanIndex = repmat(Lidar_ScanIndex, 1, Lidar_nPoints);
%         Lidar_Angles    = repmat(Lidar_Angles', Lidar_ScanCount, 1);
% 
%         % Reformat the data so there is one measurement per row
%         Lidar_Ranges = reshape(Lidar_Ranges', [], 1);
%         Lidar_Timestamp = reshape(Lidar_Timestamp', [], 1);
%         Lidar_ScanIndex = reshape(Lidar_ScanIndex', [], 1);
%         Lidar_Angles = reshape(Lidar_Angles', [], 1);
% 
% 
%         % Convert from mm to meters
%         Lidar_Ranges = Lidar_Ranges / 1000;
%         switch algo
%             case 2
%                 fprintf('ScanMatcher: Data is ready for PSM\n')
%             otherwise
%                 % Remove invalid range data (Too close or too far)
%                 I = Lidar_Ranges < 0.75 | Lidar_Ranges > 30;
%                 Lidar_ScanIndex(I) = [];
%                 Lidar_Timestamp(I) = [];
%                 Lidar_Angles(I) = [];
%                 Lidar_Ranges(I) = [];
%         end
%         % Convert to Cartisian Coordinates
%         % (Rotated data so that the Lidar faces the y-Axis)
%         [Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);
%     end
%     init = true;
% end
