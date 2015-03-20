 clear

% Select a scan matching algorithm:
% 0: GICP          (Doens't Work)
% 1: Olsen
% 2: PSM           (Doens't Work)
% 3: Hill-Climbing
% 4: LIBICP
% 5: ICP1
% 6: Chamfer

algo = 2;



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

% Set up default function
% 0: GICP          (Doens't Work)
% 1: Olsen
% 2: PSM           (Doens't Work)
% 3: Hill-Climbing
% 4: LIBICP
% 5: ICP1
% 6: Chamfer

algo = 2;
useSimWorld = true;


% Load sensor dataset
%load('../datasets/hallroomvn.mat')
% load('testData/testWorldPSM.mat')
load('testData/testworld.mat')

DataPath = '../datasets/';
DatasetName = 'hall_and_room_w_vn';

% Load dataset from log files
VectorNav_Logfile = [DataPath DatasetName '/vn.csv'];
Hokuyo_Logfile = [DataPath DatasetName '/lidar_data.csv'];

if (~exist('init','var') || init == false)
    if (exist('useSimWorld','var') && useSimWorld == true)
        Lidar_Ranges = Test_Lidar_Ranges;
        Lidar_Ranges = Lidar_Ranges;% + normrnd(0, .001, size(Test_Lidar_Ranges,1),1);

        Lidar_ScanIndex = Test_Lidar_ScanIndex;
        Lidar_Angles = Test_Lidar_Angles;
        [Lidar_X, Lidar_Y] = pol2cart(Lidar_Angles, Lidar_Ranges);
    else
        useSimWorld = false;
        ReadHokuyoLog
        ReadVectorNavLog
    end
    init = true;
end
