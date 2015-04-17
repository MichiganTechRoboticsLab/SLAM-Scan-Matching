function mattest( jobID, taskID )
%MATTEST test harness for cluster execution
%   This script is called by each node in the cluster
%   Based on the taskID, each node is assigned a dataset.
%   Results are saved to unique files for each dataset


    % Select a scan matching algorithm:
    % 0: GICP          (Doens't Work)
    % 1: Olsen
    % 2: PSM           (Doens't Work)
    % 3: Hill-Climbing
    % 4: LIBICP
    % 5: ICP1
    % 6: Chamfer

    algo = 4;


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

    % Output Path
    OutPath = ['../' num2str(jobID) '/' num2str(taskID, '%3u') '-'];
    mkdir(OutPath);

    % Keep a copy of the settings with the results
    pause(taskID);
    if ~exist([OutPath 'SLAM.m'], 'file')
      copyfile('SLAM.m'   , OutPath)
      copyfile('mattest.m', OutPath)
    end

    %Default dataset ROI
    start         = 1;     % Scan Index
    stop          = 100000; % Scan Index

    % Load sensor dataset
    %load('testData/testWorldPSM.mat')
    %load('testData/testworld.mat')

    DataPath = '../datasets/';

    switch taskID
        case 1
            DatasetName = 'hall_and_room_w_vn';
        case 2
            DatasetName = 'eerc_dillman_dow';  
            %stop        = 20000;
        case 3
            DatasetName = 'campus1'; 
            %stop        = 25000;
        case 4
            DatasetName = 'eerc_dow_dill_inout';  % Bad timestamp early in dataset {20}
            start       = 20;   
        case 5
            DatasetName = '2000-01-31-19-05-32';  % EERC8F
        case 6
            DatasetName = '2000-01-31-19-21-26';  % EERC_DOW_DIL inout  [1-32000],
        case 7
            DatasetName = '2000-01-31-19-50-23';  % Campus {1.5, 16k, 24k, 43k, [], 76k, 90k} 
        case 8
            DatasetName = '2000-01-31-20-30-59';  % EERC8f elevator to ieee, 817
        case 9
            DatasetName = 'EERC8f handheld';      % EERC 8f IEEE, Lab, stairwells. (good)
            start       = 500;   
        case 10
            DatasetName = 'EERC_DOW_DIL inout3';  % Handheld {EERC8F 1, EERC 1F 3k, 11k, 24k }
            start       = 3000;   
    end


    % Load dataset from log files
    VectorNav_Logfile = [DataPath DatasetName '/vn.csv'];
    Hokuyo_Logfile = [DataPath DatasetName '/lidar_data.csv'];  

    ReadHokuyoLog
    ReadVectorNavLog

    % Simulated data for PSM
    useSimWorld = false;

    SLAM

    % Remove debug plot when finished
    if exist([ OutPath DatasetName '-dbg.pdf' ], 'file')
      delete[ OutPath DatasetName '-dbg.pdf' ]
    end
    
    % Save result
    %save([ OutPath DatasetName '.mat'], '-v7.3');
end

