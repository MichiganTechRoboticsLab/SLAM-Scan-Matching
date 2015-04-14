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

    % Output Path
    OutPath = ['../' num2str(jobID) '/'];
    mkdir(OutPath);

    % Load sensor dataset
    %load('testData/testWorldPSM.mat')
    %load('testData/testworld.mat')

    DataPath = '../datasets/';

    switch taskID
        case 1
            DatasetName = 'hall_and_room_w_vn';
        case 2
            DatasetName = 'eerc_dillman_dow';
        case 3
            DatasetName = 'campus1';  
        case 4
            DatasetName = 'eerc_dow_dill_inout';  % Bad timestamp early in dataset {20}
        case 5
            DatasetName = '2000-01-31-19-01-35';  % EERC817 Multiple passes
        case 6
            DatasetName = '2000-01-31-19-05-32';  % EERC8F
        case 7
            DatasetName = '2000-01-31-19-10-14';  % EERC1F (bad)
        case 8
            DatasetName = '2000-01-31-19-13-51';  % EERC1F small loop 
        case 9
            DatasetName = '2000-01-31-19-15-35';  % EERC1F large loop
        case 10
            DatasetName = '2000-01-31-19-21-26';  % EERC_DOW_DIL inout (bad) [1-32000],
        case 11
<<<<<<< HEAD
            DatasetName = '2000-01-31-19-50-23';  %  Campus {1.5, 16k, 24k, 43k, [], 76k, 90k} 
        case 12
            DatasetName = '2000-01-31-20-30-59';  % EERC8f elevator to ieee, 817
        case 13
            DatasetName = 'EERC8f handheld';  % EERC 8f IEEE, Lap, stairwells. (good)
        case 14
=======
            DatasetName = '2000-01-31-19-47-37';  %  EERC Outside front no motion
        case 12
            DatasetName = '2000-01-31-19-50-23';  %  Campus {1.5, 16k, 24k, 43k, [], 76k, 90k} 
        case 13
            DatasetName = '2000-01-31-20-30-59';  % EERC8f elevator to ieee, 817
        case 14
            DatasetName = 'EERC8f handheld';  % EERC 8f IEEE, Lap, stairwells. (good)
    end


    % Load dataset from log files
    VectorNav_Logfile = [DataPath DatasetName '/vn.csv'];
    Hokuyo_Logfile = [DataPath DatasetName '/lidar_data.csv'];  

    ReadHokuyoLog
    ReadVectorNavLog

    % Simulated data for PSM
    useSimWorld = false;

    SLAM

    save([ '../' DatasetName '.mat']);
end

