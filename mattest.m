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
            DatasetName = 'eerc_dow_dill_inout';      
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

