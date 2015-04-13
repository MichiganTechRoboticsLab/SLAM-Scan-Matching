function mattest( jobID, taskID )
%MATTEST tst harness for superior/ivs clusters
%   Detailed explanation goes here


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

    SLAMbootstrap

end

