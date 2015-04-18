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
    OutPath = ['../' num2str(jobID) '/'  ];

    % Keep a copy of the settings with the results
    if taskID == 1
      mkdir(OutPath);
      copyfile('SLAM.m'   , OutPath)
      copyfile('mattest.m', OutPath)
    end

    % Add task prefix to each file
    OutPath = [OutPath num2str(taskID, '%3u') '-'];

    %Default dataset ROI
    start         = 1;     % Scan Index
    stop          = 100000; % Scan Index

    % Load sensor dataset
    %load('testData/testWorldPSM.mat')
    %load('testData/testworld.mat')

    DataPath = '../datasets/';

    switch taskID
        case 1
            DatasetName = 'eerc_dillman_dow';  
            %stop        = 20000;
        case 2
            DatasetName = 'campus1'; 
            start       = 200;   
            %stop       = 25000;
        case 3
            DatasetName = 'eerc_dow_dill_inout';
            % Before the Hill
            start       = 20;
            stop        = 19000;
        case 4
            DatasetName = 'eerc_dow_dill_inout';
            % After the Hill
            start       = 22500;
            stop        = 38000;   
        case 5
            % EERC_DOW_DIL inout  [1-32000],  
            DatasetName = '2000-01-31-19-21-26';  
        case 6 
            % Night with Natalie #1 (EERC 8F)
            DatasetName = '2015-04-17-00-44-23';  
            stop        = 14500;
        case 7
            % Night with Natalie #1 (EERC DOW DIL)
            DatasetName = '2015-04-17-00-44-23';  
            start       = 15200;           
        case 8
            % Night with Natalie #2 (EERC 8F)
            DatasetName = '2015-04-17-01-30-48';  
            start       = 50;              
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
      delete([ OutPath DatasetName '-dbg.pdf' ]);
    end
    
    % Save result
    %save([ OutPath DatasetName '.mat'], '-v7.3');
end

