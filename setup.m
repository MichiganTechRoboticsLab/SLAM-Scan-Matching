clear

% Load sensor dataset
load('../datasets/hallroomvn.mat')
%load('testData/testworld.mat')

% Set up paths
wd = pwd;
addpath( wd, [wd '/GICP'], [wd '/Olson'], [wd '/PSM'], [wd '/hill'], [wd '/util'])

% Set up default function
% 0: GICP
% 1: Olsen
% 2: PSM 
% 3: Hill-Climbing

algo = 3;

