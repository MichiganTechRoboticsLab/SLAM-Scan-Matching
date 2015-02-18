clear

% Load sensor dataset
load('../datasets/hallroomvn.mat')

% Set up paths
wd = pwd;
addpath( wd, [wd '/GICP'], [wd '/Olson'], [wd '/PSM'], [wd '/util'])

% Set up default function
% GICP = 0
% Olsen = 1
% PSM = 2

algo = 1;

