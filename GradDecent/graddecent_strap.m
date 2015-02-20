%function T = betterOlson(map, scan, pixSrch, moveRange, lidarStd, pixRadius, lidarRange)

clear
clc
close all

%number of pixels to radiate out
pixSrch = 5;

%movement range (how much do you think the scans will move between each
%other maximum) in meters
moveRange = 10;

%standard deviation of points
lidarStd = 0.05;

%pixel radius size in meters (box radius)
pixRadius = 0.5;

lidarRange = 30;

%N = size(map,1);
N = 1081;
r = rand(N,1)*lidarRange;
theta = rand(N,1)*2*pi;

%to generate our scan we creat ean offset and normalize the difference
offset = [3.3 -1.1 deg2rad(1)];
map = [r.*cos(theta) r.*sin(theta)];

R = [cos(offset(3)), -sin(offset(3)); sin(offset(3)) cos(offset(3))];

%offset and simulate the error with the hokuyo
scan = map * R;
scan = scan + repmat(offset(1:2),N,1) + (rand(N,2) * 0.1 - 0.05);
fprintf('calculating grad decent...\n');

T = gradDecentMatcher( [0 0 0], scan, map, 0.3 * 4, 0.3, 0.05);

T(1)
T(2)
rad2deg(T(3))