% Copyright 2011. All rights reserved.
% Institute of Measurement and Control Systems
% Karlsruhe Institute of Technology, Germany

% This file is part of libicp.
% Authors: Andreas Geiger

% libicp is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% libicp is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA 

% libicp demo file for MATLAB

dbstop error; clear variables; close all;
disp('================================');

% create model
[X,Y] = meshgrid(-2:.25:2,-2:.25:2);
Z = 5 * X .* exp(-X.^2-Y.^2);
M = [X(:) Y(:) Z(:)]';

% transform model yielding the template
rx = 0.3;
Tr = [1 0 0 -2; 0 cos(rx) -sin(rx) 1;0 sin(rx) cos(rx) 0;0 0 0 1];
T  = Tr(1:3,1:3)*M + Tr(1:3,4)*ones(1,size(M,2));

% fit template to model
% - init with identity transformation (eye(4))
% - no outlier rejection step (-1)
% - use point-to-plane fitting
Tr_fit = icpMex(M,T,eye(4),-1,'point_to_plane');
T_fit  = Tr_fit(1:3,1:3)*T + Tr_fit(1:3,4)*ones(1,size(T,2));

% plot
figure,axis equal,hold on; ms=8; lw=2; fs=16;
plot3(M(1,:),M(2,:),M(3,:),'or','MarkerSize',ms,'LineWidth',lw);
plot3(T(1,:),T(2,:),T(3,:),'sg','MarkerSize',ms,'LineWidth',lw);
plot3(T_fit(1,:),T_fit(2,:),T_fit(3,:),'xb','MarkerSize',ms,'LineWidth',lw);
legend('model','template','fitted template','Location','NorthEast');
set(gca,'FontSize',fs);
view(20,20);
