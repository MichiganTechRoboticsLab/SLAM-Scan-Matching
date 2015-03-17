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
M(1,:) = 0:pi/20:3*pi;
M(2,:) = sin(M(1,:));

% transform model yielding the template
rx = 0.5;
Tr = [cos(rx) -sin(rx) -1; sin(rx) cos(rx) -2;0 0 1];
T  = Tr(1:2,1:2)*M + Tr(1:2,3)*ones(1,size(M,2));

% fit template to model
% - init with identity transformation (eye(3))
% - no outlier rejection step (-1)
% - use point-to-plane fitting
Tr_fit = icpMex(M,T,eye(3),-1,'point_to_plane');
T_fit  = Tr_fit(1:2,1:2)*T + Tr_fit(1:2,3)*ones(1,size(T,2));

% plot
figure,axis equal,hold on; ms=8; lw=2; fs=16;
plot(M(1,:),M(2,:),'or','MarkerSize',ms,'LineWidth',lw);
plot(T(1,:),T(2,:),'sg','MarkerSize',ms,'LineWidth',lw);
plot(T_fit(1,:),T_fit(2,:),'xb','MarkerSize',ms,'LineWidth',lw);
legend('model','template','fitted template','Location','NorthWest');
set(gca,'FontSize',fs);
