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

% make file for building mex MATLAB wrappers

mex('icpMex.cpp','../src/icp.cpp','../src/icpPointToPoint.cpp',...
    '../src/icpPointToPlane.cpp','../src/kdtree.cpp',...
    '../src/matrix.cpp','-I../src');
mex('sparsifyMex.cpp','../src/kdtree.cpp','-I../src');
disp('done!');
