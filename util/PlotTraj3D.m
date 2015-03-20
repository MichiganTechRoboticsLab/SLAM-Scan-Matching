function PlotTraj3D(P, Q, AxisVectorLength)
%PLOTTRAJ3D  Plot a Trajectory in 3D with rotations
%   Dereck Wonnacott (c) 2014

% Inspired by http://www.mathworks.com/matlabcentral/fileexchange/24589-kinematics-toolbox/content/kinematics/screws/drawframe.m

    % Check for missing inputs
    if ~exist('Q', 'var') 
        Q = zeros(size(x, 4));
        Q(:,1) = 1;
    end
    if ~exist('AxisVectorLength', 'var') 
        AxisVectorLength = 1;
    end


    % Check array lengths
    if size(P, 1) ~= size(Q, 1)
        error('Input Array Length Mismatch')
    end
    
    % Axis Vectors
    AxisVectors = eye(3) * AxisVectorLength;

    % Rotate
    Ax = quatrotate(Q, AxisVectors(1,:));
    Ay = quatrotate(Q, AxisVectors(2,:));
    Az = quatrotate(Q, AxisVectors(3,:));
    
    % Translate
    Ax = P + Ax;
    Ay = P + Ay;
    Az = P + Az;
    
    
    % Save the hold state so we can restore it later if needed
    holdVal = ishold;
    
    % Plot em
    plot3(P( :,1), P( :,2), P( :,3), 'k');   
    hold on    
    plot3(Ax(:,1), Ax(:,2), Ax(:,3), 'r'); 
    plot3(Ay(:,1), Ay(:,2), Ay(:,3), 'g');
    plot3(Az(:,1), Az(:,2), Az(:,3), 'b');
    
    % Plot Initial Axis Markers
    line([P(1,1) Ax(1,1)], [P(1,2) Ax(1,2)], [P(1,3) Ax(1,3)], 'Color', 'r', 'LineWidth', 2);
    line([P(1,1) Ay(1,1)], [P(1,2) Ay(1,2)], [P(1,3) Ay(1,3)], 'Color', 'g', 'LineWidth', 2);
    line([P(1,1) Az(1,1)], [P(1,2) Az(1,2)], [P(1,3) Az(1,3)], 'Color', 'b', 'LineWidth', 2);
    
    % Plot Final Axis Markers
    line([P(end,1) Ax(end,1)], [P(end,2) Ax(end,2)], [P(end,3) Ax(end,3)], 'Color', 'r', 'LineWidth', 2);
    line([P(end,1) Ay(end,1)], [P(end,2) Ay(end,2)], [P(end,3) Ay(end,3)], 'Color', 'g', 'LineWidth', 2);
    line([P(end,1) Az(end,1)], [P(end,2) Az(end,2)], [P(end,3) Az(end,3)], 'Color', 'b', 'LineWidth', 2);
    
    % Restore the hold value
    if ~holdVal
        hold off
    end
end