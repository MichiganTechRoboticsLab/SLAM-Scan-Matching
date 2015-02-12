function [ offset, numIterations, lastCost ] = gicp( offset, new_data, ref, varargin )
    %UNTITLED Summary of this function goes here
    %   Detailed explantion goes here
    
    p = inputParser;
    p.addParameter('costThresh', .004, @(x)isnumeric(x));
    p.addParameter('minMatchDist', 2.0, @(x)isnumeric(x));
    p.addParameter('plotIter', false, @(x)islogical(x));
    p.addParameter('MSEThreshold',  1e-6, @(x)isnumeric(x));

    p.parse(varargin{:})
    
    costThresh = p.Results.costThresh;
    minMatchDist = p.Results.minMatchDist;
    plotIter = p.Results.plotIter;
    msethresh = p.Results.MSEThreshold;
    
    new_data_c = precomputeCovariance(new_data);
    ref_c = precomputeCovariance(ref);
    
    numIterations = 0;
    lastCost = 1e100;
    
    while true
        a_trans = zeros(size(new_data,1),2);
        a_trans_c = [];
        for i=1:size(new_data,1)
            [p, c] = dispPoint(new_data(i,:), new_data_c(:,:,i), offset);
            a_trans(i,:) = p;
            a_trans_c = cat(3, a_trans_c, c);
        end
        
        
        match_pairs = {};
        for i = 1:(size(a_trans,1))
            a_p = a_trans(i,:);
            [bInd, minDist] = findClosestPointInB(ref,a_p, [0 0 0]);
            if minDist <= minMatchDist
                Ca = a_trans_c(:,:,i);
                Cb = ref_c(:,:,bInd);
                b_p = ref(bInd,:);
                match_pairs{length(match_pairs) + 1} = { new_data(i,:), b_p, Ca, Cb};
            end
        end
        
        costFunc = @(x) cost_func(x, match_pairs);
        
%       [newOffset, newCost] = fminsearch(costFunc, offset , struct('Display', 'final', 'TolFun', msethresh*100, 'TolX',0.1));
%       [newOffset, newCost] = fminsearch(costFunc, offset , struct('Display', 'final', 'TolFun', msethresh*100));
       [ newOffset, newCost ] = fminsearch(costFunc, offset);

        options = optimset('MaxIter', 20);
%        [ newOffset, newCost ] = fminsearch(costFunc, offset, options);


        if abs( lastCost - newCost) < costThresh
            offset = newOffset;
            lastCost = newCost;
            break
        end
        
        
        % save the current offset and cost
        offset = newOffset;
        lastCost = newCost;
        numIterations = numIterations + 1;
        
        if plotIter
            draw(a_trans, ref)
        end
    end
    
end

function [Covariances] = precomputeCovariance(points, varargin)
    
    p = inputParser;
    p.addParameter('high_var', 1, @(x)isnumeric(x));
    p.addParameter('low_var', .001, @(x)isnumeric(x));
    p.parse(varargin{:})
    
    high_var = p.Results.high_var;
    low_var = p.Results.low_var;
    
    Covariances = [];
    
    for i = 1:size(points,1)
        C = [high_var 0; 0 high_var];
        
        normVec = findLocalNormal(points(1,:), points);
        C = computeVectorCovariance(normVec, low_var, high_var);
        Covariances = cat(3, Covariances, C);
    end
    
end

function [ normVec ] = findLocalNormal( pnt, points)
    % Change me if needed
    NUMBER_OF_POINTS = 10;
    pnt_list = zeros(NUMBER_OF_POINTS,2);
    
    pnt_count = 0;
    pnts_copy = points;
    while pnt_count < NUMBER_OF_POINTS && ~isempty(pnts_copy)
        [ pInd, dist ] = dsearchn(pnts_copy, pnt);
        pnt_list(pnt_count + 1,:) = pnts_copy(pInd,:);
        pnts_copy(pInd,:) = [];
        pnt_count = pnt_count + 1;
    end
    
    loadings = pca(pnt_list);
    
    normVec = loadings(2,:);
    
end

function [ Ca ] = computeVectorCovariance( vec, x_var, y_var)
    
    Cv = [x_var 0; 0 y_var];
    normVec = vec / norm(vec);
    
    if normVec(2) == 0
        R = eye(2);
    else
        B = -1 / (normVec(2) + normVec(1)^2/normVec(2));
        A = -normVec(1)*B/normVec(2);
        R = [A -B; B A];
    end
    
    Ca = R' * Cv * R;
end

function [new_point] = dispOffset(p, offset)
    dx = offset(1);
    dy = offset(2);
    theta = offset(3);
    
    T = [ cos(theta) -sin(theta) dx;
        sin(theta)  cos(theta) dy;
        0           0  1];
    p_hom = [p 1];
    temp = T*p_hom';
    new_point = temp(1:2)';
end

function [new_point, new_c] = dispPoint(p, c, offset)
    dx = offset(1);
    dy = offset(2);
    theta = offset(3);
    
    T = [ cos(theta) -sin(theta) dx;
        sin(theta)  cos(theta) dy;
        0           0  1];
    p_hom = [p 1];
    temp = T*p_hom';
    new_point = temp(1:2)';
    
    R = T(1:2,1:2);
    
    new_c = R * c * R';
end

function [ minInd, minDist ] = findClosestPointInB( b_data, a, offset)
    a_off = dispOffset(a, offset);
    [ minInd, minDist] = dsearchn(b_data, a_off);
end

function s = cost_func(offset, match_pairs)
%     for i = 1:length(match_pairs)
%         pair = match_pairs{i};
%         sum = sum + computeMatchError(offset, pair{1}, pair{2}, pair{3}, pair{4});
%     end
    s = sum(cellfun(@(x) computeMatchError(offset, x{1}, x{2}, x{3}, x{4}), match_pairs));
    
end

function errVal = computeMatchError(offset, a, b, Ca, Cb)
    dx = offset(1);
    dy = offset(2);
    theta = offset(3);
    
    T = [ cos(theta) -sin(theta) dx;
        sin(theta)  cos(theta) dy;
        0           0  1];
      
    R = T(1:2,1:2);
    
    Cv = Ca;
    
    temp = [a 1]*T';
    d_vec = [b 1]-temp;
    
    d_vec(3) = 1;
    
    %d_vec = displace( [a 1], [b 1], T);
    
    res = Cb + R*Cv*R';
    
    invMat = zeros(3);
    
    invMat(1:2,1:2) = inv(res);
    
    errVal = d_vec * invMat * d_vec';
end

function result = displace(ai, bi, T)
    temp = ai*T';
    result = bi-temp;
    
    result(3) = 1;
end

function draw(a_pnts, b_pnts)
    plot(a_pnts(:,1), a_pnts(:,2),'r.')
    hold on
    plot(b_pnts(:,1), b_pnts(:,2),'g.')
    hold off
end