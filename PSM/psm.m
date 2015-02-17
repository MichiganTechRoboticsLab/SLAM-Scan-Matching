function [ offset, iter, avg_err ] = psm( offset, new_data, ref, varargin )

    p = inputParser;
    
    
    
    p.parse(varargin{:})
    
    %% CONSTANNTS
    PM_STOP_COND = .004;
    PM_MAX_ITER = 30;
    PM_MAX_RANGE = 30;
    PM_MIN_RANGE = .1;
    LASER_Y = 0;
    PM_FOV = 270;
    PM_L_POINTS = 1081
    PM_WEIGHTING_FACTOR = 70*70;
    PM_CHANGE_WEIGHT_ITER = 10;
    PM_MAX_ERR = 1
    
    PM_RANGE = 1;
    PM_MOVING = 2;
    PM_MIXED = 3;
    PM_OCCLUDED = 4;
    PM_EMPTY = 5;
    
    PM_FI_MIN = pi / 2 - deg2rad(PM_FOV) / 2;
    PM_FI_MAX = pi / 2 + deg2rad(PM_FOV) / 2;
    PM_DFI = deg2rad(PM_FOV) / (PM_L_POINTS + 1);
    
    
    
    %% PSM Vars
    C = PM_WEIGHTING_FACTOR;
    
    %% PSM
    [ refP(:,1) refP(:,2) ] = cart2pol(ref(:,1), ref(:,2));
    [ newP(:,1) newP(:,2) ] = cart2pol(new_data(:,1), new_data(:,2));
    
    lsr = struct;
    lsn = struct;
    
    lsr.rx = 0;
    lsr.ry = 0;
    lsr.th = 0;
    lsr.data = refP;
    lsr.bad = zeros(size(refP,1),1);
    lsr.seg = zeros(size(refP,1),1);
    
    lsn.rx = offset(1);
    lsn.ry = offset(2);
    lsn.th = offset(3);
    lsn.data = newP;
    lsn.bad = zeros(size(newP,1),1);
    lsn.seg = zeros(size(newP,1),1);
    

    lsr = preProcess(lsr);
    lsn = preProcess(lsn);
    
    refS = lsr;
    act = lsn;
    
    rx = refS.rx; ry = refS.ry; rth = refS.th;
    ax = act.rx; ay = act.ry; ath = act.th;
    
    t13 = sin( rth - ath) * LASER_Y + cos(rth) * ax + sin(rth)*ay - sin(rth) * ry -rx * cos(rth);
    t23 = cos( rth - ath) * LASER_Y - sin(rth) * ax + cos(rth)*ay - cos(rth) * ry -rx * sin(rth) - LASER_Y;
    
    refS.rx = 0; refS.ry = 0; refS.th = 0; 
    refS.rx = t13; refS.ry = t23; refS.th = ath - rth; 
    
    ax = act.rx; ay = act.ry; ath = act.th;
    
    iter = 0;
    smallCorrErr = 0;
    dx =0; dy = 0; dth=0;
    while( iter < PM_MAX_ITER && smallCorrErr < 3)
        if( (abs(dx) + abs(dy) + abs(dth)) < PM_STOP_COND)
            smallCorrErr = smallCorrErr + 1;
        else
            smallCorrErr = 0;
        end
        
        act.rx = ax;
        act.ry = ay;
        act.th = ath;
        
        [act, newR, newBad ] = projectScan(act);
        
        if ( mod(iter, 2))
            dth = orientationSearch(refS, newR, newBad);
            ath = ath + dth;
            continue;
        end
        
        if (iter == PM_CHANGE_WEIGHT_ITER)
            C = C/50;
        end
        
        [avg_err, dx, dy] = translationEstimation(ref, newR, newBad, C);
        ax = ax + dx;
        ay = ay + dy;
        
    end
    
    offset = [ax, ay, ath];
    
    
    
    
    
    
    
    
    %% Helper Functions
    function [avg_err, dx, dy] = translationEstimation(ref, newR, newBad, C)
        hi1 = 0; hi2 = 0; hwi1 = 0; hwi2 = 0; hw1= 0; hw2 = 0; hwh11 = 0;
        hwh12 = 0; hwh21 = 0; hwh22 = 0; w = 0;
        dr = 0;
        abs_err = 0;
        n = 0;
        for i = 1:size(ref.data,1)
            dr = ref.data(i,2) - newR(i);
            abs_err = abs_err + abs(dr);
            if( ~ref.bad(i) && ~newBad(i) && newR(i) < PM_MAX_RANGE && newR(i) > PM_MIN_RANGE && abs(dr) < PM_MAX_ERR)
                w = C/ (dr*dr + C);
                n = n + 1;
                
                hi1 = cos(ref.data(i,1);
                hi2 = sin(ref.data(i,1);
                
                hwi1 = hi1*w;
                hwi2 = hi2*w;
                
                hw1 = hw1 + hwi1*dr;
                hw2 = hw2 + hwi2*dr;
                
                hwh11 = hwh11 + hwi1*hi1;
                hwh12 = hwh12 + hwi2*hi2;
                
            end
        end
    end
    
    function dth = orientationSearch(ref, newR, newBad)
        LARGE_NUMBER = 10000;
        n = 0;
        k=1;
        e = 0;
        dth = 0;
        err = zeros(size(newR,1),1);
        beta = zeros(size(newR,1),1);
        
        for di = -PM_SEARCH_WINDOW:1:PM_SEARCH_WINDOW
            n = 0;
            e = 0;
            minI = 0; maxI=0;
            if di <= 0
                minI = -di + 1;
                maxI = size(newR,1);
            else
                minI = 0 + 1;
                maxI = size(newR,1) - di;
            end
            
            for i= minI:maxI
                delta = abs(newR(i)-ref.data(i+di,2));
                if(~newBad(i) && ~ref.bad(i+di))
                    e = e + delta;
                    n = n+1;
                end
            end
            if(n > 0)
                err(k) = e / n;
            else
                err(k) = LARGE_NUMBER;
            end
            beta(k) = di;
            k = k + 1;
        end
        
        [emin, imin] = min(err)
        dth = beta(imin)*PM_DFI;
        
        if(imin >= 2 && imin < (k-1))
            D = err(imin -1) + err(imin+1) -2 * err(imin);
            d = LARGE_NUMBER;
            if (abs(D) > .01 && err(imin-1) > err(imin) && err(imin+1) > err(imin))
                d = (err(imin-1) - err(imin_1)) / (D/2);
            end
            if abs(d) < 1
                dth = dth + d*PM_DFI;
            end
        end
    end
    
    
    function [scan, newR, newBad] = projectScan(scan)
        newR = zeros(size(scan.data,1),1);
        newR(:) = 10000;
        newBad = zeros(size(scan.data,1),1);
        newBad = bitset(newBad, PM_EMPTY);
        
        % MAP TO REF FRAME
        delta = scan.data(:,1) + scan.th;
        x = scan.data(:,2) * cos(delta) + scan.rx;
        y = scan.data(:,2) * sin(delta) + scan.ry;
        [r fi] = cart2pol(x, y);
        fi(x < 0 & y < 0) = fi(x < 0 & y < 0) + 2*pi;
        
        % INTERPOLATION
        for i = 2:size(newR,1)
            if(scan.seg(i) ~= 0 && scan.seg(i) == scan.seg(i-1) && ~scan.bad(i) && ~scan.bad(i-1) ) % && fi(i)>PM_FI_MIN && fi(i-1) > PM_FI_MIN
                occluded = false;
                j0 = 0; j1 = 0; r0 = 0; r1 = 0; a0 = 0; a1 = 0;
                if(fi(i) - fi(i-1) >= pi)
                    continue;
                end
                if( fi(i) < fi(i-1) )
                    occluded = false;
                    a0 = fi(i-1);
                    a1 = fi(i);
                    j0 = ceil( (fi(i-1) - PM_FI_MIN) / PM_DFI);
                    j1 = floor( (fi(i) - PM_FI_MIN) / PM_DFI);
                    r0 = r(i-1);
                    r1 = r(i);
                else
                    occluded = true;
                    a0 = fi(i);
                    a1 = fi(i-1);
                    j0 = ceil( (fi(i) - PM_FI_MIN) / PM_DFI);
                    j1 = floor( (fi(i-1) - PM_FI_MIN) / PM_DFI);
                    r0 = r(i);
                    r1 = r(i-1);
                end
                
                while(j0 <= j1)
                    ri = (r1 - r0) / (a1 -a0) * ( ( j0*PM_DFI+PM_FI_MIN ) - a0) + r0;
                    if( j0 >= 0 && j0 < size(scan.data,1) && newR(j0) > ri)
                        newR(j0) = ri;
                        newBad(j0) = bitset(newBad(j0),PM_EMPTY,0 );
                        newBad(j0) = bitset(newBad(j0),PM_OCCLUDED, occluded );
                        
                    end
                    j0 = j0 + 1;
                end
            end
        end
    end
    
    function [scan] = preProcess(scan)
        scan.data(:,2) = medfilt1(scan.data(:,2), 5);
        I = scan.data(:,2) >= PM_MAX_RANGE;
        scan.bad(I) = bitset(scan.bad(I), PM_RANGE);
        scan = segmentScan(scan);
    end
    
    function [scan] = segmentScan(scan)
        dr = 0;
        segCnt = 1;
        cnt = 0
        breakSeg = false;
        
        if( abs( scan.data(1,2) - ls.data(2,2)) < SEG_MAX_DIST )
            scan.seg(1) = segCnt;
            scan.seg(2) = segCnt;
            cnt = 2;
        else
            scan.seg(1) = 0;
            scan.seg(2) = segCnt;
            cnt = 1;
        end
        
        for i = 3:size(scan.data,1)
            breakSeg = false;
           
            if( scan.bad(i) )
                breakSeg = true;
                scan.seg(i) = 0;
            else
                dr = scan.data(i,2) - (2*scan.data(i-1,2) - scan.data(i-2,2));
                if( abs(scan.data(i,2) - scan.data(i-1,2)) < SEG_MAX_DIST || ( scan.seg(i-1) == scan.seg(i-2) && abs(dr) < SEG_MAX_DIST))
                    cnt = cnt + 1;
                    scan.seg(i) = segCnt;
                else
                    breakSeg = true;
                end
            end
            
            if( breakSeg )
                if(cnt == 1)
                    dr = scan.data(i,2) - (2*scan.data(i-1,2) - scan.data(i-2,2));
                    if( scan.seg(i-2) == 0 && scan.bad(i) == 0 && scan.bad(i-1) == 0 && scan.bad(i-2) == 0 && abs(dr) < SEG_MAX_DIST)
                        scan.seg(i) = segCnt;
                        scan.seg(i-1) = segCnt;
                        scan.seg(i-2) = segCnt;
                        cnt = 3;
                    else
                        scan.seg(i-1) = 0;
                        scan.seg = segCnt;
                        cnt = 1;
                    end
                else
                    segCnt = segCnt + 1;
                    scan.seg(i) = segCnt;
                    cnt = 1;`
                end
            end
        end
        
    end
    
end