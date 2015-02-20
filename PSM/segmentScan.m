function [scan] = segmentScan(scan)
    %% constants
    global PM_SEG_MAX_DIST
    %%
    dr = 0;
    segCnt = 1;
    cnt = 0;
    breakSeg = false;
    
    if( abs( scan.data(1,2) - scan.data(2,2)) < PM_SEG_MAX_DIST )
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
%         scan.bad(i)
        if( scan.bad(i) ~= 0 )
            breakSeg = true;
            scan.seg(i) = 0;
        else
            dr = scan.data(i,2) - (2*scan.data(i-1,2) - scan.data(i-2,2));
            if( abs(scan.data(i,2) - scan.data(i-1,2)) < PM_SEG_MAX_DIST || ( scan.seg(i-1) == scan.seg(i-2) && abs(dr) < PM_SEG_MAX_DIST))
                cnt = cnt + 1;
                scan.seg(i) = segCnt;
            else
                breakSeg = true;
            end
        end
        
        if( breakSeg )
            if(cnt == 1)
                dr = scan.data(i,2) - (2*scan.data(i-1,2) - scan.data(i-2,2));
                if( scan.seg(i-2) == 0 && scan.bad(i) == 0 && scan.bad(i-1) == 0 && scan.bad(i-2) == 0 && abs(dr) < PM_SEG_MAX_DIST)
                    scan.seg(i) = segCnt;
                    scan.seg(i-1) = segCnt;
                    scan.seg(i-2) = segCnt;
                    cnt = 3;
                else
                    scan.seg(i-1) = 0;
                    scan.seg(i) = segCnt;
                    cnt = 1;
                end
            else
                segCnt = segCnt + 1;
                scan.seg(i) = segCnt;
                cnt = 1;
            end
        end
        
    end
end
