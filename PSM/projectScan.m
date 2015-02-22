function [scan, newR, newBad] = projectScan(scan)
    %% constants
    global PM_L_POINTS
    global PM_RANGE
    global PM_MOVING
    global PM_MIXED
    global PM_OCCLUDED
    global PM_EMPTY

    global PM_FI_MIN
    global PM_DFI
    global PM_FI
    %%
    newR = zeros(size(scan.data,1),1);
    newR(:) = 10000;
    newBad = zeros(size(scan.data,1),1);
    newBad = bitset(newBad, PM_EMPTY);

    % MAP TO REF FRAME
    delta = scan.data(:,1) + scan.th;
    x = scan.data(:,2) .* cos(delta) + scan.rx;
    y = scan.data(:,2) .* sin(delta) + scan.ry;
    [fi, r] = cart2pol(x, y);
    fi(x < 0 & y < 0) = fi(x < 0 & y < 0) + 2*pi;

    % INTERPOLATION
    for i = 2:size(newR,1)
        if(scan.seg(i) ~= 0 && scan.seg(i) == scan.seg(i-1) && scan.bad(i) == 0 && scan.bad(i-1) == 0 ) % && fi(i)>PM_FI_MIN && fi(i-1) > PM_FI_MIN
            occluded = false;
            j0 = 0; j1 = 0; r0 = 0; r1 = 0; a0 = 0; a1 = 0;
            if(fi(i) - fi(i-1) >= pi)
                continue;
            end
            if( fi(i) > fi(i-1) )
                occluded = false;
                a0 = fi(i-1);
                a1 = fi(i);
                j0 = ceil( (fi(i-1) - PM_FI_MIN) / PM_DFI) + 1;
                j1 = floor( (fi(i) - PM_FI_MIN) / PM_DFI) + 1;
                r0 = r(i-1);
                r1 = r(i);
            else
                occluded = true;
                a0 = fi(i);
                a1 = fi(i-1);
                j0 = ceil( (fi(i) - PM_FI_MIN) / PM_DFI) + 1;
                j1 = floor( (fi(i-1) - PM_FI_MIN) / PM_DFI) + 1;
                r0 = r(i);
                r1 = r(i-1);
            end

            while(j0 <= j1)
                ri = (r1 - r0) / (a1 - a0) * ( ( j0*PM_DFI+PM_FI_MIN ) - a0) + r0;
                if( j0 >= 1 && j0 <= PM_L_POINTS && newR(j0) > ri)
                    newR(j0) = ri;
                    newBad(j0) = bitset(newBad(j0),PM_EMPTY,0 );
                    newBad(j0) = bitset(newBad(j0),PM_OCCLUDED, occluded );

                end
                j0 = j0 + 1;
            end
        end
    end
end
