%% MainFunction
function [ offset, iter, avg_err ] = psm( offset, scan, ref, varargin )
    %% setupParser
    p = inputParser;



    p.parse(varargin{:})

    %% CONSTANTS
    global PM_STOP_COND;
    global PM_MAX_ITER;
    global PM_MAX_RANGE
    global PM_MIN_RANGE
    global LASER_Y
    global PM_FOV
    global PM_L_POINTS
    global PM_WEIGHTING_FACTOR
    global PM_SEG_MAX_DIST
    global PM_CHANGE_WEIGHT_ITER
    global PM_MAX_ERR
    global PM_SEARCH_WINDOW

    global PM_RANGE
    global PM_MOVING
    global PM_MIXED
    global PM_OCCLUDED
    global PM_EMPTY

    global PM_FI_MIN
    global PM_FI_MAX
    global PM_DFI

    global PM_FI
    global PM_SI
    global PM_CO

    %% SETUP CONSTANTS
    PM_STOP_COND             = .0004;
    PM_MAX_ITER              = 60;
    PM_MAX_RANGE             = 30;
    PM_MIN_RANGE             = .1;
    LASER_Y                  = 0;
    PM_FOV                   = 270;
    PM_L_POINTS              = 1081;
    PM_WEIGHTING_FACTOR      = 30*30;
    PM_SEG_MAX_DIST          = .2;
    PM_CHANGE_WEIGHT_ITER    = 10;
    PM_MAX_ERR               = .3;
    PM_SEARCH_WINDOW         = 200;

    PM_RANGE                 = 1;
    PM_MOVING                = 2;
    PM_MIXED                 = 3;
    PM_OCCLUDED              = 4;
    PM_EMPTY                 = 5;

    PM_FI_MIN                = pi/2 - deg2rad(PM_FOV) / 2;
    PM_FI_MAX                = pi/2 + deg2rad(PM_FOV) / 2;
    PM_DFI                   = deg2rad(PM_FOV) / (PM_L_POINTS - 1);

    PM_FI                    = ((0:PM_L_POINTS-1) * PM_DFI + PM_FI_MIN)';
    PM_SI                    = sin(PM_FI);
    PM_CO                    = cos(PM_FI);

    %% PSM Vars
    C = PM_WEIGHTING_FACTOR;

    %% PSM
    lsr = struct;
    lsr.rx = 0;
    lsr.ry = 0;
    lsr.th = 0;
    lsr.data = ref;
    [lsr.x, lsr.y] = pol2cart(lsr.data(:,1), lsr.data(:,2));
    lsr.bad = zeros(size(ref,1),1);
    lsr.seg = zeros(size(ref,1),1);

    lsn = struct;
    lsn.rx = offset(1);
    lsn.ry = offset(2);
    lsn.th = offset(3);
    lsn.data = scan;
    [lsn.x, lsn.y] = pol2cart(lsn.data(:,1), lsn.data(:,2));
    lsn.bad = zeros(size(scan,1),1);
    lsn.seg = zeros(size(scan,1),1);


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
        iter = iter+1;
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

        [avg_err, dx, dy] = translationEstimation(refS, newR, newBad, C);
        ax = ax + dx;
        ay = ay + dy;

    end

    offset = [ax, ay, ath];
    %% cleanup
    clearvars -global PM*
end
