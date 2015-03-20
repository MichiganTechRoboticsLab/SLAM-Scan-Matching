%% MainFunction
function [ offset, varargout ] = psm( offset, scan, ref, varargin )
    %% setupParser
    p = inputParser;
    p.addParameter('PM_STOP_COND', .0004, @(x)isnumeric(x));
    p.addParameter('PM_MAX_ITER', 60, @(x)isnumeric(x));
    p.addParameter('PM_MAX_RANGE', 30, @(x)isnumeric(x));
    p.addParameter('PM_MIN_RANGE', .1, @(x)isnumeric(x));
    p.addParameter('PM_LASER_Y', 0, @(x)isnumeric(x));
    p.addParameter('PM_FOV', 270, @(x)isnumeric(x));
    p.addParameter('PM_L_POINTS', 1081, @(x)isnumeric(x));
    p.addParameter('PM_WEIGHTING_FACTOR', .70*.70, @(x)isnumeric(x));
    p.addParameter('PM_REDUCED_WEIGHTING_FACTOR', .10*.10, @(x)isnumeric(x));
    p.addParameter('PM_SEG_MAX_DIST', .2, @(x)isnumeric(x));
    p.addParameter('PM_CHANGE_WEIGHT_ITER', 10, @(x)isnumeric(x));
    p.addParameter('PM_MAX_ERR', .3, @(x)isnumeric(x));
    p.addParameter('PM_SEARCH_WINDOW', 200, @(x)isnumeric(x));
    p.addParameter('PM_MEDIAN_WINDOW', 5, @(x)isnumeric(x));
    p.addParameter('VERBOSE', false, @(x)islogical(x));
    
    p.parse(varargin{:})
    
    
    
    %% CONSTANTS
    global PM_STOP_COND;
    global PM_MAX_ITER;
    global PM_MAX_RANGE
    global PM_MIN_RANGE
    global PM_LASER_Y
    global PM_FOV
    global PM_L_POINTS
    global PM_WEIGHTING_FACTOR
    global PM_REDUCED_WEIGHTING_FACTOR
    global PM_SEG_MAX_DIST
    global PM_CHANGE_WEIGHT_ITER
    global PM_MAX_ERR
    global PM_SEARCH_WINDOW
    global PM_MEDIAN_WINDOW
    
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
    global VERBOSE
    
    
    global figs
    
    %% SETUP CONSTANTS
    ROLL_WINDOW_SIZE            = 5;
    PM_STOP_COND                = p.Results.PM_STOP_COND;
    PM_MAX_ITER                 = p.Results.PM_MAX_ITER;
    PM_MAX_RANGE                = p.Results.PM_MAX_RANGE;
    PM_MIN_RANGE                = p.Results.PM_MIN_RANGE;
    PM_LASER_Y                  = p.Results.PM_LASER_Y;
    PM_FOV                      = p.Results.PM_FOV;
    PM_L_POINTS                 = p.Results.PM_L_POINTS;
    PM_WEIGHTING_FACTOR         = p.Results.PM_WEIGHTING_FACTOR;
    PM_REDUCED_WEIGHTING_FACTOR = p.Results.PM_REDUCED_WEIGHTING_FACTOR;
    PM_SEG_MAX_DIST             = p.Results.PM_SEG_MAX_DIST;
    PM_CHANGE_WEIGHT_ITER       = p.Results.PM_CHANGE_WEIGHT_ITER;
    PM_MAX_ERR                  = p.Results.PM_MAX_ERR;
    PM_SEARCH_WINDOW            = p.Results.PM_SEARCH_WINDOW;
    PM_MEDIAN_WINDOW            = p.Results.PM_MEDIAN_WINDOW;
    VERBOSE                     = p.Results.VERBOSE;
    
    PM_RANGE                 = 1;
    PM_MOVING                = 2;
    PM_MIXED                 = 3;
    PM_OCCLUDED              = 4;
    PM_EMPTY                 = 5;
    
    PM_FI_MIN                = pi/2 - deg2rad(PM_FOV) / 2;
    PM_FI_MAX                = pi/2 + deg2rad(PM_FOV) / 2;
    PM_DFI                   = deg2rad(PM_FOV) / (PM_L_POINTS);
    
    PM_FI                    = ((0:PM_L_POINTS-1) * PM_DFI + PM_FI_MIN)';
    PM_SI                    = sin(PM_FI);
    PM_CO                    = cos(PM_FI);
    
    %% PSM Vars
    C = PM_WEIGHTING_FACTOR;
    
    %% Setup varargout
    nout = max(nargout,1) - 1;
    plotCount = 0;
    if nout >= 3
        axs = zeros(1,PM_MAX_ITER);
        
    end
    
    if nout >= 4
        ays = zeros(1,PM_MAX_ITER);
    end
    
    if nout >= 5
        aths = zeros(1,PM_MAX_ITER);
    end
    
    if nout >= 6
        errs = zeros(1,PM_MAX_ITER);
    end
    
    if nout >= 7
        dxs = zeros(1,PM_MAX_ITER);
        plotCount = plotCount + 1;
    end
    
    if nout >= 8
        dys = zeros(1,PM_MAX_ITER);
        plotCount = plotCount + 1;
    end
    
    if nout >= 9
        dths = zeros(1,PM_MAX_ITER);
        plotCount = plotCount + 1;
    end
    
    if nout >= 10
        stoperr = zeros(1,PM_MAX_ITER);
        plotCount = plotCount + 1;
    end
    %% PSM
    if VERBOSE
        fprintf('PSM: Initial Guess: ')
        tmp = offset;
        tmp(3) = rad2deg(tmp(3));
        fprintf(['[ ' repmat('%g ', 1, size(tmp, 2)-1) '%g ]\n'], tmp')
    end
    
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
    
    t13 = sin( rth - ath) * PM_LASER_Y + cos(rth) * ax + sin(rth)*ay - sin(rth) * ry -rx * cos(rth);
    t23 = cos( rth - ath) * PM_LASER_Y - sin(rth) * ax + cos(rth)*ay - cos(rth) * ry -rx * sin(rth) - PM_LASER_Y;
    
    refS.rx = 0; refS.ry = 0; refS.th = 0;
    act.rx = t13; act.ry = t23; act.th = ath - rth;
    
    ax = act.rx; ay = act.ry; ath = act.th;
    
    iter = 0;
    smallCorrErr = 0;
    dx =0; dy = 0; dth=0; avg_err = 0;
    
    while( iter < PM_MAX_ITER && smallCorrErr < 3)
        
        
        if exist('stoperr', 'var')
            stoperr(iter+1) = abs(dx)*100 + abs(dy)*100 + abs(rad2deg(dth));
        end
        
        if( (abs(dx)*100 + abs(dy)*100 + abs(rad2deg(dth))) < PM_STOP_COND)
            smallCorrErr = smallCorrErr + 1;
        else
            smallCorrErr = 0;
        end
        
        if (iter == PM_CHANGE_WEIGHT_ITER)
            C = PM_REDUCED_WEIGHTING_FACTOR;
        end
        
        if exist('axs', 'var')  && VERBOSE
            axs(iter+1) = ax;
        end
        
        if exist('ays', 'var')  && VERBOSE
            ays(iter+1) = ay;
        end
        
        if exist('aths', 'var') && VERBOSE
            aths(iter+1) = ath;
        end
        
        act.rx = ax;
        act.ry = ay;
        act.th = ath;
        [act, newR, newBad ] = projectScan(act);
        
        if VERBOSE
            tmps = act;
            tmps.data(:,2) = newR;
            tmps.bad = newBad;
            [tmps.x, tmps.y ] = pol2cart(tmps.data(:,1), tmps.data(:,2));
            plotSegments(tmps,refS);
        end
        
        if ( mod(iter, 2) == 0)
            dth = orientationSearch(refS, newR, newBad, C);
            if exist('dxs', 'var')
                dxs(iter+1) = abs(dx);
            end
            
            if exist('dys', 'var')
                dys(iter+1) = abs(dy);
            end
            
            if exist('dths', 'var')
                dths(iter+1) = abs(dth);
            end
            if exist('errs', 'var')
                errs(iter+1) = avg_err;
            end
            ath = ath + dth;
            iter = iter+1;
            
            if ~VERBOSE
                fprintf('PSM: %d %f %f %f\n',iter, ax, ay, rad2deg(ath));
            end
            continue;
        end
        
        
        
        
        [avg_err, dx, dy] = translationEstimation(refS, newR, newBad, C);
        if exist('dxs', 'var')
            dxs(iter+1) = abs(dx);
        end
        
        if exist('dys', 'var')
            dys(iter+1) = abs(dy);
        end
        
        if exist('dths', 'var')
            dths(iter+1) = abs(dth);
        end
        if exist('errs', 'var')
            errs(iter+1) = avg_err;
        end
        iter = iter+1;
        ax = ax + dx;
        ay = ay + dy;
        
        if ~VERBOSE
            fprintf('PSM: %d %f %f %f\n',iter, ax, ay, rad2deg(ath));
        end
        if VERBOSE    
            change_current_figure(3);
            iters = max(1,iter-ROLL_WINDOW_SIZE):iter;
            
            subplot(4,1,1);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, dxs(iters), 'o-');
            title('Evolution of X');
            axis([iters(1),max(2,iters(end)),0,1.2*max(abs(dxs(iters)))])
            
            subplot(4,1,2);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, dys(iters), 's-');
            title('Evolution of Y');
            axis([iters(1),max(2,iters(end)),0,1.2*max(abs(dys(iters)))])
            
            subplot(4,1,3);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, rad2deg(dths(iters)),'x-');
            title('Evolution of Ө');
            axis([iters(1),max(2,iters(end)),0,1.2*max(abs(rad2deg(dths(iters))))+.1])
            
            subplot(4,1,4);
            curticks = get(gca, 'XTick');
            set( gca, 'XTickLabel', cellstr( num2str(curticks(:), '%5f') ) );
            plot(iters, stoperr(iters),'.-');
            title('Evolution of Err');
            axis([iters(1),max(2,iters(end)),0,1.2*max(abs(stoperr(iters)))+.1])
            
            drawnow
        end
    end
    
    offset = [ax, ay, ath];
    
    if nout >= 1
        varargout{1} = iter;
    end
    
    if nout >= 2
       varargout{2} = avg_err;
    end
    
    if nout >= 3
        varargout{3} = axs;
    end
    
    if nout >= 4
        varargout{4} = ays;
    end
    
    if nout >= 5
        varargout{5} = aths;
    end
    
    if nout >= 6
        varargout{6} = errs;
    end
    
    if nout >= 7
        varargout{7} = dxs;
        plotCount = plotCount + 1;
    end
    
    if nout >= 8
        varargout{8} = dys;
        plotCount = plotCount + 1;
    end
    
    if nout >= 9
        varargout{9} = dths;
        plotCount = plotCount + 1;
    end
    
    if nout >= 10
        varargout{10} = stoperr;
        plotCount = plotCount + 1;
    end
    
    %% cleanup
    clearvars -global PM*
end
