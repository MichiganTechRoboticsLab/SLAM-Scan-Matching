function dth = orientationSearch(ref, newR, newBad, C)
    %% constants
    global PM_SEARCH_WINDOW
    global PM_MAX_ERR
    global PM_L_POINTS
    global PM_MAX_RANGE;
    global PM_DFI
    %%
    LARGE_NUMBER = PM_MAX_RANGE + 10;
    n = 0;
    k=1;
    e = 0;
    dth = 0;
    beta = -PM_SEARCH_WINDOW:1:PM_SEARCH_WINDOW;
    err = zeros(size(beta,2),1);
    err(:) = LARGE_NUMBER;
    
    for di = beta
        if di <= 0
            minI = -di+1;
            maxI = PM_L_POINTS;
        else
            minI = 1;
            maxI = PM_L_POINTS - di;
        end
        
        nI = minI:(maxI);
        rI = nI + di;
        delta = abs(newR(nI) - ref.data(rI,2));
        I = ~(newBad(nI) == 0 & ref.bad(rI) == 0 & delta < PM_MAX_ERR );
        delta(I) = [];
        if(~isempty(delta))
            
            err(k) = mean(delta);
        else
            err(k) = LARGE_NUMBER;
        end
        
        change_current_figure(7);
        cla
        hold on
        plot(nI,newR(nI),'r.');
        plot(nI,ref.data(rI,2),'g.');
        title(['dI: ' num2str(di) ' Angle: ' num2str(rad2deg(di*PM_DFI))]);
        hold off
        
        change_current_figure(6);
        cla
        plot(rad2deg(beta(1:k)*PM_DFI),err(1:k),'.r-')
        drawnow;
        k = k + 1;
    end
    
    [~, imin] = min(err);
    imin = imin;
    global figs
    change_current_figure(6);
    cla
    hold on
    plot(rad2deg(beta*PM_DFI),err,'.r-')
    dth = ((beta(imin)))*PM_DFI;
    
    
    m = 0;
    if(imin >= 2 && imin < (k-1))
        xs = [-1;0;1];
        C = polyfit(xs, err(xs+imin),2);
        m = roots(polyder(C));
        newxs = min(xs):.1:max(xs);
        newbetas = beta(min(xs+imin)):.1:beta(max(xs+imin));
        newerr = polyval(C,newxs);
        plot(rad2deg(newbetas*PM_DFI),newerr,'g-');
        plot(rad2deg(dth+m*PM_DFI),polyval(C,m),'g.');
        m = roots(polyder(C));
        if( polyval(C,m) > err(imin))
            m = 0;
        end
    end
    
    hold off
    dth = dth+m*PM_DFI;
    %     if(imin >= 2 && imin < (k))
    %         D = err(imin -1) + err(imin+1) -2 * err(imin);
    %         d = LARGE_NUMBER;
    %         if (abs(D) > .01 && err(imin-1) > err(imin) && err(imin+1) > err(imin))
    %             d = (err(imin-1) - err(imin+1)) / D / 2;
    %         end
    %         if abs(d) < 1
    %             dth = dth + d*PM_DFI;
    %         end
    %     end
end
