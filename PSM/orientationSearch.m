function dth = orientationSearch(ref, newR, newBad)
    %% constants
    global PM_SEARCH_WINDOW
    global PM_MAX_ERR
    
    global PM_DFI
    %%
    LARGE_NUMBER = 10000;
    n = 0;
    k=1;
    e = 0;
    dth = 0;
    beta = -PM_SEARCH_WINDOW:1:PM_SEARCH_WINDOW;
    err = zeros(size(beta,2),1);
    err(:) = LARGE_NUMBER;
    for di = beta
        n = 0;
        e = 0;
        minI = 0; maxI=0;
        if di <= 0
            minI = -di+1;
            maxI = min(size(ref.data,1),size(newR,1));
        else
            minI = 1;
            maxI = min(size(ref.data,1),size(newR,1)) - di;
        end
        
        nI = minI:maxI;
        rI = nI + di;
        delta = abs(newR(nI) - ref.data(rI,2));
        delta(~(newBad(nI) == 0 & ref.bad(rI) == 0 )) = [];
        if(~isempty(delta))
            err(k) = mean(delta);
        else
            err(k) = LARGE_NUMBER;
        end
        k = k + 1;
    end
    
    [~, imin] = min(err);
    global figs
    change_current_figure(6);
    cla
    hold on
    plot(-3:3,err(imin-3:imin+3),'.r-')
    dth = (beta(imin))*PM_DFI;
    xs = [-1;0;1];
    C = polyfit(xs, err(xs+imin),2);
    newxs = min(xs):.1:max(xs);
    newerr = polyval(C,newxs);
    plot(newxs,newerr,'g-');
    
    m = 0;
    if(imin >= 2 && imin < (k))
        xs = [-1;0;1];
        C = polyfit(xs, err(xs+imin),2);
        m = roots(polyder(C));
        newxs = min(xs):.1:max(xs);
        newerr = polyval(C,newxs);
        plot(newxs,newerr,'g-');
        plot(m,polyval(C,m),'g.');
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
