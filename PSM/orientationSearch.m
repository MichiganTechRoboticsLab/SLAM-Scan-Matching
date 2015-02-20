function dth = orientationSearch(ref, newR, newBad)
    %% constants
    global PM_SEARCH_WINDOW

    global PM_DFI
    %%
    LARGE_NUMBER = 10000;
    n = 0;
    k=1;
    e = 0;
    dth = 0;
    err = zeros(size(newR,1),1);
    err(:) = LARGE_NUMBER;
    beta = zeros(size(newR,1),1);

    for di = -PM_SEARCH_WINDOW:1:PM_SEARCH_WINDOW
        n = 0;
        e = 0;
        minI = 0; maxI=0;
        if di <= 0
            minI = -di + 1;
            maxI = min(size(ref.data,1),size(newR,1));
        else
            minI = 0 + 1;
            maxI = min(size(ref.data,1),size(newR,1)) - di;
        end
        
        nI = minI:maxI;
        rI = nI + di;
        delta = abs(newR(nI) - ref.data(rI,2));
        delta(newBad(nI) | ref.bad(rI)) = [];
        if(length(delta) > 0)
            err(k) = mean(delta);
        else
            err(k) = LARGE_NUMBER
        end
        beta(k) = di;
        k = k + 1;
    end

    [~, imin] = min(err);
    dth = beta(imin)*PM_DFI;

    if(imin >= 2 && imin < (k))
        D = err(imin -1) + err(imin+1) -2 * err(imin);
        d = LARGE_NUMBER;
        if (abs(D) > .01 && err(imin-1) > err(imin) && err(imin+1) > err(imin))
            d = (err(imin-1) - err(imin+1)) / (D/2);
        end
        if abs(d) < 1
            dth = dth + d*PM_DFI;
        end
    end
end
