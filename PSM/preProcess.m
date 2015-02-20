function [scan] = preProcess(scan)
    %% constants
    global PM_MAX_RANGE

    global PM_RANGE
    %%
    scan.data(:,2) = medfilt1(scan.data(:,2), 5);
    I = scan.data(:,2) >= PM_MAX_RANGE;
    scan.bad(I) = bitset(scan.bad(I), PM_RANGE);
    scan = segmentScan(scan);
end
