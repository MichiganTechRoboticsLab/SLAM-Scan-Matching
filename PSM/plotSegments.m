function plotSegments(ls)
    global figs
    ls.seg(ls.seg == 0) = [];
    nSegs = unique(ls.seg);
    CM = jet(size(nSegs,1)+1);
    change_current_figure(figs(1))
    clf
    hold on
    plot(ls.x, ls.y, 'Color', CM(1,:), 'marker', '.', 'linestyle', 'none');
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
        plot(ls.x(I), ls.y(I), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
    end
    hold off
    axis equal
    change_current_figure(figs(2))
    clf
    hold on
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
        plot(ls.data(I,1), ls.data(I,2), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
    end
    hold off
    axis equal
end
