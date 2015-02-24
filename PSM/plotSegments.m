function plotSegments(ls)
   
    global figs
    %     ls.seg(ls.seg == 0) = [];
    empty = bitset(0,5);
    emptyI = ls.bad == empty;
    ls.x(emptyI) = [];
    ls.y(emptyI) = [];
    ls.data(emptyI,:) = [];
    ls.bad(emptyI) = [];
    ls.seg(emptyI) = [];
    badI = ls.bad ~= 0;
    
    nSegs = unique(ls.seg);
    CM = jet(size(nSegs,1)+1);
    change_current_figure(figs(4))
    cla reset
    hold on
    axis equal 
    
    plot(ls.rx, ls.ry, 'r.');
    
    plot(ls.x, ls.y, 'Color', CM(1,:), 'marker', '.', 'linestyle', 'none');
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
        legendTitles = cellstr(num2str(nSegs(1:i), 'Segment %d'));
        legend(legendTitles);
        plot(ls.x(I), ls.y(I), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
        plot(ls.x(I & badI), ls.y(I & badI), 'Color', CM(i+1,:), 'marker', 'x', 'linestyle', 'none');
    end
    hold off
    
    
    change_current_figure(figs(5))
    cla reset
    hold on
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
        plot(ls.data(I,1), ls.data(I,2), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
        plot(ls.data(I & badI,1), ls.data(I & badI,2), 'Color', CM(i+1,:), 'marker', 'x', 'linestyle', 'none');
    end
    hold off
    axis equal 
    legend(legendTitles);
    drawnow;
end
