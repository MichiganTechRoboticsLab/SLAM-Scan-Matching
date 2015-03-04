function plotSegments(ls, ref)
    narginchk(1, 2);
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
    change_current_figure(4)
    cla reset
    hold on
    
    
    plot(ls.rx, ls.ry, 'r.');
    
    plot(ls.x, ls.y, 'Color', CM(1,:), 'marker', '.', 'linestyle', 'none');
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
        legendTitles = cellstr(num2str(nSegs(1:i), 'Segment %d'));
        legend(legendTitles);
        plot(ls.x(I), ls.y(I), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
        plot(ls.x(I & badI), ls.y(I & badI), 'Color', CM(i+1,:), 'marker', 'x', 'linestyle', 'none');
    end
    axis equal tight
    hold off
    
    
    change_current_figure(5)
    cla reset
    hold on
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
        plot(ls.data(I,1), ls.data(I,2), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
        plot(ls.data(I & badI,1), ls.data(I & badI,2), 'Color', CM(i+1,:), 'marker', 'x', 'linestyle', 'none');
    end
    hold off
    axis equal tight
    legend(legendTitles);
    
    if( nargin == 2)
        % Plot ref
        empty = bitset(0,5);
        emptyI = ref.bad == empty;
        ref.x(emptyI) = [];
        ref.y(emptyI) = [];
        ref.data(emptyI,:) = [];
        ref.bad(emptyI) = [];
        ref.seg(emptyI) = [];
        badI = ref.bad ~= 0;
        
        nSegs = unique(ref.seg);
        CM = gray(size(nSegs,1)+1);
        change_current_figure(4)
        
        hold on
        
        
        plot(ref.rx, ref.ry, 'r.');
        
        plot(ref.x, ref.y, 'Color', CM(1,:), 'marker', '.', 'linestyle', 'none');
        for i = 1:size(nSegs,1)
            I = ref.seg == nSegs(i);
            legendTitles = cellstr(num2str(nSegs(1:i), 'Segment %d'));
            legend(legendTitles);
            plot(ref.x(I), ref.y(I), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
            plot(ref.x(I & badI), ref.y(I & badI), 'Color', CM(i+1,:), 'marker', 'x', 'linestyle', 'none');
        end
        
        axis equal tight
        hold off
        
        
        change_current_figure(5)
        
        hold on
        for i = 1:size(nSegs,1)
            I = ref.seg == nSegs(i);
            plot(ref.data(I,1), ref.data(I,2), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
            plot(ref.data(I & badI,1), ref.data(I & badI,2), 'Color', CM(i+1,:), 'marker', 'x', 'linestyle', 'none');
        end
        hold off
        axis equal tight
        legend(legendTitles);
    end
    
    
    drawnow;
end
