function plotSegments(ls, ref)
    narginchk(1, 2);
    global figs
    badColors = lines(5);
    bad = bitset(0,1:5);
    rangeI = bitand(ls.bad,bad(1))~=0;
    movingI = bitand(ls.bad,bad(2))~=0;
    mixedI = bitand(ls.bad,bad(3))~=0;
    occludedI = bitand(ls.bad,bad(4))~=0;
    emptyI = bitand(ls.bad,bad(5))~=0;
    nSegs = unique(ls.seg);
    CM = jet(size(nSegs,1)+1);
    change_current_figure(4)
    cla reset
    hold on
    
    
    plot(ls.rx, ls.ry, 'Color', CM(1,:), 'marker', '.', 'linestyle', 'none');
    legendTitlesCart = ['Scan Origin'; cellstr(num2str(nSegs, 'Scan Segment %d'))];
    
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
        
        plot(ls.x(I), ls.y(I), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');        
    end
    if(any(rangeI))
        plot(ls.x(rangeI), ls.y(rangeI), 'Color', badColors(1,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesCart = [legendTitlesCart; 'Scan Range'];
    end
    if(any(movingI))
        plot(ls.x(movingI), ls.y(movingI), 'Color', badColors(2,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesCart = [legendTitlesCart; 'Scan Moving'];
    end
    if(any(mixedI))
        plot(ls.x(mixedI), ls.y(mixedI), 'Color', badColors(3,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesCart = [legendTitlesCart; 'Scan Mixed'];
    end
    if(any(occludedI))
        plot(ls.x(occludedI), ls.y(occludedI), 'Color', badColors(4,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesCart = [legendTitlesCart; 'Scan Occluded'];
    end
    if(any(emptyI))
        plot(ls.x(emptyI), ls.y(emptyI), 'Color', badColors(5,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesCart = [legendTitlesCart; 'Scan Empty'];
    end
    axis equal
    hold off
    
    
    change_current_figure(5)
    cla reset
    hold on
    legendTitlesPol = cellstr(num2str(nSegs, 'Segment %d'));
    for i = 1:size(nSegs,1)
        I = ls.seg == nSegs(i);
     
        plot(ls.data(I,1), ls.data(I,2), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
        
    end
    if(any(rangeI))
        plot(ls.data(rangeI,1), ls.data(rangeI,2), 'Color', badColors(1,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesPol = [legendTitlesPol; 'Scan Range'];
    end
    if(any(movingI))
        plot(ls.data(movingI,1), ls.data(movingI,2), 'Color', badColors(2,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesPol = [legendTitlesPol; 'Scan Moving'];
    end
    if(any(mixedI))
        plot(ls.data(mixedI,1), ls.data(mixedI,2), 'Color', badColors(3,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesPol = [legendTitlesPol; 'Scan Mixed'];
    end
    if(any(occludedI))
        plot(ls.data(occludedI,1), ls.data(occludedI,2), 'Color', badColors(4,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesPol = [legendTitlesPol; 'Scan Occluded'];
    end
    if(any(emptyI))
        plot(ls.data(emptyI,1), ls.data(emptyI,2), 'Color', badColors(5,:), 'marker', 'x', 'linestyle', 'none');
        legendTitlesPol = [legendTitlesPol; 'Scan Empty'];
    end
    hold off
    
    
    if( nargin == 2)
        % Plot ref
        rangeI = bitand(ref.bad,bad(1))~=0;
        movingI = bitand(ref.bad,bad(2))~=0;
        mixedI = bitand(ref.bad,bad(3))~=0;
        occludedI = bitand(ref.bad,bad(4))~=0;
        emptyI = bitand(ref.bad,bad(5))~=0;
        nSegs = unique(ref.seg);
        CM = jet(size(nSegs,1)+1);
        change_current_figure(4)
%         cla reset
        hold on
        
        
        plot(ref.rx, ref.ry, 'Color', CM(1,:), 'marker', '.', 'linestyle', 'none');
        legendTitlesCart = [legendTitlesCart; 'Ref Origin'; cellstr(num2str(nSegs, 'Ref Segment %d'))];
        for i = 1:size(nSegs,1)
            I = ref.seg == nSegs(i);
            
            
            plot(ref.x(I), ref.y(I), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
           
%             legend(legendTitles,'Location','northoutside','Orientation','horizontal');
        end
        if(any(rangeI))
            plot(ref.x(rangeI), ref.y(rangeI), 'Color', badColors(1,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesCart = [legendTitlesCart; 'Ref Range'];
        end
        if(any(movingI))
            plot(ref.x(movingI), ref.y(movingI), 'Color', badColors(2,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesCart = [legendTitlesCart; 'Ref Moving'];
        end
        if(any(mixedI))
            plot(ref.x(mixedI), ref.y(mixedI), 'Color', badColors(3,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesCart = [legendTitlesCart; 'Ref Mixed'];
        end
        if(any(occludedI))
            plot(ref.x(occludedI), ref.y(occludedI), 'Color', badColors(4,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesCart = [legendTitlesCart; 'Ref Occluded'];
        end
        if(any(emptyI))
            plot(ref.x(emptyI), ref.y(emptyI), 'Color', badColors(5,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesCart = [legendTitlesCart; 'Ref Empty'];
        end
        
        axis equal
        hold off
        
        
        change_current_figure(5)
%         cla reset
        hold on
        legendTitlesPol = [legendTitlesPol; cellstr(num2str(nSegs, 'Ref Segment %d'))];
        for i = 1:size(nSegs,1)
            I = ref.seg == nSegs(i);
            plot(ref.data(I,1), ref.data(I,2), 'Color', CM(i+1,:), 'marker', '.', 'linestyle', 'none');
            
%             legend(legendTitles,'Location','northoutside','Orientation','horizontal');
        end
        if(any(rangeI))
            plot(ref.data(rangeI,1), ref.data(rangeI,2), 'Color', badColors(1,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesPol = [legendTitlesPol; 'Ref Range'];
        end
        if(any(movingI))
            plot(ref.data(movingI,1), ref.data(movingI,2), 'Color', badColors(2,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesPol = [legendTitlesPol; 'Ref Moving'];
        end
        if(any(mixedI))
            plot(ref.data(mixedI,1), ref.data(mixedI,2), 'Color', badColors(3,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesPol = [legendTitlesPol; 'Ref Mixed'];
        end
        if(any(occludedI))
            plot(ref.data(occludedI,1), ref.data(occludedI,2), 'Color', badColors(4,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesPol = [legendTitlesPol; 'Ref Occluded'];
        end
        if(any(emptyI))
            plot(ref.data(emptyI,1), ref.data(emptyI,2), 'Color', badColors(5,:), 'marker', 'x', 'linestyle', 'none');
            legendTitlesPol = [legendTitlesPol; 'Ref Empty'];
        end
        hold off
    end
    
    change_current_figure(4)
    legend(legendTitlesCart,'Location','eastoutside');
    change_current_figure(5)
    legend(legendTitlesPol,'Location','eastoutside');
    drawnow;
end
