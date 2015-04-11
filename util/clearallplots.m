function [ ] = clearallplots( n )
%CLEARALLPLOTS clears all of the figure windows

    for i = 1:n
        if ~ishandle(i);
            figure(i);
        end

        change_current_figure(i);
        clf;
    end
    drawnow;

end

