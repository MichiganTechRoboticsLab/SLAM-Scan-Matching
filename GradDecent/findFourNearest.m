function [ p ] = findFourNearest( pt, pixelSize, RangeX, RangeY, minX, minY, maxX, maxY)
%This function finds the four nearest pixels to the given point
%now we have to define the four pixels nearest to us.

a = floor(pt(1));
b = floor(pt(2));

a = a + round(mod(pt(1),pixelSize)/pixelSize);
b = b + round(mod(pt(2),pixelSize)/pixelSize);

p = [ pixelSize/2 pixelSize/2 ];
p = [ [-1 -1].*p; [1 -1].*p; [-1 1].*p; [1 1].*p ];
p = p + repmat( [ a b ] , [ 4 1 ] );
p = ptToPx(p, pixelSize, minX, minY, maxX, maxY);

end

