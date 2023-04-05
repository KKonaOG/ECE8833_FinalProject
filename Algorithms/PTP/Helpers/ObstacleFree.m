function [inFreeSpace] = ObstacleFree(pointOne,pointTwo, map_size, obstacles)
%OBSTACLEFREE Takes in two arguments: Point 1 and Point 2. It generates an
% evenly space distribution of points along the line created by Point 1 and
% Point 2. The resolution of this distribution is determined by max(100,
% map_size) where it will distribute either 100 points or map_size points
% along the line depending on whichever is greatest. These points are then
% compared with inpolygon on each obstacle to verify the lines do not
% collide. ObstacleFree simply returns a true or false value.

lineXPoints = linspace(pointOne(1, 1), pointTwo(1, 1), max(100, map_size));
lineYPoints = linspace(pointOne(1, 2), pointTwo(1, 2), max(100, map_size));
line = [lineXPoints', lineYPoints'];

inFreeSpace = true;

% Remove Points Exceeding X Bounds ( > map_size)
invalidMapBoundsX = line(:, 1) >= map_size;
if (any(invalidMapBoundsX == 1))
    inFreeSpace = false;
    return;
end

% Removing Points Exceeding X Bounds (< 0)
invalidMapBoundsX = line(:, 1) <= 0;
if (any(invalidMapBoundsX == 1))
    inFreeSpace = false;
    return;
end

% Remvoing Points Exceeding Y Bounds (> map_size)
invalidMapBoundsY = line(:, 2) >= map_size;
if (any(invalidMapBoundsY == 1))
    inFreeSpace = false;
    return;
end

% Removing Points Exceeding X Bounds (< 0)
invalidMapBoundsY = line(:, 2) <= 0;
if (any(invalidMapBoundsY == 1))
    inFreeSpace = false;
    return;
end


for i=1:height(obstacles)
    [inPoly] = inpolygon(line(:, 1), line(:, 2), obstacles(i).Vertices(:, 1), obstacles(i).Vertices(:, 2));
    if (any(inPoly==1))
        inFreeSpace = false;
        break;
    end
end

end

