function [inFreeSpace] = ObstacleFree(pointOne,pointTwo, map_size, occupancy_grid)
%OBSTACLEFREE Takes in two arguments: Point 1 and Point 2. It generates an
% evenly space distribution of points along the line created by Point 1 and
% Point 2. The resolution of this distribution is determined by max(100,
% map_size) where it will distribute either 100 points or map_size points
% along the line depending on whichever is greatest. These points are then
% compared with the obstacle occupancy grid to verify the lines do not
% collide. ObstacleFree simply returns a true or false value.

lineXPoints = linspace(pointOne(1, 1), pointTwo(1, 1), max(100, map_size));
lineYPoints = linspace(pointOne(1, 2), pointTwo(1, 2), max(100, map_size));
line = [lineXPoints', lineYPoints'];

[containsValid, validPoints] = CheckCollision(line, occupancy_grid, map_size);
inFreeSpace = height(validPoints) == height(line);

end

