function [point] = rand_center(qsource, qgoal, map_size, occupancy_grid)
%GETTREE Returns the Tgraft tree discussed in the Graft-RRT algorithm (Map
% Size is used as a resolution constraint on generating center point)

% Another implementation of this process is available at: https://github.com/robot-rolle/path-plan/blob/Graft_RRT/rand_center.m
% We do not attempt to calculate direction and process points along that
% line. Instead we generate a series of points from start to goal and
% filter out any points within an obstacle or outside the map.

% Straight Line of Points from Start_Waypoint to Goal_Waypoint

% We calculate midpoint and generate two line segments based off that
% (before and after the midpoint) such that we have points closer to the
% end_point and points closer to the start_point. Selecting the N closest
% points to the midpoint out of those points on either segment we can then
% randomly sample for a "center" point. In the terms of the paper, our
% distance parameters would be related to maximizing the distance of a
% point from both the start and end point.
straightLineMidpoint = [((qsource(1) + qgoal(1))/2) ((qsource(2) + qgoal(2))/2)];


% By default linspace uses 100 points (maybe this should be investigated at
% lower resolutions such as map_size or map_size * 2). This currently
% chooses the highest between the two in order to maintain some form of
% "scalability" on maps larger than 100 units though resolution will be 1:1
% rather than many points per unit
straightLineX = linspace(qsource(1), straightLineMidpoint(1), max(100, map_size));
straightLineY = linspace(qsource(2), straightLineMidpoint(2), max(100, map_size));
straightLineFromStart = [straightLineX', straightLineY'];

straightLineX = linspace(qgoal(1), straightLineMidpoint(1), max(100, map_size));
straightLineY = linspace(qgoal(2), straightLineMidpoint(2), max(100, map_size));
straightLineFromEnd = [straightLineX', straightLineY'];


% Remove Points within an Obstacle
[~, validStraightLineFromStart] = CheckCollision(straightLineFromStart, occupancy_grid, map_size);
[~, validStraightLineFromEnd] = CheckCollision(straightLineFromEnd, occupancy_grid, map_size);


% Calculate 25% of the number of total valid points for each segment and
% allocate that to our random sampling set
numberCenterPointsFromStart = floor(0.25*(height(validStraightLineFromStart)));
numberCenterPointsFromEnd = floor(0.25*(height(validStraightLineFromEnd)));

% Gets the desired points from the start points (takes them from closest to
% furthest from mid point)
centerStartPoints = zeros(numberCenterPointsFromStart, 2);
for i=1:numberCenterPointsFromStart
    centerStartPoints(i, :) = validStraightLineFromStart(height(validStraightLineFromStart)-i, :);
end

% Gets the desired points from the end points (takes them from closest to
% furthest from mid point)
centerEndPoints = zeros(numberCenterPointsFromEnd, 2);
for i=1:numberCenterPointsFromEnd
    centerEndPoints(i, :) = validStraightLineFromEnd(height(validStraightLineFromEnd)-i, :);
end

% Chooses a random point from the calculated portions of the segments and
% uses that as the root node of the graft tree
try
    sampleSet = [centerStartPoints;centerEndPoints];
    point = sampleSet(randi(height(sampleSet), 1), :);
catch
    disp("Sample Set issue");
end
end

