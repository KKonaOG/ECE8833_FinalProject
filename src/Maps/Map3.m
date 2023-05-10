%% Map 1: High Obstacle Count, Low Complexity
% This file configures Map 3 which is intended to be used in main.m as a "demonstration" of the system. It
% features a high count, low complexity set of obstacles. The remainder of this file demonstrates a
% semi-standardized way of creating Map files.

%% Map Configuration
map_title = "Map 3: High Count, Low Complexity Obstacles";
map_size = 16;
occupancyMap = binaryOccupancyMap(map_size, map_size, 1, "grid");
occupancy_grid = zeros(map_size, map_size);

% Map RRT Parameters
map_rrt_epsilon = 1;
map_rrt_iterations = 10000;
map_rrt_threshold = 0.5;

%% Create Map Waypoints 
%  1st Parameter: Input Waypoint Array</li>
%  2nd Parameter: New Waypoint X,Y</li>
%  Input Waypoint Array is either a new array, [], or the output array
%  provided by the previous CreateWaypoint call (in the case of the code
%  below map_waypoints)<br>New Waypoint is a simple array containing a
%  singular set of X,Y coordinates. As an example, if I wanted a waypoint on
%  the point 10, 4 then the waypoint would look like [10 4].
map_waypoints = CreateWaypoint([], [1 2]);
map_waypoints = CreateWaypoint(map_waypoints, [1 15]);
map_waypoints = CreateWaypoint(map_waypoints, [7 4]);
map_waypoints = CreateWaypoint(map_waypoints, [12 9.5]);
map_waypoints = CreateWaypoint(map_waypoints, [15 15]);
map_waypoints = CreateWaypoint(map_waypoints, [15 1]);

%% Create Map Obstacles
% If you wish tosimulate an obstacle proceeding partially in and off the map, simply clip
% the obstacle vertices to be directly on the upper or lower bound of the
% map size. 
% The CreateObstacle function takes in the following parameters:</p>
%     1st Parameter: Input Obstacle Array
%     2nd Parameter: Obstacle Vertices in Clockwise Order
% 
% Input Obstacle Array is either a new array, [], or the output array
% provided by the previous CreateObstacle call (in the case of the code
% below map_obstacles)
% 
% Obstacle Vertices is a column array with X values in column 1 and Y values in column 2
[map_obstacles] = CreateObstacle([], [0 0; 0 1; 4 1; 4 0]);
[map_obstacles] = CreateObstacle(map_obstacles, [0 4; 0 5; 4 5; 4 4]);
[map_obstacles] = CreateObstacle(map_obstacles, [0 8; 0 9; 4 9; 4 8]);
[map_obstacles] = CreateObstacle(map_obstacles, [0 13; 0 14; 4 14; 4 13]);
[map_obstacles] = CreateObstacle(map_obstacles, [3 2; 3 3; 5 3; 5 5; 6 5; 6 3; 8 3; 8 2]);
[map_obstacles] = CreateObstacle(map_obstacles, [3 6; 3 7; 8 7; 8 9; 9 9; 9 4; 8 4; 8 6;]);
[map_obstacles] = CreateObstacle(map_obstacles, [6 8; 6 10; 4 10; 4 12; 6 12; 6 14; 7 14; 7 12; 10 12; 10 10; 7 10; 7 8]);
[map_obstacles] = CreateObstacle(map_obstacles, [9 14; 9 15; 14 15; 14 14;]);
[map_obstacles] = CreateObstacle(map_obstacles, [12 10; 12 12; 16 12; 16 10;]);
[map_obstacles] = CreateObstacle(map_obstacles, [10 4; 10 9; 14 9; 14 8; 11 8; 11 5; 14 5; 14 4]);
[map_obstacles] = CreateObstacle(map_obstacles, [12 2; 12 3; 16 3; 16 2]);

% Convert polyshape obstacles to occupancy grid 
for i = 1:map_size
    for j = 1:map_size
        x = map_size - i - 0.5 + 1;
        y = j - 0.5;
        isOccupied = false;
        for k = 1:numel(map_obstacles)
            isOccupied = inpolygon(x, y, map_obstacles(k).Vertices(:, 2), map_obstacles(k).Vertices(:, 1));
            if (isOccupied)
                occupancy_grid(i, j) = 1;
                break
            end
        end
    end
end

%% Update Map with Generated Occupancy Map
setOccupancy(occupancyMap, occupancy_grid);
map = figure;
show(occupancyMap);

%% Configure Map Display
title(map_title);
axis ([0 map_size 0 map_size]);
yticks(0:1:map_size);
xticks(0:1:map_size);
axis square;
grid on;
hold on;


% Plot Waypoint Configuration onto the Map Figure
plot(map_waypoints(:,1), map_waypoints(:,2), "o");
grid on;


% We do some cleanup of unneeded workspace variables
clear i
clear j
clear k
clear x
clear y
