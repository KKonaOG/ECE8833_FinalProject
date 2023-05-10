%% Map 1: Low Obstacle Count, Low Complexity
% This file configures Map 1 which is intended to be used in main.m as a "demonstration" of the system. It
% features a low count, low complexity set of obstacles. The remainder of this file demonstrates a
% semi-standardized way of creating Map files.

%% Map Configuration
map_title = "Map 1: Low Count, Low Complexity Obstacles";
map_size = 16;
occupancyMap = binaryOccupancyMap(map_size, map_size, 1, "grid");
occupancy_grid = zeros(map_size, map_size);

% Map RRT Parameters
map_rrt_epsilon = 1;
map_rrt_iterations = 15000;
map_rrt_threshold = 0.3;

%% Create Map Waypoints 
%  1st Parameter: Input Waypoint Array</li>
%  2nd Parameter: New Waypoint X,Y</li>
%  Input Waypoint Array is either a new array, [], or the output array
%  provided by the previous CreateWaypoint call (in the case of the code
%  below map_waypoints)<br>New Waypoint is a simple array containing a
%  singular set of X,Y coordinates. As an example, if I wanted a waypoint on
%  the point 10, 4 then the waypoint would look like [10 4].
map_waypoints = CreateWaypoint([], [1 1]);
map_waypoints = CreateWaypoint(map_waypoints, [1 15]);
map_waypoints = CreateWaypoint(map_waypoints, [8 4]);
map_waypoints = CreateWaypoint(map_waypoints, [7 12]);
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
map_obstacles = CreateObstacle([], [2 8; 2 16; 4 16; 4 8;]);
map_obstacles = CreateObstacle(map_obstacles, [1 3; 1 6; 4 6; 4 3;]);
map_obstacles = CreateObstacle(map_obstacles, [5 1; 5 2; 9 2; 9 1;]);
map_obstacles = CreateObstacle(map_obstacles, [7 6; 7 10; 9 10; 9 6;]);
map_obstacles = CreateObstacle(map_obstacles, [8 13; 8 16; 9 16; 9 13;]);
map_obstacles = CreateObstacle(map_obstacles, [11 0; 11 8; 13 8; 13 0;]);
map_obstacles = CreateObstacle(map_obstacles, [12 11; 12 14; 15 14; 15 11;]);

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
