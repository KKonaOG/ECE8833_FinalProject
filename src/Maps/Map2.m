% <h2 style="text-align: left;">Map 1: Low Obstacle Count, Low Complexity</h2>
% <p>This file configures Map 1 which is intended to be used in <a
%         href="src/main.m">main.m</a> as a "demonstration" of the system. It
%     features a low count, low complexity set of obstacles which can be used to
%     verify code functionality.</p>
% <p><img style="display: block; margin-left: auto; margin-right: auto;"
%         title="Map 1" src="images/Map1.png" alt="Map 1" width="537"
%         height="435"></p>
% <p style="text-align: left;">The remainder of this file demonstrates a
%     semi-standardized way of creating Map files.</p>
% <h2 style="text-align: left;">Configure Map Boundaries and Visualization</h2>
% <p>Configurations such as the map title, map size (map_size) and ticks
%     (yticks, xticks) are dependent on the particular map
%     implementation.<br>Modifications to axis are <strong><span
%             style="text-decoration: underline;">not</span></strong>
%     recommended as the system is intended to run with a lower bound of zero.
%     Non-zero use cases have not been tested.<br><br></p>
map = figure();
map_title = "Map 2: Low Count, Low Complexity Obstacles";
title(map_title)
map_size = 16;
axis ([0 map_size 0 map_size]);
yticks(0:1:map_size);
xticks(0:1:map_size);
axis square;
grid on;
hold on;

% <h2>Create Map Waypoints (See:&nbsp;<a
%         href="src/Maps/Helpers/CreateWaypoint.m">CreateWaypoint.m</a>)</h2>
% <p>The create waypoint system does <span
%         style="text-decoration: underline;"><strong>not</strong></span> verify
%     the waypoints fall within the boundaries of the map size. As such, care
%     should be taken when implementing the waypoints such that they remain
%     fully within the specified map boundaries of 0 to map_size on both the X
%     and Y axis.<br><br></p>
% <p>The CreateWaypoint function takes in the following parameters:</p>
% <ul>
%     <li>1st Parameter: Input Waypoint Array</li>
%     <li>2nd Parameter: New Waypoint X,Y</li>
% </ul>
% <p>Input Waypoint Array is either a new array, [], or the output array
%     provided by the previous CreateWaypoint call (in the case of the code
%     below map_waypoints)<br>New Waypoint is a simple array containing a
%     singular set of X,Y coordinates. As an example, if I wanted a waypoint on
%     the point 10, 4 then the waypoint would look like [10 4].</p>
% <p>&nbsp;</p>
map_waypoints = CreateWaypoint([], [1 4]);
map_waypoints = CreateWaypoint(map_waypoints, [1 15]);
map_waypoints = CreateWaypoint(map_waypoints, [8 4]);
map_waypoints = CreateWaypoint(map_waypoints, [12 9]);
map_waypoints = CreateWaypoint(map_waypoints, [12 15]);
map_waypoints = CreateWaypoint(map_waypoints, [15 1]);

% <h2>Create Map Obstacles (See:&nbsp;<a
%         href="src/Maps/Helpers/CreateObstacle.m">CreateObstacle.m</a>)</h2>
% <p>Similar to the create waypoint system, the create obstacle system
%     does&nbsp;<span
%         style="text-decoration: underline;"><strong>not</strong></span> verify
%     the obstacle points fall within the boundaries of the map. If you wish to
%     simulate an obstacle proceeding partially in and off the map, simply clip
%     the obstacle vertices to be directly on the upper or lower bound of the
%     map size. An additional important note is that the create obstacle system
%     applies a buffer of 0.5 units to the obstacle as safety buffer to prevent
%     collisions with the PTP scheme.</p>
% <p>The CreateObstacle function takes in the following parameters:</p>
% <ul>
%     <li>1st Parameter: Input Obstacle Array</li>
%     <li>2nd Parameter: Input Obstacle Buffers</li>
%     <li>3rd Parameter: Obstacle Vertices in Clockwise Order</li>
% </ul>
% <p>Input Obstacle Array is either a new array, [], or the output array
%     provided by the previous CreateObstacle call (in the case of the code
%     below map_obstacles)<br>Input Obstacle Buffers is either a new array, [],
%     or the buffers provided by the previous CreateObstacle call (in the case
%     of the code below obstacle_buffers)<br>Obstacle Vertices is a column array
%     with X values in column 1 and Y values in column 2</p>
% <p>&nbsp;</p>
[map_obstacles, obstacle_buffers] = CreateObstacle([], [], [0 12; 0 14; 4 14; 4 12]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [1 5; 1 10; 3 10; 3 5]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [1 1; 1 3; 7 3; 7 1]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [5 5; 5 10; 7 10; 7 5]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [6 11; 6 16; 8 16; 8 11]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [9 10; 9 13; 14 13; 14 10]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [15 13; 15 16; 16 16; 16 13]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [9 2; 9 7; 12 7; 12 2]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [14 2; 14 8; 16 8; 16 2]);

% <h2>Plot Configuration onto the Map Figure</h2>
plot(map_waypoints(:,1), map_waypoints(:,2), "o");
plot(map_obstacles, "FaceColor", "Black", "FaceAlpha", 1);
plot(obstacle_buffers);
grid on;
