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
map_title = "Map 3: Low Count, Low Complexity Obstacles";
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
map_waypoints = CreateWaypoint([], [1 2]);
map_waypoints = CreateWaypoint(map_waypoints, [1 15]);
map_waypoints = CreateWaypoint(map_waypoints, [7 4]);
map_waypoints = CreateWaypoint(map_waypoints, [12 9.5]);
map_waypoints = CreateWaypoint(map_waypoints, [15 15]);
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
[map_obstacles, ~] = CreateObstacle([], [], [0 0; 0 1; 4 1; 4 0]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [0 4; 0 5; 4 5; 4 4]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [0 8; 0 9; 4 9; 4 8]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [0 13; 0 14; 4 14; 4 13]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [3 2; 3 3; 5 3; 5 5; 6 5; 6 3; 8 3; 8 2]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [3 6; 3 7; 8 7; 8 9; 9 9; 9 4; 8 4; 8 6;]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [6 8; 6 10; 4 10; 4 12; 6 12; 6 14; 7 14; 7 12; 10 12; 10 10; 7 10; 7 8]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [9 14; 9 15; 14 15; 14 14;]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [12 10; 12 12; 16 12; 16 10;]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [10 4; 10 9; 14 9; 14 8; 11 8; 11 5; 14 5; 14 4]);
[map_obstacles, ~] = CreateObstacle(map_obstacles, [], [12 2; 12 3; 16 3; 16 2]);

% <h2>Plot Configuration onto the Map Figure</h2>
plot(map_waypoints(:,1), map_waypoints(:,2), "o");
plot(map_obstacles, "FaceColor", "Black", "FaceAlpha", 1);
%plot(obstacle_buffers, "FaceColor", "Black");
grid on;
