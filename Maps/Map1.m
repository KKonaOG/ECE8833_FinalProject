%% Map Configuration
map = figure();
map_size = 16;
axis ([0 map_size 0 map_size]);
yticks(0:2:map_size);
xticks(0:2:map_size);
axis square;
hold on;

%% Create Waypoints
map_waypoints = CreateWaypoint([], [4 14]);
map_waypoints = CreateWaypoint(map_waypoints, [13 12]);
map_waypoints = CreateWaypoint(map_waypoints, [13 9]);
map_waypoints = CreateWaypoint(map_waypoints, [15 5]);
map_waypoints = CreateWaypoint(map_waypoints, [13 4]);
map_waypoints = CreateWaypoint(map_waypoints, [4 3]);
map_waypoints_x = map_waypoints(:, 1);
map_waypoints(:, 1) = map_waypoints(:, 2);
map_waypoints(:, 2) = map_waypoints_x;

%% Create Obstacles (This applies a buffer of 5 units)
[map_obstacles, obstacle_buffers] = CreateObstacle([], [], [1 8; 1 10; 7 10; 7 8;]);
[map_obstacles, obstacle_buffers] = CreateObstacle(map_obstacles, obstacle_buffers, [9 8; 9 10; 15 10; 15 8;]);

%% Plot Waypoints on Map
plot(map_waypoints(:,1), map_waypoints(:,2), "o");
plot(map_obstacles);
plot(obstacle_buffers);
grid on;