%% Map Configuration
map = figure();
map_size = 16;
axis ([0 map_size 0 map_size]);
yticks(0:1:map_size);
xticks(0:1:map_size);
axis square;
hold on;

%% Create Waypoints
map_waypoints = CreateWaypoint([], [4 14]);
map_waypoints = CreateWaypoint(map_waypoints, [7 12]);
map_waypoints = CreateWaypoint(map_waypoints, [7 9]);
map_waypoints = CreateWaypoint(map_waypoints, [13 5]);
map_waypoints = CreateWaypoint(map_waypoints, [9 4]);
map_waypoints = CreateWaypoint(map_waypoints, [4 3]);
map_waypoints_x = map_waypoints(:, 1);
map_waypoints(:, 1) = map_waypoints(:, 2);
map_waypoints(:, 2) = map_waypoints_x;

%% Plot Waypoints on Map
plot(map_waypoints(:,1), map_waypoints(:,2), "o");
grid on;