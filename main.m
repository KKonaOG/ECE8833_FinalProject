%% Initialize by running a map
run("Maps/Map1.m");
[network, optimal_route, route_distance] = self_organizing_map(map_waypoints, map_size, 27500, 0.8);

for i=1:height(optimal_route)
    text(map_waypoints(optimal_route(i), 1), map_waypoints(optimal_route(i), 2), string(i));
end