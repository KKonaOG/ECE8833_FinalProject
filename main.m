%% Initialize by running a map (this plots obstacles and waypoints)
run("Maps/Map1.m");

%% Calculate TSP (Self-Organizing Maps)
solution_time = 0;
tic
[network, optimal_route, route_distance] = self_organizing_map(map_waypoints, map_size, 100000, 0.8);
optimal_route(end+1) = optimal_route(1); % We want the robot to loop back around
for i=1:height(optimal_route)
    text(map_waypoints(optimal_route(i), 1), map_waypoints(optimal_route(i), 2), string(i));
end
solution_time = solution_time + toc;
disp("Solved TSP in " + solution_time + " seconds");

%% Find Navigation Solution via PTP Algorithm (Graft RRT)
route_solutions = struct(); % This pre-allocates an array with the number of routes we will generate (number of waypoints in optimal route - 1)
colors = hsv(height(optimal_route)-1);
for i=1:height(optimal_route)-1
    [route, route_calculation_time] = graft_rrtstar(map_waypoints(optimal_route(i), :), map_waypoints(optimal_route(i+1), :), map_size, obstacle_buffers, 2, 1, 10000);
    color = colors(i, :);
    for j=1:height(route)-1
        line([route(j, 1), route(j+1, 1)], [route(j, 2), route(j+1, 2)], "Color", color);
    end
    solution_time = solution_time + route_calculation_time;
    route_solutions(i).data = route;
end
disp("Produced Navigable Route in " + solution_time + " seconds")

%% Local Navigation