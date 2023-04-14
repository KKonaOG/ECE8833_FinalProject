% <h1>Main.m</h1>
% <p>Main.m is responsible for handling the code-flow for the entire simulation.
%     This process is broken down into stages where each stage, more or less,
%     maps to a portion defined in the <a href="../README.cchtml">README</a>.
% </p>
% <h2><a id="Stage1"></a>&nbsp;Stage 1: <a
%         href="#h_68789198298041681190021467">Map Selection</a></h2>
% <p>This stage is responsible for executing a Map file such as <a
%         href="Maps/Map1.m">Map1.m</a> so that the workspace contains the
%     needed information to execute the remaining stages. A map file is
%     responsible for creating the map's figure, setting the map size, creating
%     map obstacles, and creating map waypoints. Importantly, it should contain
%     constants that define the various parameter inputs required by the
%     remaining stages.</p>
% <p>&nbsp;</p>
%% Initialize by running a map (this plots obstacles and waypoints)
run("Maps/Map1.m");

% <h2><a id="Stage2"></a>&nbsp;Stage 2a: <a
%         href="../README.cchtml#h_907645437334861681190082514">Determine
%         Semi-Optimal Waypoint Ordering</a></h2>
% <p>This stage is responsible for executing the Self-Organizing Map function
%     (<a href="Algorithms/TSP/self_organizing_map.m">self_organizing_map.m</a>)
%     on the Map provided in <a href="main.m#Stage1">Stage 1</a>.<br>Importantly
%     the function takes in the following parameters:</p>
% <ul>
%     <li>Map Waypoints</li>
%     <li>Map Size</li>
%     <li>Max Iterations</li>
%     <li>Initial Learning Rate</li>
% </ul>
% <p>The number of iterations and learning rate will need to be modified based
%     off each map by the researcher in order to identify an optimal parameter
%     set. It is recommended that once optimal parameter sets are found for each
%     map. That they are stored within the MapX.m file.</p>
% <p>The outputs of the function are:</p>
% <ul>
%     <li>Node Network</li>
%     <li>Best Calculated Route</li>
%     <li>Best Calculated Route Distance</li>
% </ul>
% <p>Importantly, the function does not produce a looping output by default,
%     this remedied by adding the starting waypoint back onto the end of the
%     best calculated route output by the function.</p>
% <p>&nbsp;</p>
%% Calculate TSP (Self-Organizing Maps)
solution_time = 0;
tic
[network, optimal_route, route_distance] = self_organizing_map(map_waypoints, map_size, 100000, 0.8);
optimal_route(end+1) = optimal_route(1); % We want the robot to loop back around

% <h2>Stage 2b (Optional): Display Route Ordering on Map and Output Solution
%     Calculation Time</h2>
% <p>This stage simply loops over the most optimal route calculated by the
%     Self-Organizing Map and labels the points in order on the map. It then
%     increments the solution calculation time and displays the time to solve
%     TSP.</p>
% <p>&nbsp;</p>
for i=1:height(optimal_route)
    text(map_waypoints(optimal_route(i), 1), map_waypoints(optimal_route(i), 2), string(i));
end
solution_time = solution_time + toc;
disp("Solved TSP in " + solution_time + " seconds");

% <h2><a id="Stage3"></a>&nbsp;Stage 3: <a
%         href="../README.cchtml#h_277825871586261681190638724">Find
%         Point-To-Point Path for Each Waypoint in Order</a></h2>
% <p>This stage is responsible for executing the Graft-RRT* function (<a
%         href="Algorithms/PTP/graft_rrtstar.m">graft_rrtstar.m</a>) on the
%     waypoints in the order they were provided in <a href="main.m#Stage2">Stage
%         2</a>.</p>
% <p>Importantly the function takes in the following parameters:</p>
% <ul>
%     <li>Start Waypoint</li>
%     <li>Target Waypoint</li>
%     <li>Map Size</li>
%     <li>Obstacle Buffers</li>
%     <li>Epsilon (Step Size)</li>
%     <li>Threshold (Threshold to Goal Point)</li>
%     <li>Number of Iterations</li>
% </ul>
% <p>The start waypoint and target/goal waypoint are determined by looping over
%     the length/height of the produced best route in <a
%         href="main.m#Stage2">Stage 2</a>. Obstacle Buffers are calculated in
%     the MapX.m file by the CreateObstacle function (<a
%         href="Maps/Helpers/CreateObstacle.m">CreateObstacle.m</a>). Epsilon,
%     Threshold, and Iterations are researcher and map dependent and once an
%     ideal set of parameter values has been identified it is recommend to save
%     them into the MapX.m file.</p>
% <p>The outputs of the function are:</p>
% <ul>
%     <li>Route</li>
%     <li>Route Calculation Time</li>
% </ul>
% <p>The output route is not the entire solution tree, but the singular best
%     route produced by the Graft-RRT algorithm. A line is then drawn from point
%     to point in order to visualize the path taken from waypoint to waypoint on
%     the map. Additionally, random colors are generated via the MATLAB command
%     <a href="https://www.mathworks.com/help/matlab/ref/hsv.html">hsv</a>, this
%     command is used simply to produce a unique variety of colors for each
%     route present within the solution.</p>
% <p>&nbsp;</p>
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

% <h2>Stage 4: <a href="../README.cchtml#h_330493934706951681190786455">Navigate
%         along the Path while Localizing and Mapping Terrain</a></h2>
% <p>[TODO]</p>
% <p>&nbsp;</p>