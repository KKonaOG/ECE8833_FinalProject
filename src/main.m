%% Initialization

% Clear Previous Workspace Information
clc;
clear;
close all;

% Setup Paths and add them to the environment
addpath(genpath("Algorithms"));
addpath(genpath("Digital Twin"));
addpath(genpath("Maps"));
addpath(genpath("Toolboxes"));
addpath(genpath('Toolboxes/MRS Toolbox/examples'));
addpath(genpath('Toolboxes/MRS Toolbox/src'));
addpath(genpath('Toolboxes/MRS Toolbox/doc'));

%% Parameter Selection

% Initialize new workspace by running a map (Valid Options: Map1.m, Map2.m, and Map 3.m)
run("Maps/Map1.m");

% Digital Twin - Priority Override
doPriorityOverride = false;
transitionWaypoint = [1 15];
priorityWaypoint = [15 1];

% Digital Twin - Obstacle Override
doObstacleOverride = false;
hiddenObstaclePolygon = [5 3; 5 6; 6 6; 6 3;];

%% Stage 1: Solve Travelling Salesman Problem (Self-Organizing Maps)
tic;
[network, optimal_route, route_distance] = self_organizing_map(map_waypoints, map_size, 100000, 0.8);
optimal_route(end+1) = optimal_route(1); % We want the robot to loop back around

for i=1:height(optimal_route)
    text(map_waypoints(optimal_route(i), 1), map_waypoints(optimal_route(i), 2), string(i));
end
disp("Solved TSP in " + toc + " seconds");

%% Digital Twin - Waypoint Priority Override
if (doPriorityOverride)
    % Generate Priority Route
    [priorityTransitionOccured, newRoute, priorityTransitionPoint] = PriorityOverride(transitionWaypoint, priorityWaypoint, optimal_route, map_waypoints, map_size);

    % This block handles painting a second figure for the Digital Twin to
    % graph on, it only opens if the transition produces actionable results
    if (priorityTransitionOccured)
        mapPrio = figure;
        show(occupancyMap);
        title(map_title + " - Priority Waypoint");
        axis ([0 map_size 0 map_size]);
        yticks(0:1:map_size);
        xticks(0:1:map_size);
        axis square;
        grid on;
        hold on;
        plot(map_waypoints(:,1), map_waypoints(:,2), "o");

        % Overlay the new route on the map
        for i=1:height(newRoute)
            text(map_waypoints(newRoute(i), 1), map_waypoints(newRoute(i), 2), string(i));
        end
    end
end

% In the event we are doing a priority override, prevent the obstacle
% override
if (doPriorityOverride && doObstacleOverride)
    doObstacleOverride = false;
    disp("Only one DT simulation can be run at once. Obstacle Override Ignored.");
end

%% Digital Twin - Obstacle Override
if (doObstacleOverride && ~doPriorityOverride)
    [mapObst, occupancyMapObst, occupancy_grid_obst] = ObstacleOverride(map_waypoints, ...
        hiddenObstaclePolygon, occupancyMap, optimal_route, map, map_size, ...
        map_obstacles, occupancy_grid, map_title);
end

%% Stage 3: Find Point-To-Point Path for Each Waypoint in Order
% This stage is responsible for executing the Graft-RRT* function  on the
% waypoints in the order they were provided in Stage 2 or by Digital Twin
% Importantly the function takes in the following parameters:
%     Start Waypoint
%     Target Waypoint
%     Map Size
%     Obstacle Buffers
%     Epsilon (Step Size)
%     Threshold (Threshold to Goal Point)
%     Number of Iterations
% The start waypoint and target/goal waypoint are determined by looping over
% the length/height of the produced best route in Stage 2.  Epsilon, Threshold, and Iterations are researcher and 
% map dependent and once an ideal set of parameter values has been identified it is recommend to save
% them into the MapX.m file.
% The outputs of the function are:
%     Route
%     Route Calculation Time
% The output route is not the entire solution tree, but the singular best
% route produced by the Graft-RRT algorithm. A line is then drawn from point
% to point in order to visualize the path taken from waypoint to waypoint on
% the map.

%% Find Navigation Solution via PTP Algorithm (Graft RRT)
route_solutions = struct(); % This pre-allocates an array with the number of routes we will generate (number of waypoints in optimal route - 1)
obstacle_route_solutions = struct(); % Same as route_solutions but is used in the event the Digital Twin for Obstacles is used

solution_time = 0;
tic
for i=1:height(optimal_route)-1
    % Creates the route solution for the original map
    [route, route_calculation_time] = graft_rrtstar(map_waypoints(optimal_route(i), :), map_waypoints(optimal_route(i+1), :), map_size, occupancy_grid,map_rrt_epsilon, map_rrt_threshold, map_rrt_iterations);
    solution_time = solution_time + route_calculation_time;
    route_solutions(i).data = route;
    route_solutions(i).recalculated = false;

    % If we are doing obstacle override, we check the generated path and
    % see if we need to re-perform the RRT on the specific waypoint
    % transition (i.e it collides with the new obstacles)
    if (doObstacleOverride)
        for j=1:height(route)-1
            if (~ObstacleFree(route(j, 1:2), route(j+1, 1:2), map_size, occupancy_grid_obst))
                [new_route, new_route_calculation_time] = graft_rrtstar(map_waypoints(optimal_route(i), :), map_waypoints(optimal_route(i+1), :), map_size, occupancy_grid_obst,map_rrt_epsilon, map_rrt_threshold, map_rrt_iterations);
                solution_time = solution_time + route_calculation_time;
                obstacle_route_solutions(i).data = new_route;
                route_solutions(i).recalculated = true;
                break;
            end
        end
    end
end

% This handles creating an entirely new path for the remaing route
% waypoints
if (doPriorityOverride && priorityTransitionOccured)
    priority_route_solutions = struct();
    for i=1:height(newRoute)-1
        [prio_route, prio_route_calculation_time] = graft_rrtstar(map_waypoints(newRoute(i), :), map_waypoints(newRoute(i+1), :), map_size, occupancy_grid,map_rrt_epsilon, map_rrt_threshold, map_rrt_iterations);
        solution_time = solution_time + prio_route_calculation_time;
        priority_route_solutions(i).data = prio_route;
    end
end

disp("Path Plan Generated in " + solution_time + " seconds")

% Plot Navigation Solution
for i=1:height(optimal_route)-1

    % This block handles plotting data in the event priority override was
    % enabled
    if  (doPriorityOverride && priorityTransitionOccured && i >= priorityTransitionPoint)
        figure(mapPrio);
        solutionPath = priority_route_solutions(i).data;
        line(solutionPath(:, 1), solutionPath(:, 2), "Color", "k", "LineStyle", "--")
    elseif (doPriorityOverride && priorityTransitionOccured)
        figure(mapPrio);
        solutionPath = route_solutions(i).data;
        priority_route_solutions(i).data = solutionPath;
        line(solutionPath(:, 1), solutionPath(:, 2), "Color", "g", "LineStyle", "-")
    end


    % This block handles plotting in the event that the obstale override
    % was enabled, recalculated is only set to true in the event the path
    % is recalculated, and is all false when override is disabled. This
    % means we don't need a specific if check for the doObstacleOverride
    solutionPath = route_solutions(i).data;
    if (route_solutions(i).recalculated)
        figure(map)
        line(solutionPath(:, 1), solutionPath(:, 2), "Color", "r", "LineStyle", "-")
        figure(mapObst)
        line(solutionPath(:, 1), solutionPath(:, 2), "Color", "r", "LineStyle", "-")
        solutionPath = obstacle_route_solutions(i).data;
        figure(mapObst)
        line(solutionPath(:, 1), solutionPath(:, 2), "Color", "g", "LineStyle", "--")
    else
        figure(map)
        line(solutionPath(:, 1), solutionPath(:, 2), "Color", "g", "LineStyle", "-")
        if (doObstacleOverride)
            figure(mapObst)
            line(solutionPath(:, 1), solutionPath(:, 2), "Color", "g", "LineStyle", "-")
        end
    end
end

% This simply handles executing the VFH based off the execution parameters
if (doPriorityOverride && priorityTransitionOccured)
    % Priority Waypoint VFH Case
    VFHOnPath(priority_route_solutions, mapPrio, true);
    VFHOnPath(route_solutions, map, false);
elseif (doObstacleOverride)
    % Obstacle Override VFH Case
    solutionPath = route_solutions;

    % This regenerates a combined solution since the the obstacle route
    % simply uses recalculated values
    for i=1:height(optimal_route)-1
        if (route_solutions(i).recalculated)
            solutionPath(i).data = obstacle_route_solutions(i).data;
            route_solutions(i).data = [map_waypoints(optimal_route(i), :); map_waypoints(optimal_route(i+1), :)]; % This allows the VFH to "postulate" on how to solve invalid paths
        end
    end

    VFHOnPath(solutionPath, mapObst, true);
    VFHOnPath(route_solutions, map, false);
else
    % Normal Run Case - No Digital Twin Case
    VFHOnPath(route_solutions, map, false);
end

