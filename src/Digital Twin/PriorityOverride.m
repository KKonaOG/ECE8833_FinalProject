function [isNeeded, newRoute, transitionPoint] = PriorityOverride(transitionWaypoint,priorityWaypoint,optimalRoute,mapWaypoints,mapSize)
newRoute = zeros(height(optimalRoute), 1);

% The user has indicated they would like a specific translation on two
% waypoints. They should be specified as the actual waypoint
% coordinates as we do not want to rely on the "current" ordering. That
% is, we will force a "re-wire" at a "current position" to go towards the "priority" waypoint.

% Get Map Waypoint Index (ID)
waypoint_curr_id = find(ismember(mapWaypoints, transitionWaypoint,'rows'));

% Get Map Waypoint Location in SOM-route
waypoint_route_index = find(optimalRoute == waypoint_curr_id, 1);
transitionPoint = waypoint_route_index;

% Determine already travelled waypoints
newRoute(1:waypoint_route_index) = optimalRoute(1:waypoint_route_index);
route_to_travel = optimalRoute(waypoint_route_index+1:end);

% We now circumvent SOM-ordering by merging the next-waypoint into the
% next position of route_traveled.

% Get Map Waypoint Index (ID)
waypoint_prio_id = find(ismember(mapWaypoints, priorityWaypoint,'rows'));
waypoint_prio_route_index = find(optimalRoute == waypoint_prio_id, 1);

% We check to see if the swap makes sense, aka, have we already
% "visited" the priority waypoint by the time the waypoint transition
% would occur
isNeeded = true;
if (waypoint_prio_route_index < waypoint_route_index)
    disp("Priority Override not needed")
    isNeeded = false;
    newRoute = NaN;
end

if (isNeeded)
    % Add Priority Waypoint next in line in priorityRoute
    newRoute(waypoint_route_index + 1) = waypoint_prio_id;

    % Re-calculate Route-To-Travel (we must first map the ids back to
    % their actual waypoints)
    neededWaypoints = mapWaypoints(route_to_travel, :);
    neededWaypoints(end, :) = [];

    % We now have the order in which they should appear
    [~, priority_optimalRoute, ~] = self_organizing_map(neededWaypoints, mapSize, 100000, 0.8);

    % Re-map priority_optimalRoute back to neededWaypoints
    neededWaypointsOrdered = neededWaypoints(priority_optimalRoute, :);

    neededWaypointsOriginalReferences = zeros(height(neededWaypointsOrdered), 1);
    for i=1:height(neededWaypointsOrdered)
        for j=1:height(mapWaypoints)
            if (mapWaypoints(j, :) == neededWaypointsOrdered(i, :))
                neededWaypointsOriginalReferences(i) = j;
                break;
            end
        end
    end

    % We circular shift the waypoints such that they appear in the correct
    % order
    shiftAmount = find(neededWaypointsOriginalReferences == waypoint_prio_id);
    if (shiftAmount ~= 1)
        neededWaypointsOriginalReferences = circshift(neededWaypointsOriginalReferences, 1-shiftAmount);
    end

    % The route should always loop back to the beginning
    newRoute(waypoint_route_index+1:end-1) = neededWaypointsOriginalReferences(1:end);
    newRoute(end, :) = newRoute(1, :);
end

