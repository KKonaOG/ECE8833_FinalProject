function [mapObst, occupancyMapObst, occupancy_grid_obst] = ObstacleOverride(map_waypoints, hiddenObstaclePolygon, occupancyMap, optimal_route, map, map_size, map_obstacles, occupancy_grid, map_title)
    % Create new Binary Grid
    occupancyMapObst = binaryOccupancyMap(map_size, map_size, 1, "grid");
    occupancy_grid_obst = zeros(map_size, map_size);
    setOccupancy(occupancyMapObst, occupancy_grid_obst);

    % This mimics the system in Maps/MapX.m
    % We are just doing it to get the new obstacle associated with the
    % previous map obstacles and add them to the occupancy map as necessary
    digitalTwinObst = CreateObstacle(map_obstacles, hiddenObstaclePolygon, occupancy_grid);
    for i = 1:map_size
        for j = 1:map_size
            x = map_size - i - 0.5 + 1;
            y = j - 0.5;
            for k = 1:numel(digitalTwinObst)
                isOccupied = inpolygon(x, y, digitalTwinObst(k).Vertices(:, 2), digitalTwinObst(k).Vertices(:, 1));
                if (isOccupied)
                    occupancy_grid_obst(i, j) = 1;
                    break
                end
            end
        end
    end

    % Update Occupancy Map for new Figure
    setOccupancy(occupancyMapObst, occupancy_grid_obst);
    mapObst = figure;
    show(occupancyMapObst);

    % Setup Map Parameters
    title(map_title + " - Obstacle");
    axis ([0 map_size 0 map_size]);
    yticks(0:1:map_size);
    xticks(0:1:map_size);
    axis square;
    grid on;
    hold on;

    % Plots waypoints
    plot(map_waypoints(:,1), map_waypoints(:,2), "o");

    % Re-Label Waypoints
    for i=1:height(optimal_route)
        text(map_waypoints(optimal_route(i), 1), map_waypoints(optimal_route(i), 2), string(i));
    end

    % This re-updates the current map with the new occupancy map
    setOccupancy(occupancyMap, occupancy_grid_obst);
    figure(map);
    show(occupancyMap)
    title(map_title);
    axis ([0 map_size 0 map_size]);
    yticks(0:1:map_size);
    xticks(0:1:map_size);
    axis square;
    grid on;
    hold on;

    % Plots Waypoints
    plot(map_waypoints(:,1), map_waypoints(:,2), "o");

    % Re-Labels Waypoints
    for i=1:height(optimal_route)
        text(map_waypoints(optimal_route(i), 1), map_waypoints(optimal_route(i), 2), string(i));
    end
end

