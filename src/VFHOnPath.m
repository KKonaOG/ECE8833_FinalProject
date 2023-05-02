function VFHOnPath(currentRoute, mapObstacles, mapFigure)
    VFH = controllerVFH;
    VFH.UseLidarScan = true;
    VFH.MinTurningRadius = 0;
    VFH.RobotRadius = 0.1;
    VFH.NumAngularSectors = 360;
    VFH.DistanceLimits = [0.05 0.5];
    h = figure;
    steeringDirection = 0;
    for i=1:height(currentRoute)-1
        goalPathPosition = currentRoute(i+1, 1:2);

        % This will be updated by the VFH loop
        currentPathPosition = currentRoute(i, 1:2);
        
        goalNotReached = true;
        while (goalNotReached)
            figure(mapFigure);
            % First Calculate Target Direction
            deltaX = goalPathPosition(1) - currentPathPosition(1);
            deltaY = goalPathPosition(2) - currentPathPosition(2);
            targetDirection = atan2(deltaY, deltaX) - pi/2;

            % Generate Simulated LiDAR Scan
            numberScanPoints = 360;
            angleIncrement = 360/numberScanPoints;
            angles = 0:angleIncrement:359;
            scanRadius = 1;
            ranges=zeros(numberScanPoints, 1);
            for j = 1:numberScanPoints
                angle = deg2rad(angles(j));
                end_point = currentPathPosition + scanRadius * [cos(angle), sin(angle)];
                closest_range = 256;
                for k = 1:numel(mapObstacles)
                    % Check if line segment intersects with obstacle polygon
                    [int_x, int_y] = polyxpoly([currentPathPosition(1) end_point(1)], [currentPathPosition(2) end_point(2)],mapObstacles(k).Vertices(:, 1), mapObstacles(k).Vertices(:, 2));
                    if ~isempty(int_x)
                        % If intersection exists, compute range to intersection point
                        range = norm([int_x(1), int_y(1)] - currentPathPosition);
                        if range < closest_range
                            closest_range = range;
                        end
                    else
                        % If no intersection, check if end point is inside obstacle polygon
                        if inpolygon(end_point(1), end_point(2), mapObstacles(k).Vertices(:, 1), mapObstacles(k).Vertices(:, 2))
                            range = norm(end_point - currentPathPosition);
                            if range < closest_range
                                closest_range = range;
                            end
                        end
                    end
                end
                ranges(j) = closest_range;
            end
            
            
            % Create Scan Object from the Simulated Data
            scan = lidarScan(ranges, angles);
         
            steeringDirectionNew = VFH(scan, targetDirection);
            if (isnan(steeringDirectionNew))
                steeringDirection = steeringDirection + 0.5;
            else
                steeringDirection = steeringDirectionNew;
                translationDistance = min(norm(goalPathPosition - currentPathPosition), 0.5);
            end


            translationX = translationDistance * cos(steeringDirection + pi/2);
            translationY = translationDistance * sin(steeringDirection + pi/2);

            currentPathPosition = currentPathPosition + [translationX, translationY];
            
            plot(currentPathPosition(1), currentPathPosition(2), 'go', 'MarkerSize', 8, 'LineWidth', 0.1);

            % Check if goal has been reached
            if norm(goalPathPosition - currentPathPosition) < 0.1
                goalNotReached = false;
            end
            pause(0.1);
            figure(h);
            show(VFH);
            pause(0.1);
        end
    end
end

