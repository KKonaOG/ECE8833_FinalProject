function [containsValid, validPoints] = CheckCollision(points, occupancy_grid, map_size)
% CheckCollision The purpose function is dual purpose. It is used within
% pre-processing to determine what points (if any) in the array of points
% are valid. The returns are containsValid and validPoints. containsValid
% is a quick check (implemented primarily for ease of use when checking a
% singular point) and validPoints is an array of all points (inside of the passed in
% points) that are not in an obstacle or outside the map.

% Remove Points Exceeding X Bounds ( > map_size)
invalidMapBoundsX = points(:, 1) >= map_size;
points(invalidMapBoundsX, :) = [];

% Removing Points Exceeding X Bounds (< 0)
invalidMapBoundsX = points(:, 1) <= 0;
points(invalidMapBoundsX, :) = [];


% Remvoing Points Exceeding Y Bounds (> map_size)
invalidMapBoundsY = points(:, 2) >= map_size;
points(invalidMapBoundsY, :) = [];

% Removing Points Exceeding X Bounds (< 0)
invalidMapBoundsY = points(:, 2) <= 0;
points(invalidMapBoundsY, :) = [];

% Remove Points within Obstacles
invalidPoints = zeros(height(points), 1);
for i=1:height(points)
    pointX = floor(points(i, 1) + 1);
    pointY = map_size - floor(points(i, 2));

    if (occupancy_grid(pointY, pointX) == 1)
        invalidPoints(i) = 1;
    end
end
points = points(~invalidPoints, :);

containsValid = height(points)>=1;
validPoints = points;

end