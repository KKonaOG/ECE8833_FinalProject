function [closestPoint,closestDistance] = GetClosestPoint(origin,canidates)
 % GetClosestPoint takes in a point (origin) and an array of other points
 % (canidates) and returns the index of the canidate that is closest to origin in terms
 % of eucledian distance.

 closestPoint = 1;
 closestDistance = pdist([origin; canidates(1, :)]);
 for i = 1:height(canidates)
    distance = pdist([origin; canidates(i, :)]);
    if (distance < closestDistance)
        closestPoint = i;
        closestDistance = distance;
    end
 end

end

