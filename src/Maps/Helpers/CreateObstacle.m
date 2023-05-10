function [outObstacles, occupancy_map] = CreateObstacle(inObstacles, inVertices, occupancy_map)
%CREATEOBSTACLE Create a new obstacle which will be inserted into a
%previously existing obstacle array.
%   Takes in a map's obstacle array and merges in a new polyshape given the
%   obstacle's verticies

% Create new Polyshape
newObstacle = polyshape(inVertices);

% Merge into Array
outObstacles = [inObstacles; newObstacle];
end

