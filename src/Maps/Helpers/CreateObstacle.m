function [outObstacles, outBuffers] = CreateObstacle(inObstacles, inBuffers, inVertices)
%CREATEOBSTACLE Create a new obstacle which will be inserted into a
%previously existing obstacle array.
%   Takes in a map's obstacle array and merges in a new polyshape given the
%   obstacle's verticies

% Create new Polyshape
newObstacle = polyshape(inVertices);

% Simplify Polyshape
% newObstacle = convhull(newObstacle);

% Create new Polybuffer
newBuffer = polybuffer(newObstacle, 0.25, "JointType", "miter");


% Merge into Array
outObstacles = [inObstacles; newObstacle];
outBuffers = [inBuffers; newBuffer];
end

