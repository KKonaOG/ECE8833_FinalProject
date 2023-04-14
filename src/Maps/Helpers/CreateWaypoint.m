function [outWaypoints] = CreateWaypoint(inWaypoints, newWaypoint)
%CREATEWAYPOINT Create a new waypoint which will be inserted into a
%previously existing waypoint array.
%   Takes in a map's waypoint array and merges in a new waypoint given the
%   waypoint's coordinates

% Merge into Array
outWaypoints = [inWaypoints; newWaypoint];
end

