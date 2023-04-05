function [network, optimalRoute, routeDistance] = self_organizing_map(map_waypoints, map_size, iterations, learning_rate)
% SOM (Self-Organizing Map) leverages a neural network to determine
% sub-optimal solutions for the Traveling Salesman Problem. MATLAB code was
% adapted from: https://github.com/diego-vicente/som-tsp

%% Normalize Waypoints
% Waypoints are normalized based off the map size (i.e, the bounds of the
% waypoints will not exceed 0..1). Alternative methods are available, such
% as making the greatest waypoint coordinate mark as 1.
map_waypoints_normalized = map_waypoints/map_size;

%% Generate Network

% Neurons will be dynamically scaled with the number of waypoints present
% on the map. The constant multiplier can be adjusted to fine-tune the
% network.
activeNeurons = height(map_waypoints_normalized) * 8;
networkNeurons = rand(activeNeurons, 2);

%% "Train" the Network
for i = 1:iterations
     % Sample a Random Waypoint
     waypoint = map_waypoints_normalized(randperm(height(map_waypoints_normalized), 1), :);

     % Find Closest Neuron (save its index and value), defaults to being
     % the first element in the networkNeurons array. This can potentially
     % be functionalized/optimized by moving it to a function and finding a
     % way to apply pdist to two different arrays.
     closestNeuron_Index = 1;
     closestNeuron_Distance = pdist([waypoint; networkNeurons(1, :)]);
     for j = 2:height(networkNeurons)
         distance = pdist([waypoint; networkNeurons(j, :)]);
         if (distance < closestNeuron_Distance)
             closestNeuron_Index = j;
             closestNeuron_Distance = distance;
         end
     end

     % Calculate a gaussian based off closest neuron, this will be applied
     % to the rest of the network as a way to update their weights. This
     % priorities points within a neighborhood to the closest point with
     % dwindling affects as the distance to the closest point increases.
     % Radius (radix) is determined by the "activeNeurons", labeled this
     % way primarily since the number is based off the number of neurons at
     % start at decays to 0. In many ways this is resemblent of a type of
     % "drop out" for neurons that are further away from the source neuron.
        
     % Center -- Closest Neuron Index
     % Radix -- Population Size divided by 10 (and rounded down)
     % Domain -- Number of Neurons
     
     center = closestNeuron_Index;
     domain = height(networkNeurons);
     radix = floor(activeNeurons/10);

     % This is used to prevent NaNs from populating into the Network
     if (radix < 1)
            radix = 1;
     end

     deltas = abs(center - (0:1:domain-1));
     distances = min(deltas, (domain-deltas));
    
     % Calculate Guassian
     gaussian = exp(-(distances.*distances) / (2*(radix*radix)));
    
     % Apply Gaussian Filter to the Network
     networkNeurons = networkNeurons + ((gaussian(:) .* ([(waypoint(:, 1) - networkNeurons(:, 1)) (waypoint(:, 2) - networkNeurons(:, 2))])) * learning_rate);

     % Decay Learning Parameters
     learning_rate = learning_rate * 0.99997;
     activeNeurons = activeNeurons * 0.9997;

     % If the radius (or amount of activeNeurons) has completely decayed we
     % can go ahead and exit out as we can no longer learn.
     if (activeNeurons < 1)
         break;
     end

     % If the learning rate has completely decayed we can go ahead and exit out as we can no longer learn.
     if (learning_rate < 0.001)
         break;
     end
    
end


 % Build the route, we only want the "best" neurons. Imagine this as
 % knowing where to go giveb a specific waypoint. The TSP is usually
 % understood as cities, however, in our case these waypoints can be seen
 % as robot goal positions with one waypoint being the starting position.

 % For each city, find the closest neuron, and mark it as the winning
 % neuron
optimalRoute = [];
for i = 1:height(map_waypoints_normalized)
    [bestRoutePoint, ~] = GetClosestPoint(map_waypoints_normalized(i,:), networkNeurons);
    optimalRoute = [optimalRoute; [i bestRoutePoint]];
end
optimalRoute = sortrows(optimalRoute, 2);
optimalRoute = optimalRoute(:, 1);

% Final Step, calculate our "best route" cost
routeDistance = 0;
for i = 1:height(optimalRoute)-1
   [~, distance] = GetClosestPoint([map_waypoints(optimalRoute(i), 1), map_waypoints(optimalRoute(i), 2)], [map_waypoints(optimalRoute(i+1), 1), map_waypoints(optimalRoute(i+1), 2)]); 
   routeDistance = routeDistance + distance;
end

network = networkNeurons*map_size;
end

