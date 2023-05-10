function [network, optimalRoute, routeDistance] = self_organizing_map(map_waypoints, map_size, iterations, learning_rate)
%% Stage 1: Generate the Initial Network</h2>

% Neurons are dynamically scaled with the number of waypoints present on the
% map. The constant multiplier can be adjusted to fine-tune the network.
activeNeurons = height(map_waypoints) * 8;
networkNeurons = rand(activeNeurons, 2);

% In order to early terminate for dwindling improvements, a relationship must be 
% made between the closest neuron to a waypoint and its distance. This in
% effect should cause the program to terminate when the rate of improvement
% begins to dwindle.
error_array = zeros(height(map_waypoints), 2);
error_array(:, 2) = map_size*map_size;
%% Stage 2: Train the Network
for i = 1:iterations
     error_array(:, 1) = error_array(:, 2);

     % Stage 2a: Sample In Waypoint Order
     waypoint_index = mod(i-1, height(map_waypoints)) + 1;
     waypoint = map_waypoints(waypoint_index, :);

     % Stage 2b: Find Closest ("Winning") Neuron
     % Its index and value are saved to be used in the gaussian calculation,
     % the winning neuron defaults to being the first element in the
     % generated network (networkNeurons). This can potentially be
     % functionalized/optimized by moving it to a function and finding a way
     % to apply pdist to two arrays.

     closestNeuron_Index = 1;
     closestNeuron_Distance = pdist([waypoint; networkNeurons(1, :)]);
     for j = 2:height(networkNeurons)
         distance = pdist([waypoint; networkNeurons(j, :)]);
         if (distance < closestNeuron_Distance)
             closestNeuron_Index = j;
             closestNeuron_Distance = distance;
         end
     end

     % Stage 2c: Apply Gaussian Regression
     % Calculate a gaussian based off closest neuron, this will be applied to
     % the rest of the network as a way to update their weights. This
     % methodology allows for a greater effect on points closer to the
     % closest point with dwindling affects as the distance to the closest
     % point increases. Radius (radix) is determined by the
     % "activeNeurons", and is labeled this way primarily since the number
     % is based off the number of neurons at start and it decays to 0. In
     % many ways this resembles "drop out" seen in many Neural Networks such
     % as CNNs.
      
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

     % Stage 2d: Decay Learning Parameters
     learning_rate = learning_rate * 0.99997;
     activeNeurons = activeNeurons * 0.9997;
    
     % If the radius (or amount of activeNeurons) has completely decayed we
     % can go ahead and exit out as we can no longer learn.
     if (activeNeurons < 1)
         break;
     end

     % If the learning rate has completely decayed we can go ahead and exit
     % out as we can no longer learn.
     if (learning_rate < 0.001)
         break;
     end

     % Stage 2e: Early Termination Clause for Real-time Priortization
     error_array(waypoint_index, 2) = pdist([waypoint; networkNeurons(closestNeuron_Index, :)]);
     if (((mean(error_array(:,1) - mean(error_array(:, 2)))) < 0.001) && mean(error_array(:, 2) < 0.001))
        break;
     end
end


%% Stage 3: Build Calculated Route
% Unlike in a normal SOM, the output of our training is the data we want. We
% identify the waypoint order by finding the "winning" neurons which are
% the neurons closest to each waypoint. This list of winning neurons can be
% found in order and the order in which they appear is the "optimal"
% waypoint route found by the SOM.<br>This portion of the code leverages
% the GetClosestPoint helper function to quickly calculate the closest 
% point in the network neurons to the specific map waypoint provided.

% For each city, find the closest neuron, and mark it as the winning neuron
optimalRoute = [];
for i = 1:height(map_waypoints)
    [bestRoutePoint, ~] = GetClosestPoint(map_waypoints(i,:), networkNeurons);
    optimalRoute = [optimalRoute; [i bestRoutePoint]];
end
optimalRoute = sortrows(optimalRoute, 2);
optimalRoute = optimalRoute(:, 1);

%% Stage 4: Calculate Path Cost (Optional)
routeDistance = 0;
for i = 1:height(optimalRoute)-1
   [~, distance] = GetClosestPoint([map_waypoints(optimalRoute(i), 1), map_waypoints(optimalRoute(i), 2)], [map_waypoints(optimalRoute(i+1), 1), map_waypoints(optimalRoute(i+1), 2)]); 
   routeDistance = routeDistance + distance;
end

network = networkNeurons;
end

