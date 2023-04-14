function [network, optimalRoute, routeDistance] = self_organizing_map(map_waypoints, map_size, iterations, learning_rate)
% <h1><a href="../../../README.cchtml#h_907645437334861681190082514">SOM
%         (Self-Organizing Map)</a></h1>
% <p>The following code leverages a neural network to determine sub-optimal
%     solutions for the Traveling Salesman Problem. This code was adapted from:
%     <a
%         href="https://github.com/diego-vicente/som-tsp">https://github.com/diego-vicente/som-tsp</a>
% </p>
% <h2>Function Inputs:</h2>
% <table style="border-collapse: collapse; width: 43.499%; height: 5px;"
%     border="1">
%     <colgroup>
%         <col style="width: 50.0006%;">
%         <col style="width: 50.0006%;">
%     </colgroup>
%     <tbody>
%         <tr>
%             <td>Parameter Name</td>
%             <td>Parameter Purpose</td>
%         </tr>
%         <tr>
%             <td>Map Waypoints (map_waypoints)</td>
%             <td>A list of waypoint locations in no particular order to operate
%                 on.</td>
%         </tr>
%         <tr>
%             <td>Map Size (map_size)</td>
%             <td>The size of the map being operated on. This is used in the
%                 normalization procedure.</td>
%         </tr>
%         <tr>
%             <td>Iterations</td>
%             <td>The maximum of iterations to use when calculating the path.
%                 The function can return earlier if either the number of active
%                 neurons decays entirely or the learning rate decays entirely.
%             </td>
%         </tr>
%         <tr>
%             <td>Learning Rate</td>
%             <td>Specifies the initial learning rate (percentage of neurons) to
%                 use when training.</td>
%         </tr>
%     </tbody>
% </table>
% <h3>Function Outputs:</h3>
% <table style="border-collapse: collapse; width: 43.8865%; height: 5px;"
%     border="1">
%     <colgroup>
%         <col style="width: 50%;">
%         <col style="width: 50%;">
%     </colgroup>
%     <tbody>
%         <tr>
%             <td>Output Name</td>
%             <td>Output Purpose</td>
%         </tr>
%         <tr>
%             <td>Network</td>
%             <td>This is the final raw node network calculated by the
%                 Self-Organizing Map function.</td>
%         </tr>
%         <tr>
%             <td>Optimal Route</td>
%             <td>The waypoints in the optimal order produced by the
%                 Self-Organizing Map function.</td>
%         </tr>
%         <tr>
%             <td>Route Distance</td>
%             <td>The distance of the optimal route produced by the
%                 Self-Organizing Map function.</td>
%         </tr>
%     </tbody>
% </table>
% <p>&nbsp;</p>

% <h2>Stage 1: Normalize Waypoints</h2>
% <p>Waypoints are normalized based off the map size (i.e, the bounds of the
%     waypoints will not exceed 0..1). Alternative methods are available, such
%     as making the greatest waypoint coordinate mark as 1.</p>
map_waypoints_normalized = map_waypoints/map_size;

% <h2>Stage 2: Generate the Initial Network</h2>

% <p>Neurons are dynamically scaled with the number of waypoints present on the
%     map. The constant multiplier can be adjusted to fine-tune the network.</p>
activeNeurons = height(map_waypoints_normalized) * 8;
networkNeurons = rand(activeNeurons, 2);

% <h2>Stage 3: Train the Network</h2>
for i = 1:iterations
     % <h3>Stage 3a: Sample a Random Waypoint</h3>
     waypoint = map_waypoints_normalized(randperm(height(map_waypoints_normalized), 1), :);

     % <h3>Stage 3b: Find Closest ("Winning") Neuron</h3>
     % <p>Its index and value are saved to be used in the gaussian calculation,
     %     the winning neuron defaults to being the first element in the
     %     generated network (networkNeurons).<br>This can potentially be
     %     functionalized/optimized by moving it to a function and finding a way
     %     to apply <a
     %         href="https://www.mathworks.com/help/stats/pdist.html">pdist
     %     </a>to two different arrays.</p>
     closestNeuron_Index = 1;
     closestNeuron_Distance = pdist([waypoint; networkNeurons(1, :)]);
     for j = 2:height(networkNeurons)
         distance = pdist([waypoint; networkNeurons(j, :)]);
         if (distance < closestNeuron_Distance)
             closestNeuron_Index = j;
             closestNeuron_Distance = distance;
         end
     end

     % <h3>Stage 3c: Apply Gaussian Regression</h3>
     % <p>Calculate a gaussian based off closest neuron, this will be applied to
     %     the rest of the network as a way to update their weights. This
     %     methodology allows for a greater effect on points closer to the
     %     closest point with dwindling affects as the distance to the closest
     %     point increases. <br>Radius (radix) is determined by the
     %     "activeNeurons", and is labeled this way primarily since the number
     %     is based off the number of neurons at start and it decays to 0. In
     %     many ways this resembles "drop out" seen in many Neural Networks such
     %     as CNNs.</p>
        
     % <p>Center -- Closest Neuron Index<br>Radix -- Population Size divided by
     %     10 (and rounded down) <br>Domain -- Number of Neurons</p>
     
     center = closestNeuron_Index;
     domain = height(networkNeurons);
     radix = floor(activeNeurons/10);

     % <p>This is used to prevent NaNs from populating into the Network</p>
     if (radix < 1)
            radix = 1;
     end

     deltas = abs(center - (0:1:domain-1));
     distances = min(deltas, (domain-deltas));
    
     % <p>Calculate Guassian</p>
     gaussian = exp(-(distances.*distances) / (2*(radix*radix)));
    
     % <p>Apply Gaussian Filter to the Network</p>
     networkNeurons = networkNeurons + ((gaussian(:) .* ([(waypoint(:, 1) - networkNeurons(:, 1)) (waypoint(:, 2) - networkNeurons(:, 2))])) * learning_rate);

     % <h3>Stage 3d: Decay Learning Parameters</h3>
     learning_rate = learning_rate * 0.99997;
     activeNeurons = activeNeurons * 0.9997;

     % <p>If the radius (or amount of activeNeurons) has completely decayed we
     %     can go ahead and exit out as we can no longer learn.</p>
     if (activeNeurons < 1)
         break;
     end

     % <p>If the learning rate has completely decayed we can go ahead and exit
     %     out as we can no longer learn.</p>
     if (learning_rate < 0.001)
         break;
     end
    
end


 % <h3>Stage 4: Build Calculated Route</h3>
 % <p>Unlike in a normal SOM, the output of our training is the data we want. We
 %     identify the waypoint order by finding the "winning" neurons which are
 %     the neurons closest to each waypoint. This list of winning neurons can be
 %     found in order and the order in which they appear is the "optimal"
 %     waypoint route found by the SOM.<br>This portion of the code leverages
 %     the <a href="Helpers/GetClosestPoint.m">GetClosestPoint.m</a> helper
 %     function to quickly calculate the closest point in the network neurons to
 %     the specific map waypoint provided.</p>

 % <p>For each city, find the closest neuron, and mark it as the winning neuron
 % </p>
optimalRoute = [];
for i = 1:height(map_waypoints_normalized)
    [bestRoutePoint, ~] = GetClosestPoint(map_waypoints_normalized(i,:), networkNeurons);
    optimalRoute = [optimalRoute; [i bestRoutePoint]];
end
optimalRoute = sortrows(optimalRoute, 2);
optimalRoute = optimalRoute(:, 1);

% <h4>Stage 4b: Calculate Path Cost (Optional) | De-Normalize Data
%     (Non-Optional)</h4>
routeDistance = 0;
for i = 1:height(optimalRoute)-1
   [~, distance] = GetClosestPoint([map_waypoints(optimalRoute(i), 1), map_waypoints(optimalRoute(i), 2)], [map_waypoints(optimalRoute(i+1), 1), map_waypoints(optimalRoute(i+1), 2)]); 
   routeDistance = routeDistance + distance;
end

network = networkNeurons*map_size;
end

