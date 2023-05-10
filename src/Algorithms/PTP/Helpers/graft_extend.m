function     [GraftTree,static_param_data,donytic_param_data]=graft_extend(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map,obstacles)
% creat param talbe for input data
%    static_param_data=   stepsize              disTh                     maxFailedAttempts      rrstar_area
%                        source(1,1)          source(1,2)               point_connect(1,1)      point_connect(1,2)
%                        center_point(1.1)    center_point(1,2)          prev_ahead             prev-graft-ahead
%                        goal(1,2)            goal(1,2)                  prev_behing	         prev-graft_behind
% 			 point_connectmain(1,1)   point_connectmain(1,2)    	 point_connect2(1,1)    point_connect2(1,2)
%    donytic_param_data =[Graft_ahead,Graft_behind,ahead_found,behind_found];

faildcount =0;


while faildcount <= static_param_data(1,3)

    % Sampling Type is handled based off the currently missing portions of
    % the tree determined by donytic_param_data as a set of states
    if  ~donytic_param_data(1,3) && ~donytic_param_data(1,4)
        if  rand < 0.5
            sample = RandomPoint(0, map);
        elseif rand < 0.75
            sample = [static_param_data(2,1) static_param_data(2,2)];

        else
            sample = [static_param_data(4,1) static_param_data(4,2)];
        end
    end
    if  donytic_param_data(1,4)
        if rand <0.5
            sample = RandomPoint(0, map);
        else
            sample = [static_param_data(2,1) static_param_data(2,2)];
        end
    end
    if  donytic_param_data(1,3)
        if rand <0.5
            sample = RandomPoint(0, map);
        else
            sample = [static_param_data(4,1) static_param_data(4,2)];
        end
    end

    % Find closest point in graft tree to sample point
    [~,I] = min(in_distance(GraftTree(:,1:2),sample),[],1);

    % We now know the closest node in the graft tree
    closestNode = GraftTree(I(1), 1:2);

    % Point the growth direction twoards the newPoint
    theta = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));

    % Generate Graft Tree point from found trajectory (remember this is bound
    % by epsilon)  
    newPoint = double(closestNode(1:2) + static_param_data(1,1) * [sin(theta)  cos(theta)]);

    % If it's not obstacle free, we skip this iteration and move on
    if ~ObstacleFree(closestNode(1:2),newPoint,map,obstacles)
        faildcount = faildcount+1;
        continue;
    end

    % Grab's the closest point in entire Start Tree by index to the new
    % point
    [~,Index_main] = min(in_distance(RRTree_main(:,1:2),newPoint),[],1);
    %    check  two tree  whether link in the map
    %    first  judge is for the  ahead
    %    the second judge is for the behind t
   % This is intended to connect to the two trees (Graft and Start)
    if distanceCost(RRTree_main(Index_main(1),1:2),newPoint)<static_param_data(1,2) && ObstacleFree(RRTree_main(Index_main(1),1:2),newPoint,map, obstacles)
        donytic_param_data(1,2) = false;
        donytic_param_data(1,3) = true;

        static_param_data(2,3) = newPoint(1,1);
        static_param_data(2,4) = newPoint(1,2);

        static_param_data(3,3) = Index_main(1);
        static_param_data(3,4) = I(1);
        break;
    end

    % Grab's the closest point in entire Goal Tree by index to the new
    % point
    [~,Index_goal] = min(in_distance(RRTree_goal(:,1:2),newPoint),[],1);
    %    check  two tree  whether link in the map
    %    first  judge is for the  ahead
    %    the second judge is for the behind t
    % This is intended to connect to the two trees (Graft and Goal)
    if distanceCost(RRTree_goal(Index_goal(1),1:2),newPoint)<static_param_data(1,2) && ObstacleFree(RRTree_goal(Index_goal(1),1:2),newPoint,map, obstacles)
        donytic_param_data(1,1) = false;
        donytic_param_data(1,4) = true;

        static_param_data(5,3) = newPoint(1,1);
        static_param_data(5,4) = newPoint(1,2);

        static_param_data(4,3) = Index_goal(1);
        static_param_data(4,4) = I(1);
        break;
    end

    [~,Index_graft] =  min(in_distance(GraftTree(:,1:2),newPoint),[], 1);

    % We continue the loop if we are not within the distance threshold
    if  in_distance(newPoint, GraftTree(Index_graft(1),1:2)) < static_param_data(1,2)
        faildcount = faildcount+1;
        continue;
    end

    GraftTree = [GraftTree; newPoint  I(1)];
    break;
end

end