function     [RRTree_goal,static_param_data,donytic_param_data]=g_rstarb(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map,obstacles)
	% creat param talbe for input data
	%    static_param_data=   stepsize              disTh                     maxFailedAttempts      rrstar_area
	%                        source(1,1)          source(1,2)               point_connect(1,1)      point_connect(1,2)
	%                        center_point(1.1)    center_point(1,2)          prev_ahead             prev-graft-ahead
	%                        goal(1,2)            goal(1,2)                  prev_behing	         prev-graft_behind  
	% 			 point_connectmain(1,1)   point_connectmain(1,2)    	 point_connect2(1,1)    point_connect2(1,2)
	%    donytic_param_data =[Graft_ahead,Graft_behind,ahead_found,behind_found];
	faildcount =0;	

    % Loop until iteration count is reached
    while faildcount <= static_param_data(1,3)

        % Get the sample point based off rules specified by Graft-RRT
        % algorithm
		if  rand < 0.5
			sample = RandomPoint(0, map);
		elseif rand < 0.75
			sample = [static_param_data(3,1) static_param_data(3,2)];
	    else 
			sample = [static_param_data(2,1) static_param_data(2,2)];
		end
    

        % This is a method for acquiring Q_new that is faster than the for loop method 
	    buffer_distance =  in_distance(RRTree_goal(:,1:2),sample);
	    I(1) = check_loacalvalue(buffer_distance,RRTree_goal,static_param_data(1,4));
	    
        % 0 is not allowed so we just quickly set it to 1 (indexing)
	    if I(1) == 0
		    I(1) =1;
        end

        % We now know the closest node in the goal tree
	    closestNode = RRTree_goal(I(1), 1:2);
    
        % Point the growth direction twoards the newPoint
	    theta = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));
    
        % Generate Goal Tree point from found trajectory (remember this is bound
        % by epsilon)
	    newPoint = double(closestNode(1:2) + static_param_data(1,1) * [sin(theta)  cos(theta)]);
    
        % If it's not obstacle free, we skip this iteration and move on
	    if ~ObstacleFree(closestNode(1:2),newPoint,map,obstacles)
		    faildcount = faildcount+1;
		    continue
	    end
	    
        % Grab's the closest point in entire Graft Tree by index to the new
        % point
	    [~,Index_graft] = min(in_distance(GraftTree(:,1:2),newPoint),[],1);
    
        % This is intended to connect to the two trees (Graft and Goal)
	    if distanceCost(GraftTree(Index_graft(1),1:2),newPoint)<static_param_data(1,2) && ObstacleFree(GraftTree(Index_graft(1),1:2),newPoint,map,obstacles) 
		    donytic_param_data(1,2) = false; 	
		    donytic_param_data(1,4) = true;
		    
		    static_param_data(5,3) = newPoint(1,1);
		    static_param_data(5,4) = newPoint(1,2);
		    
		    static_param_data(4,3) = I(1);
		    static_param_data(4,4) = Index_graft(1);
		    break;
        end

        % check the start tree with the goal tree state;
        [~,Index_goal] = min(in_distance(RRTree_main(:,1:2),newPoint),[],1); % Whether it is optimized here depends on the phenomenon, the memory usage is too high
        
        % Same as above but for Start Tree
        if distanceCost(RRTree_main(Index_goal(1),1:2),newPoint)<static_param_data(1,2) && ObstacleFree(RRTree_main(Index_goal(1),1:2),newPoint,map,obstacles) 
		    donytic_param_data(1,2) = false; 
		    donytic_param_data(1,3) = true;
		    donytic_param_data(1,4) = true;
		    donytic_param_data(1,5) = true;
    
		    static_param_data(5,1) = newPoint(1,1);
		    static_param_data(5,2) = newPoint(1,2);
		    
		    static_param_data(4,3) = I(1);
		    static_param_data(3,3) = Index_goal(1);
		    break;
        end

        
	    [Distance_goal,DIndex_goal] =  min( in_distance(RRTree_goal(:,1:2),newPoint),[], 1);
    
        % We continue the loop if we are not within the distance threshold
	    if  in_distance(newPoint, RRTree_goal(DIndex_goal(1),1:2)) < static_param_data(1,2)
		    faildcount = faildcount+1;
		    continue;
	    end
	    
	    buffer_local = Distance_goal;
	    buffer_global = RRTree_goal(I(1),4) + buffer_local;
    
	    RRTree_goal = [RRTree_goal;newPoint I(1) buffer_global  buffer_local];
	    
	    donytic_param_data(1,2) = false; 
	    
	    break;
    end
end
    