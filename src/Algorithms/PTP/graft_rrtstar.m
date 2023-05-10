function [path, calculation_time] = graft_rrtstar(source, goal, map_size, occupancy_grid, epsilon, threshold, iterations)
    stepsize=epsilon; 
    disTh=threshold; 
    maxFailedAttempts = iterations;
    rrstar_area = stepsize .* 2;
    center_point = rand_center(source, goal, map_size, occupancy_grid);
    

    % Safety Checks on the algorithm
    if ~CheckCollision(source, occupancy_grid, map_size)
        error('Source Waypoint is occluded by an Obstacle'); 
    end

    if ~CheckCollision(goal, occupancy_grid, map_size)
        error('Goal Waypoint is occluded by an Obstacle');
    end

    static_param_data =[stepsize disTh maxFailedAttempts rrstar_area; source 0 0 ; center_point 0 0 ; goal 0 0];
    %  static_param_data =   stepsize              disTh                     maxFailedAttempts      rrstar_area
    %                        source(1,1)          source(1,2)                point_connect(1,1)     point_connect(1,2)
    %                        center_point(1.1)    center_point(1,2)          prev_ahead             prev_behind
    %                        goal(1,2)            goal(1,2)                  prev_behind_ahead      prev_behing_behind        
	% 	            		 point_connect2(1,1)  point_connect2(1,2)    	
 

    % Initialize Base Values
    global_length = 0;
    local_length = 0;

    % Initalize the Three Trees
    RRTree_main=double([source -1 global_length local_length ]);
    RRTree_goal=double([goal -1 global_length local_length]);
    GraftTree=double([center_point  -1]);

    % These help manage state during the while loop (are placed into the
    % combined donytic_param_data) for easier reference
    Graft_ahead=false;
    Graft_behind=false;
    ahead_found = false;
    behind_found = false;
    errtree_found = false;
    
    donytic_param_data =[Graft_ahead,Graft_behind,ahead_found,behind_found,errtree_found];

    tic;

    % Verify the two points cannot be visibly connected
    if (ObstacleFree(source, goal, map_size, occupancy_grid))
        % They can be, manually create path and return early
        distance = distanceCost(source, goal);
        path = [source(1), source(2), 0, 0];
        path = [path; goal(1), goal(2), distance, distance];
        calculation_time = toc;
        return;
    end

    % We loop while we haven't found/connected all trees
    while  ~donytic_param_data(3) || ~donytic_param_data(4)
        % If we have not connected graft to the goal tree and the goal tree
        % hasn't been found yet
        if ~donytic_param_data(1) && ~donytic_param_data(3)
            % Perform RRT from the Start Tree
            [RRTree_main,static_param_data,donytic_param_data]=g_rstara(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map_size, occupancy_grid);
        end

        % If we have not connected graft to the start tree and the start
        % tree hasn't been found
        if  ~donytic_param_data(2) && ~donytic_param_data(4)
            % Perform RRT from the Goal Tree
            [RRTree_goal,static_param_data,donytic_param_data]=g_rstarb(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map_size, occupancy_grid);
        end
        
        % If we haven't found the goal tree or start tree
        if ~donytic_param_data(3) || ~donytic_param_data(4)
            % Perform RRT from the Graft Tree
            [GraftTree,static_param_data,donytic_param_data]=graft_extend(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map_size, occupancy_grid);
        end
        
        % If we have found goal tree and behind tree
        if donytic_param_data(3) && donytic_param_data(4)
            % Generate Path and exit loop
            in_path = re_path(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data);
            break;
        end
    end

    % This handles compression and expansion phase of Graft-RRT
    in_path=check_globalvalue(in_path,map_size, occupancy_grid);
    out_path=check_globalvalueII(in_path,20,map_size,occupancy_grid);
    in_path=check_globalvalue(out_path,map_size, occupancy_grid);

    % Set output Path to the path from the compression/expansion phase
    path=in_path;
    calculation_time = toc;
end