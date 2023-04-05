function [path, calculation_time] = graft_rrtstar(source, goal, map_size, obstacles, epsilon, threshold, iterations)
    stepsize=epsilon; 
    disTh=threshold; 
    maxFailedAttempts = iterations;
    rrstar_area = stepsize .* 2;
    center_point = rand_center(source, goal, map_size, obstacles);

    if ~CheckCollision(source, obstacles, map_size)
        error('false '); 
    end

    if ~CheckCollision(goal, obstacles, map_size)
        error('false ');
    end

    static_param_data =[stepsize disTh maxFailedAttempts rrstar_area; source 0 0 ; center_point 0 0 ; goal 0 0];
    % creat param talbe for input data
    %  static_param_data =   stepsize              disTh                     maxFailedAttempts      rrstar_area
    %                        source(1,1)          source(1,2)                point_connect(1,1)      point_connect(1,2)
    %                        center_point(1.1)    center_point(1,2)          prev_ahead             prev_behind
    %                        goal(1,2)            goal(1,2)                  prev_behind_ahead      prev_behing_behind        
	    % 	            		 point_connect2(1,1)   point_connect2(1,2)    	
   
    % check point
    %imshow(map);
    % if display,
    %         imshow(map);
    %         rectangle('position',[1 1 size(map)-1],'edgecolor','k'); 
    % end
 
    global_length = 0;
    local_length = 0;
    RRTree_main=double([source -1 global_length local_length ]);
    RRTree_goal=double([goal -1 global_length local_length]);
    GraftTree=double([center_point  -1]);

    Graft_ahead=false;
    Graft_behind=false;
    ahead_found = false;
    behind_found = false;
    errtree_found = false;
    
    donytic_param_data =[Graft_ahead,Graft_behind,ahead_found,behind_found,errtree_found];

    tic;
    % Verify the two points cannot be visibly connected
    if (ObstacleFree(source, goal, map_size, obstacles))
        % They can be, manually create path and return early
        distance = distanceCost(source, goal);
        path = [source(1), source(2), 0, 0];
        path = [path; goal(1), goal(2), distance, distance];
        calculation_time = toc;
        return;
    end


    while  ~donytic_param_data(3) || ~donytic_param_data(4)
        if ~donytic_param_data(1) && ~donytic_param_data(3)
            [RRTree_main,static_param_data,donytic_param_data]=g_rstara(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map_size, obstacles);
        end

        if  ~donytic_param_data(2) && ~donytic_param_data(4)
            [RRTree_goal,static_param_data,donytic_param_data]=g_rstarb(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map_size, obstacles);
        end

        if ~donytic_param_data(3) || ~donytic_param_data(4)
            [GraftTree,static_param_data,donytic_param_data]=graft_extend(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map_size, obstacles);
        end
        
        if donytic_param_data(3) && donytic_param_data(4)
            in_path = re_path(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data);
            break;
        end
    end
    in_path=check_globalvalue(in_path,map_size, obstacles);
    out_path=check_globavalueII(in_path,20,map_size,obstacles);
    in_path=check_globalvalue(out_path,map_size, obstacles);
    path=in_path;
    calculation_time = toc;
end