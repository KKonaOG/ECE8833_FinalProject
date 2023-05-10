function     new_path = re_path(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data)
	% creat param talbe for input data
	%    static_param_data=   stepsize              disTh                     maxFailedAttempts      rrstar_area
	%                        source(1,1)          source(1,2)               point_connect(1,1)      point_connect(1,2)
	%                        center_point(1.1)    center_point(1,2)          prev_ahead             prev-graft-ahead
	%                        goal(1,2)            goal(1,2)                  prev_behing	         prev-graft_behind  
	%    donytic_param_data		 point_connectmain(1,1)   point_connectmain(1,2)    	 point_connect2(1,1)    point_connect2(1,2)
	%    donytic_param_data =[Graft_ahead,Graft_behind,ahead_found,behind_found,errtree_found];
	main_tree=[];
	graft_tree_ahead=[];
	graft_tree_behind = [];
	goal_tree=[];

    % If errtree_found is true generate the tree path using the main and
    % goal trees based off the prev_ahead and prev_graft_behind point
	if donytic_param_data(5),
		prev = static_param_data(3,3);
		main_tree = [static_param_data(5,1) static_param_data(5,2)]; % Connection Point
		while prev > 0,
			main_tree = [RRTree_main(prev,1:2);main_tree]; % Grab path to Connection Point
			prev = RRTree_main(prev,3);
		end
		prev =static_param_data(4,4);
		goal_tree = [static_param_data(5,1) static_param_data(5,2)]; % Connection Point
		while prev > 0,
			goal_tree = [goal_tree;RRTree_goal(prev,1:2)]; % Grab path to connection point
			prev = RRTree_goal(prev,3);
		end	
		main_tree(end,:)=[]; % we don't want the end since it is just connection point and that is already in goal tree
		new_path=[main_tree;goal_tree ]; % Combine the paths
		
    else
        % If errtree_found is not true then the tree path is generated from
        % the connect points
		prev = static_param_data(3,3);
		main_tree =[static_param_data(2,3) static_param_data(2,4)];
	    
        % Recursively build_main tree from prev to get path from connect
        % point
		while prev > 0,
			main_tree = [RRTree_main(prev,1:2);main_tree];
			prev = RRTree_main(prev,3);
		end

        % Repeat the same process for the goal tree
		prev = static_param_data(4,3);
		goal_tree =[static_param_data(5,3) static_param_data(5,4)];
		while prev > 0,
			goal_tree = [goal_tree;RRTree_goal(prev,1:2)];
			prev = RRTree_goal(prev,3);		
		end


        % Repeat the same process for the graft tree (we have to do it in
        % two places though since it is connected on both sides of the
        % tree)
		graft_path_ahead =  [static_param_data(2,3) static_param_data(2,4)];
		prev =  static_param_data(3,4);
		
		while    prev >0,
			graft_path_ahead = [graft_path_ahead;GraftTree(prev,1:2)];
			 prev = GraftTree(prev,3);
		end

		graft_path_behind = [static_param_data(5,3) static_param_data(5,4)];
		prev = static_param_data(4,4);

		while  	prev  >0,
			graft_path_behind = [GraftTree(prev,1:2);graft_path_behind];
			prev = GraftTree(prev,3);
        end

        % Connect Start Tree, Graft Tree, and Goal Tree form one full
        % uniform path

		main_tree(end,:)=[];
        % Remove connect points on graft tree as the Start and Goal trees
        % will contain those for us
		graft_path_ahead(end,:)=[];	
		graft_path_behind(end,:)=[];
		new_path = [main_tree; graft_path_ahead;graft_path_behind;goal_tree];

			
    end