function in_path = check_globalvalue(path,map,obstacles)  
	stack_path = [path(1,1:2),0,0];  % Stored in the cache tree
	tree_length= 0;  
	stack_length=1;
	queue_path =0; % Queue Head
	step_length=0;
	i_path =1;
	while i_path < length(path)
	   for j_path = i_path:length(path)
		  if ObstacleFree(path(i_path,1:2),path(j_path,1:2),map,obstacles)
			queue_path =j_path;  % Put it in the queue and delete the previous one directly
		  end 
	   end
	  step_length =distanceCost(path(i_path,1:2),path(queue_path,1:2));

      % It is possible for this function to enter an indefinite loop, this
      % catches that case and prevents it by forcing a queue_path increment
      if (step_length == 0)
          warning("Degraded Path - Escaping Loop Entrapment")
          queue_path = queue_path+1;
      end
	  tree_length =step_length+stack_path(stack_length,3);% One round of looping down to detect the length of the stack and the head of the queue
	  i_path=queue_path; % Put into the queue to directly replace the scan of the queue
	  stack_path=[stack_path;[path(i_path,1:2),tree_length,step_length]];
	  stack_length=stack_length+1;
	end
	in_path =stack_path;