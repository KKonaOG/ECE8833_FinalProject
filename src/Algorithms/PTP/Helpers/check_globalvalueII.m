function  out_path= check_globalvalueII(in_path,step_size,map,obstacles)
	
	
	buffer_tree =in_path(:,1:2); % input the path of the final path 
	buffer_tree(:,3:6)=0;
	 %set  two tree  which is the old path's children , 
	 %the left children is  buffer_tree(:,3:4) . the right children is buffer_tree(:,5:6)
	 %   what we want found  path is the riht children  and left children 
	 %
	 buffer_tree(1,3:4)=buffer_tree(1,1:2);
	% buffer_tree(length(buffer_tree)-1,5:6)=buffer_tree(length(buffer_tree),1:2);
	i_path = 2;
	buffer_point1=[0,0];
	buffer_point2=[0,0];
	new_length=0;
	[l,c]=size(buffer_tree);
	while i_path <=(l-1),
        % Generates new point
		[L,R] = line_check(buffer_tree((i_path-1),3:4),buffer_tree(i_path,1:2),buffer_tree((i_path+1),1:2),map,step_size,obstacles);
		buffer_tree(i_path-1,5:6)=L;
		buffer_tree(i_path,3:4)=R;		
		i_path = i_path+1;
	end
	buffer_tree(l-1,5:6)=buffer_tree(l,1:2);
	 new_tree=[ buffer_tree(1,3:4);buffer_tree(1,5:6)];
	for index = 2:l-1
		new_tree = [new_tree;buffer_tree(index,3:4);buffer_tree(index,5:6)];
	end
	out_path = new_tree;