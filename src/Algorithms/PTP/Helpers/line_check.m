% input:first point  center point  and the last point , map , scan size;
% output: new point 1 2;

function [new_point1,new_point2]=line_check(front_point,center_point,back_point,map,step_size,obstacles)
	front_step = 0;
	back_step =0;
	step_size_count =1; 			%count the number of the  step

    % Using the center point we attempt to gradually check the line between
    % two points in order to simplify its geometry if possible
	new_pointL =center_point;
	new_pointR =center_point;

	test_pointL =center_point;
	test_pointR =center_point;

    % Determine distances from each side of the center point
	line_front= distanceCost(front_point,center_point);
	line_back= distanceCost(center_point,back_point);

    % Identify the shortest and the longest lengths
	short_len =min(line_front,line_back);
	long_len = max(line_front,line_back);

    % We have k steps to work with in the simplification process
	k =long_len/short_len;

	short_step = short_len/step_size;
	long_step = short_step.*k;

    % Point thetaI in direction to front point
    % Point thetaII in direction to back point
	thetaI =atan2(front_point(1)-center_point(1),front_point(2)-center_point(2));
	thetaII = atan2(back_point(1)-center_point(1),back_point(2)-center_point(2));
	
	% This determines step direction for long and short step
	if line_front == long_len
		front_step =  long_step;
		back_step = short_step;
    elseif  line_front == short_len
		front_step = short_step;
		back_step = long_step;	
    end

    % Loop over gradually testing new points and verifying they're obstacle
    % free. The simplified points are returned once we run out of steps to
    % check
	while step_size_count < step_size ,
		test_pointL = double(center_point(1:2)+(front_step .* step_size_count) * [sin(thetaI)  cos(thetaI)]);
		test_pointR = double(center_point(1:2)+(back_step .* step_size_count) * [sin(thetaII) cos(thetaII)]);		
		if ObstacleFree(test_pointL,test_pointR,map,obstacles),
			new_pointL = test_pointL;
			new_pointR = test_pointR;
		end
		step_size_count = step_size_count+1;
	end
	new_point1 = new_pointL;
	new_point2 = new_pointR;
	
