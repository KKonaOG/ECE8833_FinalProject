% This code was adapted from the MRS Toolbox's mrsDiffDrivePathFollowing.m
% Example

function VFHOnPath(routes, map, digitalTwin)
    warning("off", "all"); % This is done to get rid of a deprecancy warning
    % Sample Time
    sampleTime = 0.1;              % Sample time [s]
    r = rateControl(1/sampleTime); % Allows for the uniform updating of the while loop for display and simulation purposes

    % Create lidar sensor (MRS Toolbox)
    lidar = LidarSensor;
    lidar.sensorOffset = [0,0];
    lidar.scanAngles = linspace(-pi,pi,180);
    lidar.maxRange = 2;
    lidar.mapName = 'occupancyMap'; % Apply occupancy map to LiDAR data so MRS can properly simulate it on the data

    % Create waypoints array for MATLAB NAV Controllers
    waypoints = [];

    for i=1:numel(routes)
        for j=1:height(routes(i).data)
            waypoints = [waypoints; routes(i).data(j, 1:2)];
        end
    end

    % Initial conditions
    initPose = [waypoints(1, 1:2)'; 0]; % Initial pose (x y theta)
    pose = zeros(3, 1);   % Pose matrix
    pose(:,1) = initPose; % Set first pose in pose array as inital

    % Pure Pursuit Controller (Waypoint Following) Configuration
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.LookaheadDistance = 0.40;
    controller.DesiredLinearVelocity = 0.20;
    controller.MaxAngularVelocity = 0.9;

    % Vector Field Histogram (VFH) Configuration
    vfh = controllerVFH;
    vfh.DistanceLimits = [0.05 1.0];
    vfh.NumAngularSectors = 360;
    vfh.HistogramThresholds = [3 8];
    vfh.RobotRadius = 0.1;
    vfh.SafetyDistance = 0.2;

    %% Simulation loop
    idx = 2;
    goalPose = waypoints(end, 1:2);

    while true 
        % Get the sensor readings
        curPose = pose(:,idx-1);
        ranges = lidar(curPose);
            
        % Run the path following and obstacle avoidance algorithms
        [vRef,wRef,lookAheadPt] = controller(curPose);
        targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
        steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
        if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
            wRef = 0.5*steerDir;
        end

        % Control the robot
        velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
        vel = bodyToWorld(velB,curPose);  % Convert from body to world
       
        % Perform forward discrete integration step
        pose(:,idx) = curPose + vel*sampleTime; 
        
        % Update visualization
        waitfor(r);
        idx = idx+1;

        % Rather than relying on the toolbox's built in VizTool, we use
        % the figures we have
        figure(map);

        % This if statement just handle's plotting the different system's
        % path in a different color
        if digitalTwin
            plot(curPose(1), curPose(2), 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', 'r', 'MarkerSize', 1);
        else
            plot(curPose(1), curPose(2), 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', 'b', 'MarkerSize', 1);
        end

        % Exit once we are within 0.5 units of the final goal point
        % idx > 1000 (etc.) might need to be adjusted depending on the
        % specific map and route taken
        if (idx > 1000 && sqrt((goalPose(1)-curPose(1)).^2 + (goalPose(2)-curPose(2)).^2 ) < 0.25)
            disp("Done!")
            break;
        end
    end
    warning("on", "all");
end

