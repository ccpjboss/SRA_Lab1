

%
% TurtleBot control class 
%
%       Autonomous Robotic Systems 2022, DEEC, UC 
%
% versions:
% v01 - initial release 

classdef TurtleBot

properties
      velMsg    % velocity message
      robotCmd  % robot velocity command 
      odomSub   % odometry subscriber 
      lidarSub  % lidar subscriber 
   end

   methods

    % Constructor
    function obj = TurtleBot()
        
        % -> REQUIRES MANUALLY SETTING THE IP ADDRESSES 
        IP_OF_TURTLEBOT = "192.168.99.130";              % IP OF VIRTUAL MACHINE or TURTLE ROBOT
        IP_OF_HOST_COMPUTER = "10.101.227.59";          % LOCAL IP

        % Init ROS 
        rosinit(IP_OF_TURTLEBOT, "NodeHost", IP_OF_HOST_COMPUTER);

        % create a publisher for the /cmd_vel topic and the corresponding message 
        obj.robotCmd = rospublisher("/cmd_vel","DataFormat","struct");
        obj.velMsg = rosmessage(obj.robotCmd);

        %subscriber for the odometry messages
        obj.odomSub = rossubscriber("/odom","DataFormat","struct");

        %subscribe to the lidar topic:
        obj.lidarSub = rossubscriber("/scan","DataFormat","struct");
    end

    % send (linear + angular) velocity commands. (v,w) are scalars
    function setVelocity(obj, v, w)
        obj.velMsg.Linear.X = v(1);
        obj.velMsg.Linear.Y = 0;
        obj.velMsg.Linear.Z = 0;
        obj.velMsg.Angular.X = 0;
        obj.velMsg.Angular.Y = 0;
        obj.velMsg.Angular.Z = w(1);
        send(obj.robotCmd, obj.velMsg)
    end

    % stop robot - send (zero) velocity commands
    function stop(obj)
        obj.setVelocity( 0, 0)
    end

    % reads 2D pose 
    % return (x,y) positon and (theta) orientation [deg]
    function [x,y,theta] = readPose(obj)
        odomMsg = receive(obj.odomSub,3);
        pose = odomMsg.Pose.Pose;
        x = pose.Position.X;
        y = pose.Position.Y;
        %z = pose.Position.Z;
        
        % read pose (returns a quaternion)
        quat = pose.Orientation;
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);      
        % angles in Euler format: ZYX
        theta = rad2deg( angles(1) ); 
    end

    % read lidar data, returns scanMsg structure, use rosPlot(scanMsg) to display
    function scanMsg = readLidar(obj)
        scanMsg = receive(obj.lidarSub);
    end 

    % Destructor: clear the workspace of publishers, subscribers, and other ROS-related objects
    % note: stops robot and disconnects ROS if class object if deleted.   
    function delete(obj)
        obj.stop()      % stop robot 
        rosshutdown     % shutdown ROS
    end

    end 
end


