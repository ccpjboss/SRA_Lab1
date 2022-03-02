
% 
% Demo - Read from TurtleBot3 Lidar (pooling vs ~fixed timming) 
%
% Launch the virtual machine.
% Open terminal, and start the house Gazebo world: ./start-gazebo-house.sh.
% [Gazebo tip] The robot initial pose configuration can be recovered at Edit -> Reset Model Poses


% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end 

% start by moving the robot forward
tbot.setVelocity(0.075, 0)



% -----------------------------------------------
% option 1: read lidar data by pooling
% -----------------------------------------------
fprintf('read lidar data by pooling\n')

tic
% run from 10 seconds
while (toc < 10)             

    % load lidar data from robot 
    scan = tbot.readLidar();

    % display data
    figure(1); clf;
    rosPlot( scan )
    pause(0.1)      
end

% stop robot 
tbot.stop()


% ---------------------------------------------------------


% move the robot forward, turning slightly right
tbot.setVelocity(0.075, -0.05)

% -----------------------------------------------
% option 2: read lidar data w/ fixed timming
% -----------------------------------------------
fprintf('read lidar data w/ time control\n')

% init ratecontrol obj (enables to run a loop at a fixed frequency) 
r = rateControl(5);     % run at 5Hz

% run 50 iterations (50 x 1/5 = 10s) 
for i = 1:1:50

    % load lidar data from robot 
    scan = tbot.readLidar();

    % display data
    figure(1); clf;
    rosPlot( scan )

    % adaptive pause
    waitfor(r);
end

% check statistics of past execution periods
% stats = statistics(r)

% stop robot 
tbot.stop()

