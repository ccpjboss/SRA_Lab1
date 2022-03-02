% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end 

kv = 0.1;
ks = 0.4;

goal_pose = [1 1.5];
dist = 1;
x_= [];
y_= [];

while (dist > 0.05)
    % read TurtleBot pose
    [x,y,theta] = tbot.readPose();
    %theta = deg2rad(theta);
    x_ = [x_ x];
    y_ = [y_ y];
    plot_pose(x, y,theta, goal_pose, x_, y_);
    dist = sqrt((goal_pose(1)-x)^2+(goal_pose(2)-y)^2);
    v = kv*dist;
    phi = atan((goal_pose(2)-y)/(goal_pose(1)-x));   
    w = ks*atan2(sin(phi-theta),cos(phi-theta));
    tbot.setVelocity(v,w);
end

tbot.resetPose();

function plot_pose(x, y, theta, goal_pose, x_, y_)
    figure(1); clf; hold on;            % clear figure, hold plots
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    plot(goal_pose(1), goal_pose(2),'bx', 'MarkerSize', 5)
    quiver(x,y,cos(theta),sin(theta), 0.1, 'Color','r','LineWidth',1, 'ShowArrowHead',1)
    %line([0 x], [0 y], 'LineStyle', '--')
    plot(x_(1:5:end), y_(1:5:end),'--')
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-2, 2, -2, 2])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    pause(0.1)
end

