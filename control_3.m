clearvars -except tbot

% init TurtleBot connection (tbot object), if required
if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end 

goal_pose = [1, 1.5, pi/2];
% tbot.setPose(0,1,0);

k_rho = 0.1;
k_alpha = 0.3;
k_beta = -0.2;
x_ = [];
y_ = [];

[x,y,theta]=tbot.readPose();
%theta = deg2rad(theta);

dx = goal_pose(1) - x;
dy = goal_pose(2) - y;
rho_ = sqrt(dx^2 + dy^2);

alpha_ = -theta + atan2(dy,dx);
alpha_ = atan2(sin(alpha_),cos(alpha_));

beta_ = -theta - alpha_ + goal_pose(3);
beta_ = atan2(sin(beta_),cos(beta_));

while (rho_>0.01 || beta_>0.05)
    dx = goal_pose(1) - x;
    dy = goal_pose(2) - y;
    rho_ = sqrt(dx^2 + dy^2);

    alpha_ = -theta + atan2(dy,dx);
    alpha_ = atan2(sin(alpha_),cos(alpha_));

    beta_ = -theta - alpha_ + goal_pose(3);
    beta_ = atan2(sin(beta_),cos(beta_));

    plot_pose(x,y,theta,goal_pose,x_,y_);

    dRho = -k_rho*rho_*cos(alpha_);
    dAlpha = k_rho*sin(alpha_)-k_alpha*alpha_-k_beta*beta_;
    dBeta = -k_rho*sin(alpha_);

    rho_ = dRho+rho_;
    alpha_ = dAlpha+alpha_;
    alpha_ = atan2(sin(alpha_),cos(alpha_));
    beta_ = dBeta+beta_;
    beta_ = atan2(sin(beta_),cos(beta_));

    v = k_rho*rho_;
    w = k_alpha*alpha_+k_beta*beta_;

    tbot.setVelocity(v,w);

    [x,y,theta]=tbot.readPose();
    x_ = [x_ x];
    y_ = [y_ y];
    %theta = deg2rad(theta);
end

tbot.resetPose();

function plot_pose(x, y, theta, goal_pose, x_, y_)
    figure(1); clf; hold on;            % clear figure, hold plots
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    plot(goal_pose(1), goal_pose(2),'bx', 'MarkerSize', 15)
    quiver(x,y,cos(theta),sin(theta), 0.1, 'Color','r','LineWidth',1, 'ShowArrowHead',1)
    quiver(goal_pose(1),goal_pose(2),cos(goal_pose(3)),sin(goal_pose(3)), 0.1, 'Color','b','LineWidth',1, 'ShowArrowHead',1)
    %line([0 x], [0 y], 'LineStyle', '--')
    plot(x_(1:5:end), y_(1:5:end),'--','Color','b')
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-2, 2, -2, 2])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    pause(0.1)
end