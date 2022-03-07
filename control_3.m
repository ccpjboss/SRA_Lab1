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

rho_=1;
beta=1;
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

    [rho_,alpha_,beta_] = update_parameters(rho_,alpha_,beta_,k_rho,k_alpha,k_beta,last_update);
    last_update = tic;
    v = k_rho*rho_;
    w = k_alpha*alpha_+k_beta*beta_;

    tbot.setVelocity(v,w);

    [x,y,theta]=tbot.readPose();
    x_ = [x_ x];
    y_ = [y_ y];
end

fprintf("Goal reached!\n");
tbot.resetPose();

function plot_pose(x, y, theta, goal_pose, x_, y_)
    figure(1); clf; hold on;            % clear figure, hold plots
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    plot(goal_pose(1), goal_pose(2),'bx', 'MarkerSize', 15)
    
    quiver(x,y,cos(theta),sin(theta), 0.1, 'Color','r','LineWidth',1, ...
        'ShowArrowHead',1)

    quiver(goal_pose(1),goal_pose(2),cos(goal_pose(3)),sin(goal_pose(3)), 0.1, ...
        'Color','b','LineWidth',1, 'ShowArrowHead',1)

    plot(x_(1:5:end), y_(1:5:end),'--','Color','b')
    quiver(0,0,1,0,'r')                 % draw arrow for x-axis 
    quiver(0,0,0,1,'g')                 % draw arrow for y-axis 
    axis([-2, 2, -2, 2])                % the limits for the current axes [xmin xmax ymin ymax]
    grid on;                            % enable grid 
    xlabel('x')                         % axis labels 
    ylabel('y')
    pause(0.1)
end

function [rho_new, alpha_new, beta_new] = update_parameters(rho, alpha, beta, k_rho, k_alpha, k_beta,T)
    dRho = -k_rho*rho*cos(alpha);
    dAlpha = k_rho*sin(alpha)-k_alpha*alpha-k_beta*beta;
    dBeta = -k_rho*sin(alpha);

    rho_new = dRho*toc(T)+rho;
    alpha_new = dAlpha*toc(T)+alpha;
    alpha_new = atan2(sin(alpha_new),cos(alpha_new));
    beta_new = dBeta*toc(T)+beta;
    beta_new = atan2(sin(beta_new),cos(beta_new));
end