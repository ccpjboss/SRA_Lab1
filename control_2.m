% init TurtleBot connection (tbot object), if required
clearvars -except tbot

if ( ~exist("tbot") ) 
    % Note: edit TurtleBot.m to define the robot and local host IP addresses
    tbot = TurtleBot(); 
end

% reta = [0 1 -0.5]; %ax+by+c=0
% reta = [5 1 0]; %ax+by+c=0
hold on
figure(1);
axis([-2, 2, -2, 2])                % the limits for the current axes [xmin xmax ymin ymax]
grid on;                            % enable grid 
xlabel('x')                         % axis labels 
ylabel('y')
title(['Escolha 2 pontos para desenhar a linha'])
[px,py]=ginput(2);
title(['Movimento segundo uma linha'])
hold off

reta = [py(1)-py(2), px(2)-px(1), px(1)*py(2)-px(2)*py(1)]; %Vetor reta (a,b,c) c/ ax+by+c=0

x_ = [];
y_ = [];

kd = 0.6;
kh = 0.3;

% tbot.setPose(0,1,0);
while (1)
    [x,y,theta] = tbot.readPose();
    x_ = [x_ x];
    y_ = [y_ y];
%     theta = deg2rad(theta);
    plot_pose(x, y, theta,reta,x_,y_);

    d = dot(reta,[x,y,1])/sqrt(reta(1)^2+reta(2)^2);
    phi = atan(-reta(1)/reta(2));

    alpha_d = -kd*d;
    alpha_h = kh*atan2(sin(phi-theta),cos(phi-theta));
    w = alpha_d+alpha_h;

    tbot.setVelocity(0.075,w);
end

function plot_pose(x, y, theta, reta, x_, y_)
    figure(1); clf; hold on;            % clear figure, hold plots
    plot(x, y,'--or', 'MarkerSize', 10)  % display (x,y) location of the robot
    x_reta = -2:0.2:2;
    y_reta = (-reta(1).*x_reta-reta(3))./reta(2); 
    plot(x_reta,y_reta);
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