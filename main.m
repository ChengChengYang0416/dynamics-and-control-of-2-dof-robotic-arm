% This a simulation for 2 dof robot arm. The dynamics of robot is described
% in this simulation and the controller is implemented.

close all;
init();

% simulation time
delta_t = 0.005;
sim_t = 20;
t = 0:delta_t:sim_t;

% initialize the state variables of the robot arm
theta = zeros(2, length(t)+1);
theta_dot = zeros(2, length(t)+1);
theta_ddot = zeros(2, length(t));
tau = zeros(2, length(t));
theta_degree = zeros(2, length(t)+1);
theta_error_now = zeros(2, 1);
theta_error_accu = zeros(2, 1);

% initialize the parameters of the dynamics
inertia = zeros(2, 2);
cen_cor = zeros(2, 1);
gravity = zeros(2, 1);

% desired angle and desired angular velocity
desired_theta = [-(1/3)*pi; -(2/3)*pi];
desired_theta_dot = [0; 0];

% simulation start
for i = 1:length(t)
    
    % inertia matrix
    inertia = [(arm1.m+arm2.m)*arm1.l^2+arm2.m*arm2.l^2+2*arm2.m*arm1.l*arm2.l*cos(theta(2, i)) arm2.m*arm2.l^2+arm2.m*arm1.l*arm2.l*cos(theta(2, i));
                arm2.m*arm2.l^2+arm2.m*arm1.l*arm2.l*cos(theta(2, i))                           arm2.m*arm2.l^2];
            
    % centrifugal force and Coriolis force
    cen_cor = [-2*arm2.m*arm1.l*arm2.l*sin(theta(2, i))*theta_dot(1, i)*theta_dot(2, i)-arm2.m*arm1.l*arm2.l*sin(theta(2, i))*theta_dot(2, i)^2;
                arm2.m*arm1.l*arm2.l*sin(theta(2, i))*theta(1, i)^2];
            
    % gravity force
    gravity = [(arm1.m+arm2.m)*g*arm1.l*arm1.l*cos(theta(1, i))+arm2.m*g*arm2.l*cos(theta(1, i)+theta(2, i));
                arm2.m*arm2.l*g*cos(theta(1, i)+theta(2, i))];

    % angle error and integral of angle error
    theta_error_now = desired_theta - theta(:, i);
    theta_error_dot_now = desired_theta_dot - theta_dot(:, i);
    theta_error_accu = theta_error_accu + theta_error_now;
    theta_error_accu(1) = error_bound(theta_error_accu(1), 5);
    theta_error_accu(2) = error_bound(theta_error_accu(2), 5);
    
    % pid controller
    tau(:, i) = controller_pid(pid1, pid2, theta_error_now, theta_error_dot_now, theta_error_accu, gravity);
    
    % dynamics of robotic arm
    states = dynamics(delta_t, inertia, cen_cor, gravity, tau(:, i), theta_dot(:, i), theta(:, i));
    theta_ddot(:, i) = states(1:2);
    theta_dot(:, i+1) = states(3:4);
    theta(:, i+1) = states(5:6);
    
    % convert radian to degree
    theta_degree(1, i+1) = rad2deg(theta(1, i+1));
    theta_degree(2, i+1) = rad2deg(theta(2, i+1));
end

% plot the theta1 , theta2, and control input
plotter(t, theta_degree, tau);

% animation
figure(3)
x = zeros(2, length(theta(1, :)));
y = zeros(2, length(theta(1, :)));
t(1, length(theta(1, :))) = sim_t;
set(gca, 'Xlim', [-4.2 4.2], 'Ylim', [-4.2 1])
grid on
arm_1 = animatedline('Linewidth', 2, 'color', 'g');
arm_2 = animatedline('Linewidth', 2, 'color', 'b');
l = zeros(1, length(theta(1, :)));
l2 = zeros(1, length(theta(1, :)));
xlabel('X(m)')
ylabel('Y(m)')
title('2-D Position of Robotic Arm')

for i = 1:length(theta(1, :))
    x(1, i) = arm1.l*cos(theta(1, i));
    y(1, i) = arm1.l*sin(theta(1, i));
    x(2, i) = arm1.l*cos(theta(1, i)) + arm2.l*cos(theta(2, i));
    y(2, i) = arm1.l*sin(theta(1, i)) + arm2.l*sin(theta(2, i));
    clearpoints(arm_1);
    clearpoints(arm_2);
    delete(t_text);
    addpoints(arm_1, 0, 0);
    addpoints(arm_1, x(1, i), y(1, i));
    addpoints(arm_2, x(1, i), y(1, i));
    addpoints(arm_2, x(2, i), y(2, i));
    t_text = text(-3.5, 0.65, "Time : "+num2str(t(i), '%4.2f')+" / "+num2str(t(end), '%4.2f'));
    drawnow
    l(i) = sqrt(x(1, i)^2+y(1, i)^2);
    l2(i) = sqrt((x(2, i)-x(1, i))^2+(y(2, i)-y(1, i))^2);
end
