% This a simulation for 2 dof robot arm. The dynamics of robot is described
% in this simulation and the controller is implemented.

close all;

% parameters of robot arm
l1 = 2;
l2 = 1;
m1 = 1;
m2 = 0.5;
g = 9.81;

% simulation time
delta_t = 0.001;
sim_t = 50;
t = 0:delta_t:sim_t;

% initialize the state variables of the robot arm
x = zeros(2, length(t));
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

% simulation start
for i = 1:length(t)
    
    % inertia matrix
    inertia = [(m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(theta(2, i)) m2*l2^2+m2*l1*l2*cos(theta(2, i));
                m2*l2^2+m2*l1*l2*cos(theta(2, i))               m2*l2^2];
            
    % centrifugal force and Coriolis force
    cen_cor = [-2*m2*l1*l2*sin(theta(2, i))*theta_dot(1, i)*theta_dot(2, i)-m2*l1*l2*sin(theta(2, i))*theta_dot(2, i)^2;
                m2*l1*l2*sin(theta(2, i))*theta(1, i)^2];
            
    % gravity force
    gravity = [(m1+m2)*g*l1*cos(theta(1, i))+m2*g*l2*cos(theta(1, i)+theta(2, i));
                m2*l2*g*cos(theta(1, i)+theta(2, i))];
            
    % controller : PID control
    kp1 = 26;
    kd1 = 65;
    ki1 = 0.4;
    kp2 = 15;
    kd2 = 35;
    ki2 = 1;
    desired_theta = [-(1/3)*pi; (1/3)*pi];
    desired_theta_dot = [0; 0];
    
    % angle error and integral of angle error
    theta_error_now = desired_theta - theta(:, i);
    theta_error_dot_now = desired_theta_dot - theta_dot(:, i);
    theta_error_accu = theta_error_accu + theta_error_now;
    theta_error_accu(1) = error_bound(theta_error_accu(1));
    theta_error_accu(2) = error_bound(theta_error_accu(2));
    
    % control input
    tau(1, i) = kp1*(theta_error_now(1)) ...
                + kd1*(theta_error_dot_now(1)) ...
                + ki1*theta_error_accu(1) ...
                + gravity(1);
    tau(2, i) = kp2*(theta_error_now(2)) ...
                + kd2*(theta_error_dot_now(2)) ...
                + ki2*theta_error_accu(2) ...
                + gravity(2);
    
    % angular acceleration
    theta_ddot(:, i) = inertia\(tau(:, i) - cen_cor - gravity);
    
    % angular velocity (numerical integration)
    theta_dot(:, i+1) = theta_dot(:, i) + theta_ddot(:, i)*delta_t;
    
    % angle (numerical integration)
    theta(:, i+1) = theta(:, i) + theta_dot(:, i)*delta_t;
    theta_degree(1, i+1) = rad2deg(theta(1, i+1));
    theta_degree(2, i+1) = rad2deg(theta(2, i+1));
end

% plot the theta1 , theta2, and control input
plotter(t, theta_degree, tau);