function states = dynamics(dt, inertia, cen_cor, gravity, tau, theta_dot, theta)
    states = zeros(6, 1);
    states(1:2) = inertia\(tau - cen_cor - gravity);
    states(3:4) = theta_dot + states(1:2)*dt;
    states(5:6) = theta + states(3:4)*dt;
end