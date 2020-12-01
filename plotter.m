function plotter(t, theta_degree, tau)
    % plot theta1 and theta2
    figure(1)
    subplot(2, 1, 1)
    plot(t, theta_degree(1, 1:length(t)));
    title('Robotic Arm Angle (degree)')
    y = ylabel('$\theta_{1}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    subplot(2, 1, 2)
    plot(t, theta_degree(2, 1:length(t)))
    y = ylabel('$\theta_{2}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    xlabel('Time(sec)')

    % plot the control input
    figure(2)
    subplot(2, 1, 1)
    plot(t, tau(1, :));
    title('Control Input (N*m)')
    y = ylabel('$\tau_{1}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    subplot(2, 1, 2)
    plot(t, tau(2, :));
    y = ylabel('$\tau_{2}$', 'Interpreter', 'latex', 'rotation', 0); grid on;
    set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41]);
    xlabel('Time(sec)')
end