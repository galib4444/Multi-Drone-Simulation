function DroneSimulation()
    % Simple Real-Time Drone Flight Simulation with Live Plot & PID Controller
    clc; clear; close all;

    % Constants
    m = 2;         % Mass (kg)
    g = 9.81;      % Gravity (m/s^2)
    k = 0.02;      % Drag coefficient
    dt = 0.1;      % Time step (s)
    t_max = 10;    % Total simulation time (s)

    % Wind Resistance
    v_wind = -5;   % Wind speed (m/s) (negative = headwind, positive = tailwind)
    k_w = 0.01;    % Wind drag coefficient

    % PID Controller Parameters
    target_altitude = 100;   % Desired altitude (meters)
    Kp = 1.5;                  % Proportional gain
    Ki = 0.3;                % Integral gain
    Kd = 1.2;                  % Derivative gain

    integral = 0;            % Initialize integral term
    previous_error = 0;      % Store last error value

    % Initialize variables
    v = 0;  % Initial velocity
    h = 0;  % Initial altitude
    time = 0;

    % Create figure for real-time plotting
    figure('Name', 'Drone Flight Simulation', 'NumberTitle', 'off');
    altitudePlot = subplot(2,1,1); hold on;
    velocityPlot = subplot(2,1,2); hold on;

    title(altitudePlot, 'Altitude Over Time');
    xlabel(altitudePlot, 'Time (s)'); ylabel(altitudePlot, 'Altitude (m)');
    grid on;

    title(velocityPlot, 'Velocity Over Time');
    xlabel(velocityPlot, 'Time (s)'); ylabel(velocityPlot, 'Velocity (m/s)');
    grid on;

    % Real-time simulation loop
    while time <= t_max
        % Compute Error for PID Controller
        error = target_altitude - h;   % Compute altitude error
        integral = integral + error * dt;  % Integral term (sum of errors)
        derivative = (error - previous_error) / dt;  % Derivative term (rate of change)
        
        % Compute PID-based Thrust Control
        T = Kp * error + Ki * integral + Kd * derivative;  % PID formula

        % Compute forces
        D = k * v^2 + k_w * (v - v_wind)^2;  % Includes wind resistance

        a = (T - D - m*g) / m;  % Acceleration
        v = v + a * dt;  % Update velocity
        h = h + v * dt;  % Update altitude
        time = time + dt; % Update time
        
        % Store error for next iteration
        previous_error = error;  

        % Plot real-time updates
        subplot(2,1,1); plot(time, h, 'bo', 'MarkerSize', 5);
        subplot(2,1,2); plot(time, v, 'ro', 'MarkerSize', 5);
        pause(0.05); % Small delay for visualization
    end
end
