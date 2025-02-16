function DroneSimulation3D()
    % Enhanced 3D Drone Flight Simulation with PID Control & Wind Resistance
    clc; clear; close all;

    % Constants
    m = 2;        % Mass (kg)
    g = 9.81;     % Gravity (m/s^2)
    k = 0.02;     % Drag coefficient
    dt = 0.1;     % Time step (s)
    t_max = 20;   % Simulation time (s)

    % Wind Resistance in 3D
    wind_x = 5;    % Wind speed in X direction (m/s)
    wind_y = -3;   % Wind speed in Y direction (m/s)
    wind_z = -5;   % Wind affecting altitude (negative = headwind)

    % PID Controller Parameters
    target_altitude = 100;  % Desired altitude (meters)
    Kp = 1.5; Ki = 0.2; Kd = 1.5;
    integral = 0; previous_error = 0;

    % Initialize drone state
    x = 0; y = 0; z = 0;  % Position
    vx = 5; vy = 3; vz = 0; % Initial velocity

    % 3D Plot
    figure('Name', '3D Drone Flight Simulation', 'NumberTitle', 'off');
    axis([-100 100 -100 100 0 150]); xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on; hold on; drone_plot = plot3(x, y, z, 'bo', 'MarkerSize', 5);

    % Simulation loop
    time = 0;
    while time <= t_max
        % PID control for altitude
        error = target_altitude - z;
        integral = integral + error * dt;
        derivative = (error - previous_error) / dt;
        T = Kp * error + Ki * integral + Kd * derivative;

        % Forces & motion
        Dx = k * vx^2; Dy = k * vy^2; Dz = k * vz^2;
        ax = (-Dx + wind_x) / m; ay = (-Dy + wind_y) / m;
        az = (T - Dz - m*g + wind_z) / m;

        % Update velocity & position
        vx = vx + ax * dt; vy = vy + ay * dt; vz = vz + az * dt;
        x = x + vx * dt; y = y + vy * dt; z = max(0, z + vz * dt);

        % Update plot
        if mod(time, 0.2) < dt
            set(drone_plot, 'XData', x, 'YData', y, 'ZData', z);
            drawnow;
        end

        previous_error = error; time = time + dt; pause(0.05);
    end
end
