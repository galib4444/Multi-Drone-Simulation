function DroneSimulationAI()
    % AI-Powered 3D Drone Simulation with PID Control & Obstacle Avoidance
    clc; clear; close all;

    % === Simulation Parameters ===
    m = 2;        % Mass of the drone (kg)
    g = 9.81;     % Gravity (m/s^2)
    k = 0.02;     % Drag coefficient
    dt = 0.1;     % Time step (s)
    t_max = 60;   % Total simulation time (s)

    % === Wind Resistance (Affects Drone Motion) ===
    wind_x = 2;  % Wind in X direction (m/s)
    wind_y = -2; % Wind in Y direction (m/s)
    wind_z = -5; % Wind affecting altitude (negative = headwind)

    % === PID Controller for Altitude (Z-Axis) ===
    target_altitude = 100;  % Desired altitude (meters)
    Kp = 2.5;  % Proportional gain (increased for faster correction)
    Ki = 0.5;  % Integral gain (stabilizes altitude)
    Kd = 1.8;  % Derivative gain (dampens overshoot)
    integral = 0; 
    previous_error = 0;

    % === Initialize Drone State ===
    x = 0;  y = 0;  z = 0;   % Initial position
    vx = 10; vy = 10; vz = 5;  % Initial velocities

    % === Define Obstacles (Static Positions) ===
    obstacles = [10, 15, 50; -20, -10, 80; 25, 30, 60; -10, 20, 90; 5, -15, 70];

    % === Create 3D Plot ===
    figure('Name', 'AI Drone Flight Simulation', 'NumberTitle', 'off');
    axis([-50 50 -50 50 0 120]);  
    xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Altitude (m)');
    grid on; hold on;
    drone_plot = plot3(x, y, z, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    scatter3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 100, 'r', 'filled'); % Plot obstacles

    % === Simulation Loop ===
    for time = 0:dt:t_max
        % === Compute Altitude Error for PID Control ===
        error = target_altitude - z;
        integral = integral + error * dt;
        derivative = (error - previous_error) / dt;
        T = Kp * error + Ki * integral + Kd * derivative; % PID-controlled thrust

        % === Compute Forces & Motion ===
        Dx = k * vx^2; Dy = k * vy^2; Dz = k * vz^2; % Drag forces
        ax = (-Dx + wind_x) / m;  
        ay = (-Dy + wind_y) / m;  
        az = (T - Dz - m*g + wind_z) / m;

        % === Update Velocity & Position ===
        vx = vx + ax * dt;  vy = vy + ay * dt;  vz = vz + az * dt;
        x = x + vx * dt;  y = y + vy * dt;  z = z + vz * dt;

        % === AI-Based Obstacle Avoidance ===
        for i = 1:size(obstacles,1)
            obs_dist = norm([x, y, z] - obstacles(i,:));
            if obs_dist < 10  % If within 10m range of an obstacle
                % Compute escape direction
                direction = ([x, y, z] - obstacles(i,:)) / obs_dist;
                vx = vx + direction(1) * 8;  % Move away in X
                vy = vy + direction(2) * 8;  % Move away in Y
                vz = vz + direction(3) * 5;  % Adjust altitude slightly
            end
        end

        % Store error for next iteration
        previous_error = error;

        % === Update Drone Position in 3D Plot ===
        set(drone_plot, 'XData', x, 'YData', y, 'ZData', z);
        drawnow;
        
        pause(0.05);  % Small delay for visualization
    end
end
