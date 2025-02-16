function MultiDroneSimulationAI()
    % Multi-Drone AI Simulation with Obstacle Avoidance & Target Navigation
    clc; clear; close all;

    % === Simulation Parameters ===
    m = 2;        % Mass of drones (kg)
    g = 9.81;     % Gravity (m/s^2)
    k = 0.02;     % Drag coefficient
    dt = 0.1;     % Time step (s)
    t_max = 40;   % Total simulation time (s)
    num_drones = 3; % Number of drones

    % === Wind Resistance (Affects Drone Motion) ===
    wind_x = 2;  wind_y = -2;  wind_z = -5;

    % === PID Controller for Altitude (Z-Axis) ===
    target_altitude = 100;  
    target_x = 50; % X-Goal
    target_y = 50; % Y-Goal
    Kp = 2.5; Ki = 0.5; Kd = 1.8;
    integral = zeros(num_drones, 1); 
    previous_error = zeros(num_drones, 1);

    % === Initialize Drone States ===
    drones = struct();
    for i = 1:num_drones
        drones(i).x = -50;   % Start at (-50, -50)
        drones(i).y = -50;   
        drones(i).z = 0;  % Start on the ground
        drones(i).vx = 3;  % Initial velocity towards target
        drones(i).vy = 3;  
        drones(i).vz = 3;
    end

    % === Define Obstacles (Static Positions) ===
    obstacles = [0, 0, 50; 20, 10, 80; -10, 30, 60; -30, 10, 90; 15, -20, 70];

    % === Create 3D Plot ===
    figure('Name', 'Multi-Drone AI Navigation', 'NumberTitle', 'off');
    axis([-60 60 -60 60 0 120]);  
    xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Altitude (m)');
    grid on; hold on;
    drone_plots = gobjects(num_drones, 1);
    
    % Plot obstacles
    scatter3(obstacles(:,1), obstacles(:,2), obstacles(:,3), 100, 'r', 'filled');

    % Initialize drone plots
    for i = 1:num_drones
        drone_plots(i) = plot3(drones(i).x, drones(i).y, drones(i).z, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    end

    % === Simulation Loop ===
    for time = 0:dt:t_max
        for i = 1:num_drones
            % === Compute Altitude Error for PID Control ===
            error = target_altitude - drones(i).z;
            integral(i) = integral(i) + error * dt;
            derivative = (error - previous_error(i)) / dt;
            T = Kp * error + Ki * integral(i) + Kd * derivative;

            % === Compute Forces & Motion ===
            Dx = k * drones(i).vx^2; Dy = k * drones(i).vy^2; Dz = k * drones(i).vz^2; 
            ax = (-Dx + wind_x) / m;  
            ay = (-Dy + wind_y) / m;  
            az = (T - Dz - m*g + wind_z) / m;

            % === Compute Direction Toward Target (50, 50) ===
            direction = [target_x - drones(i).x, target_y - drones(i).y, target_altitude - drones(i).z];
            direction = direction / norm(direction); % Normalize vector

            % Move toward the target
            drones(i).vx = drones(i).vx + direction(1) * 3 + ax * dt;
            drones(i).vy = drones(i).vy + direction(2) * 3 + ay * dt;
            drones(i).vz = drones(i).vz + direction(3) * 3 + az * dt;
            drones(i).x = drones(i).x + drones(i).vx * dt;  
            drones(i).y = drones(i).y + drones(i).vy * dt;  
            drones(i).z = drones(i).z + drones(i).vz * dt;

            % === AI-Based Obstacle Avoidance ===
            for j = 1:size(obstacles,1)
                obs_dist = norm([drones(i).x, drones(i).y, drones(i).z] - obstacles(j,:));
                if obs_dist < 10  
                    % Compute escape direction
                    avoid_dir = ([drones(i).x, drones(i).y, drones(i).z] - obstacles(j,:)) / obs_dist;
                    drones(i).vx = drones(i).vx + avoid_dir(1) * 8;  
                    drones(i).vy = drones(i).vy + avoid_dir(2) * 8;  
                    drones(i).vz = drones(i).vz + avoid_dir(3) * 5;  
                end
            end

            % === Update Drone Position in 3D Plot ===
            set(drone_plots(i), 'XData', drones(i).x, 'YData', drones(i).y, 'ZData', drones(i).z);
            
            % Store error for next iteration
            previous_error(i) = error;
        end
        drawnow;
        pause(0.05);  % Small delay for visualization
    end
end
