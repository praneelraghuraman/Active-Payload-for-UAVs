% Simulation parameters
t = linspace(0, 30, 300); % Time vector from 0 to 30 seconds

% Desired drone trajectory waypoints
waypoints = [
    0, 0, 1;
    2, 4, 5;
    2, 8, 5;
    6, 8, 5;
    6, 4, 5;
    2, 4, 5
];

% Time spent at each waypoint (assuming constant speed)
waypoint_times = linspace(0, 30, size(waypoints, 1));

% Interpolate between waypoints
x_drone_des = interp1(waypoint_times, waypoints(:,1), t, 'linear');
y_drone_des = interp1(waypoint_times, waypoints(:,2), t, 'linear');
z_drone_des = interp1(waypoint_times, waypoints(:,3), t, 'linear');

% Constants
T_s = 0.1; % Sampling time
g = 9.81; % Gravity
m_l = 1; % Mass of the payload (kg)
l = 1; % Length of the cable (m)
max_phi = pi / 3; % Maximum swing angle (60 degrees)

% Wind disturbance parameters
wind_disturbance_mean_mps = 4 * 0.51444; % Mean wind speed (m/s)
wind_disturbance_std_mps = 1; % Standard deviation for wind disturbance (m/s)

% Number of simulations
num_simulations = 1;
rms_errors_transient = zeros(1, num_simulations);
rms_errors_steady_state = zeros(1, num_simulations);

for sim = 1:num_simulations
    % Initial conditions
    phi_x = 0; % Initial swing angle in X direction (radians)
    phi_x_dot = 0; % Initial angular velocity of the swing in X direction (radians/s)
    phi_y = 0; % Initial swing angle in Y direction (radians)
    phi_y_dot = 0; % Initial angular velocity of the swing in Y direction (radians/s)

    % Pre-allocate arrays for positions
    x_payload_positions = zeros(1, length(t));
    y_payload_positions = zeros(1, length(t));
    z_payload_positions = zeros(1, length(t));

    % Assume initial payload position directly below the drone
    x_payload_positions(1) = x_drone_des(1);
    y_payload_positions(1) = y_drone_des(1);
    z_payload_positions(1) = z_drone_des(1) - l;

    % Desired payload positions (directly below the drone)
    x_payload_des = x_drone_des;
    y_payload_des = y_drone_des;
    z_payload_des = z_drone_des - l;

    % Iterate through the trajectory
    for k = 2:length(t)
        % Wind disturbance generation
        wind_disturbance = normrnd(wind_disturbance_mean_mps, wind_disturbance_std_mps, 2, 1); % Random disturbance in x and y directions

        % Calculate drone's accelerations
        if k > 2
            ax_drone = (x_drone_des(k) - 2*x_drone_des(k-1) + x_drone_des(k-2)) / T_s^2;
            ay_drone = (y_drone_des(k) - 2*y_drone_des(k-1) + y_drone_des(k-2)) / T_s^2;
            az_drone = (z_drone_des(k) - 2*z_drone_des(k-1) + z_drone_des(k-2)) / T_s^2;
        else
            % Approximate accelerations for the first two steps
            ax_drone = (x_drone_des(k) - x_drone_des(k-1)) / T_s^2;
            ay_drone = (y_drone_des(k) - y_drone_des(k-1)) / T_s^2;
            az_drone = (z_drone_des(k) - z_drone_des(k-1)) / T_s^2;
        end

        % Update angular acceleration of the payload swing due to drone's motion and wind
        phi_x_ddot = (-g/l) * sin(phi_x) + (ax_drone + wind_disturbance(1)) * cos(phi_x) / l;
        phi_y_ddot = (-g/l) * sin(phi_y) + (ay_drone + wind_disturbance(2)) * cos(phi_y) / l;

        % Update angular velocities and angles
        phi_x_dot = phi_x_dot + phi_x_ddot * T_s;
        phi_x = phi_x + phi_x_dot * T_s;

        phi_y_dot = phi_y_dot + phi_y_ddot * T_s;
        phi_y = phi_y + phi_y_dot * T_s;

        % Constrain the swing angles to the maximum allowed value
        if abs(phi_x) > max_phi
            phi_x = sign(phi_x) * max_phi; % Limit swing angle in X to ±60 degrees
            phi_x_dot = 0; % Stop angular velocity if the limit is reached
        end
        if abs(phi_y) > max_phi
            phi_y = sign(phi_y) * max_phi; % Limit swing angle in Y to ±60 degrees
            phi_y_dot = 0; % Stop angular velocity if the limit is reached
        end

        % Update payload position in the x-y-z plane
        x_payload_positions(k) = x_drone_des(k) - l * sin(phi_x);
        y_payload_positions(k) = y_drone_des(k) - l * sin(phi_y);
        z_payload_positions(k) = z_drone_des(k) - l * cos(phi_x) * cos(phi_y);
    end

    % Calculate RMS errors for different time frames
    transient_time_indices = find(t <= 8);
    steady_state_time_indices = find(t > 8);

    rms_errors_transient(sim) = sqrt(mean((x_payload_positions(transient_time_indices) - x_payload_des(transient_time_indices)).^2 + ...
                                          (y_payload_positions(transient_time_indices) - y_payload_des(transient_time_indices)).^2 + ...
                                          (z_payload_positions(transient_time_indices) - z_payload_des(transient_time_indices)).^2));

    rms_errors_steady_state(sim) = sqrt(mean((x_payload_positions(steady_state_time_indices) - x_payload_des(steady_state_time_indices)).^2 + ...
                                             (y_payload_positions(steady_state_time_indices) - y_payload_des(steady_state_time_indices)).^2 + ...
                                             (z_payload_positions(steady_state_time_indices) - z_payload_des(steady_state_time_indices)).^2));

    % Plot the trajectories only for the first simulation
    if sim == 1
        figure('Position', [100, 100, 800, 600]);

        subplot(4, 1, 1);
        plot(t, x_payload_positions, 'r', 'LineWidth', 1.5);
        hold on;
        plot(t, x_drone_des, 'm--', 'LineWidth', 1.5); % Drone path as yellow dotted line
        plot(t, x_payload_des, 'k--', 'LineWidth', 1.5); % Desired payload path as black dotted line
        title('X Trajectory', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('X Position (m)', 'FontSize', 12);
        legend('Payload Path', 'Drone Path', 'Desired Payload Path', 'FontSize', 10);
        grid on;
    
        subplot(4, 1, 2);
        plot(t, y_payload_positions, 'g', 'LineWidth', 1.5);
        hold on;
        plot(t, y_drone_des, 'm--', 'LineWidth', 1.5); % Drone path as yellow dotted line
        plot(t, y_payload_des, 'k--', 'LineWidth', 1.5); % Desired payload path as black dotted line
        title('Y Trajectory', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Y Position (m)', 'FontSize', 12);
        legend('Payload Path', 'Drone Path', 'Desired Payload Path', 'FontSize', 10);
        grid on;
    
        subplot(4, 1, 3);
        plot(t, z_payload_positions, 'b', 'LineWidth', 1.5);
        hold on;
        plot(t, z_drone_des, 'm--', 'LineWidth', 1.5); % Drone path as yellow dotted line
        plot(t, z_payload_des, 'k--', 'LineWidth', 1.5); % Desired payload path as black dotted line
        title('Z Trajectory', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Z Position (m)', 'FontSize', 12);
        legend('Payload Path', 'Drone Path', 'Desired Payload Path', 'FontSize', 10);
        grid on;

        % 3D Plot
        figure('Position', [100, 100, 800, 600]);
        plot3(x_payload_positions, y_payload_positions, z_payload_positions, 'r', 'LineWidth', 1.5);
        hold on;
        plot3(x_payload_des, y_payload_des, z_payload_des, 'b--', 'LineWidth', 1.5); % Desired payload path
        plot3(x_drone_des, y_drone_des, z_drone_des, 'k--', 'LineWidth', 1.5);
        grid on;
        title('3D Trajectory of Payload and Drone', 'FontSize', 14);
        xlabel('X Position (m)', 'FontSize', 12);
        ylabel('Y Position (m)', 'FontSize', 12);
        zlabel('Z Position (m)', 'FontSize', 12);
        legend('Payload Path', 'Desired Payload Path', 'Drone Path', 'FontSize', 10);

        % Wind disturbance at the final position of the drone
        wind_disturbance_final = normrnd(wind_disturbance_mean_mps, wind_disturbance_std_mps, 2, 1); % Final wind disturbance

        % Calculate wind intensity and direction
        wind_intensity = norm(wind_disturbance_final); % Euclidean norm of the wind vector
        wind_direction = atan2d(wind_disturbance_final(2), wind_disturbance_final(1)); % Angle in degrees

        % Plot wind disturbance vector
        quiver3(x_drone_des(end), y_drone_des(end), z_drone_des(end), ...
                wind_disturbance_final(1), wind_disturbance_final(2), 0, 2, 'c', 'LineWidth', 2);

        % Construct wind parameters text for the legend
        wind_params_text = sprintf('Wind: Intensity = %.2f m/s, Direction = %.2f°', wind_intensity, wind_direction);

        % Include wind parameters in the legend
        legend('Payload Path', 'Desired Payload Path', 'Drone Path', ['Wind Disturbance (' wind_params_text ')'], 'FontSize', 10);
    end
end

% Plot histogram of Transient RMS errors
figure('Position', [100, 100, 800, 600]);
histogram(rms_errors_transient, 'Normalization', 'pdf', 'BinWidth', 0.01);
title('Histogram of Transient to Steady State RMS Errors (0-8 seconds)', 'FontSize', 14);
xlabel('RMS Error (m)', 'FontSize', 12);
ylabel('Frequency', 'FontSize', 12);
grid on;

% Fit a Gaussian distribution to the Transient RMS errors
mu_transient = mean(rms_errors_transient);
sigma_transient = std(rms_errors_transient);

% Plot the Gaussian curve
hold on;
x_values_transient = linspace(min(rms_errors_transient), max(rms_errors_transient), 100);
gaussian_curve_transient = (1/(sigma_transient*sqrt(2*pi))) * exp(-((x_values_transient-mu_transient).^2)/(2*sigma_transient^2));
plot(x_values_transient, gaussian_curve_transient, 'r-', 'LineWidth', 2);
legend('Transient RMS Error Distribution', 'Fitted Gaussian Curve', 'FontSize', 10);

% Plot histogram of Steady State RMS errors
figure('Position', [100, 100, 800, 600]);
histogram(rms_errors_steady_state, 'Normalization', 'pdf', 'BinWidth', 0.01);
title('Histogram of Steady State RMS Errors (8-30 seconds)', 'FontSize', 14);
xlabel('RMS Error (m)', 'FontSize', 12);
ylabel('Frequency', 'FontSize', 12);
grid on;

% Fit a Gaussian distribution to the Steady State RMS errors
mu_steady_state = mean(rms_errors_steady_state);
sigma_steady_state = std(rms_errors_steady_state);

% Plot the Gaussian curve
hold on;
x_values_steady_state = linspace(min(rms_errors_steady_state), max(rms_errors_steady_state), 100);
gaussian_curve_steady_state = (1/(sigma_steady_state*sqrt(2*pi))) * exp(-((x_values_steady_state-mu_steady_state).^2)/(2*sigma_steady_state^2));
plot(x_values_steady_state, gaussian_curve_steady_state, 'r-', 'LineWidth', 2);
legend('Steady State RMS Error Distribution', 'Fitted Gaussian Curve', 'FontSize', 10);
