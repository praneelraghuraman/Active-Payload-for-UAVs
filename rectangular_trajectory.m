% Initialize workspace variables
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

% Sampling time and constants
T_s = 0.1; % Sampling time
g = 9.81; % Gravity
m_payload = 1; % Mass of the payload
m_drone = 5; % Mass of the drone
l_cable = 1; % Length of the cable

% Wind parameters
wind_min_speed_knots = 5; % Minimum wind speed in knots
wind_max_speed_knots = 10; % Maximum wind speed in knots
wind_min_speed_mps = wind_min_speed_knots * 0.51444; % Convert knots to m/s
wind_max_speed_mps = wind_max_speed_knots * 0.51444; % Convert knots to m/s

% Wind disturbance parameters
wind_disturbance_mean_knots = 4; % Peak of the Gaussian distribution in knots
wind_disturbance_mean_mps = wind_disturbance_mean_knots * 0.51444; % Convert knots to m/s
wind_disturbance_std_mps = 1; % Standard deviation for the Gaussian distribution in m/s

% Initial conditions
theta = 0; % Initial assumption
phi = 0; % Initial assumption

% Calculate the desired payload trajectory based on the drone's waypoints
x_payload_des = x_drone_des - l_cable * sin(theta) * cos(phi);
y_payload_des = y_drone_des - l_cable * sin(phi);
z_payload_des = z_drone_des - l_cable * cos(theta) * cos(phi);

% Desired payload trajectory
x_des = x_payload_des;
y_des = y_payload_des;
z_des = z_payload_des;

% State-space matrices for payload dynamics
A_d = [
    1,  T_s, 0,   0,  0,  0,  0,  0,  0,  0;
    0,  1,   0,   0,  0,  0,  T_s * (-g * sin(theta) * cos(phi) / cos(theta)), 0,  0,  0;
    0,  0,   1,   T_s, 0,  0,  0,  0,  T_s * (-g * sin(phi) / cos(theta)), 0;
    0,  0,   0,   1,   0,  0,  0,  0,  0,  0;
    0,  0,   0,   0,   1,  T_s, 0,  0,  0,  0;
    0,  0,   0,   0,   0,  1,  0,  0,  0,  0;
    0,  0,   0,   0,   0,  0,  1,  T_s, 0,  0;
    0,  0,   0,   0,   0,  0,  0,  1,  T_s * (-m_payload * g * l_cable * sin(phi) / (m_payload * g)), 0;
    0,  0,   0,   0,   0,  0,  0,  0,  1,  T_s;
    0,  0,   0,   0,   0,  0,  T_s * (m_payload * g * l_cable * tan(theta) * cos(phi) / (m_payload * g)), 0,  0,  1
];

B_d = [
    0,  0, 0;
    T_s * (1/m_payload), 0, 0;
    0,  0, 0;
    0,  T_s * (1/m_payload), 0;
    0,  0, 0;
    0,  0, T_s/m_payload;
    0,  0, 0;
    0,  0, 0;
    0,  0, 0;
    0,  0, 0
];

C = eye(10); % 10x10 Identity matrix
D = zeros(10, 3); % 10x3 Zero matrix (updated for 3 control inputs)

% PID controller parameters (tuning required)
Kp = [5; 5; 5]; % Proportional gain for X, Y, and Z
Ki = [2; 2; 2]; % Integral gain for X, Y, and Z
Kd = [3; 3; 3]; % Derivative gain for X, Y, and Z

% Number of simulations
num_simulations = 10;
rms_errors_transient = zeros(1, num_simulations);
rms_errors_steady_state = zeros(1, num_simulations);

% Time indices for calculating RMS errors
transient_time_indices = t >= 0 & t <= 8;
steady_state_time_indices = t > 8 & t <= 30;

% Run the simulation 20,000 times
for sim = 1:num_simulations
    % Initialize state vectors
    x = zeros(10, length(t)); % Payload state
    u = zeros(3, length(t)); % Control input vector with 3 inputs (for u1, u2, u3)
    e_int = zeros(3, length(t)); % Error integral
    e_prev = zeros(3, length(t)); % Previous error

    % Initialize drone position
    x_drone = x_drone_des(1);
    y_drone = y_drone_des(1);
    z_drone = z_drone_des(1);

    % Pre-allocate arrays for plotting
    x_drone_positions = zeros(1, length(t));
    y_drone_positions = zeros(1, length(t));
    z_drone_positions = zeros(1, length(t));

    % Error arrays for plotting
    error_x = zeros(1, length(t));
    error_y = zeros(1, length(t));

    for k = 1:length(t)-1
        % Wind disturbance generation using Gaussian distribution
        wind_disturbance = normrnd(wind_disturbance_mean_mps, wind_disturbance_std_mps, 2, 1); % Random disturbance in x and y directions

        % Update drone position based on desired trajectory (without wind disturbance for drone)
        x_drone = x_drone_des(k);
        y_drone = y_drone_des(k);
        z_drone = z_drone_des(k);
        
        % Store drone positions for plotting
        x_drone_positions(k+1) = x_drone;
        y_drone_positions(k+1) = y_drone;
        z_drone_positions(k+1) = z_drone;

        % Calculate the payload's desired position in relation to the drone
        x_payload_des = x_drone - l_cable * sin(theta) * cos(phi);
        y_payload_des = y_drone - l_cable * sin(phi);
        z_payload_des = z_drone - l_cable * cos(theta) * cos(phi);

        % Error calculation
        x_actual = x(1, k);
        y_actual = x(3, k);
        z_actual = x(5, k);
        
        e = [x_payload_des - x_actual; y_payload_des - y_actual; z_payload_des - z_actual];
        e_dot = (e - e_prev(:, k)) / T_s;
        e_int(:, k+1) = e_int(:, k) + e * T_s;
        
        % PID control law for x, y, and z
        u1 = Kp(1) * e(1) + Ki(1) * e_int(1, k) + Kd(1) * e_dot(1);
        u2 = Kp(2) * e(2) + Ki(2) * e_int(2, k) + Kd(2) * e_dot(2);
        u3 = Kp(3) * e(3) + Ki(3) * e_int(3, k) + Kd(3) * e_dot(3);
        
        % Ensure u1, u2, and u3 do not exceed physical limits
        max_force = 50; % Example maximum force (adjust as needed)
        u1 = min(max(u1, -max_force), max_force);
        u2 = min(max(u2, -max_force), max_force);
        max_thrust = 3; % Example maximum thrust (adjust as needed)
        u3 = min(max(u3, -max_thrust), max_thrust);

        % Add wind disturbance effect to control inputs
        u1 = u1 - wind_disturbance(1);
        u2 = u2 - wind_disturbance(2);

        u(:, k+1) = [u1; u2; u3];

        % State update based on control inputs
        x(:, k+1) = A_d * x(:, k) + B_d * u(:, k+1);
        
        % Update previous error
        e_prev(:, k+1) = e;
        
        % Store errors for plotting
        error_x(k+1) = e(1);
        error_y(k+1) = e(2);
    end

    % Calculate RMS errors for different phases
    rms_errors_transient(sim) = sqrt(mean((x(1, transient_time_indices) - x_des(transient_time_indices)).^2 + ...
                                          (x(3, transient_time_indices) - y_des(transient_time_indices)).^2 + ...
                                          (x(5, transient_time_indices) - z_des(transient_time_indices)).^2));

    rms_errors_steady_state(sim) = sqrt(mean((x(1, steady_state_time_indices) - x_des(steady_state_time_indices)).^2 + ...
                                             (x(3, steady_state_time_indices) - y_des(steady_state_time_indices)).^2 + ...
                                             (x(5, steady_state_time_indices) - z_des(steady_state_time_indices)).^2));

    % Plot the trajectories only for the first simulation
    if sim == 1
        % Plotting the results       
        % Plotting control inputs
        figure('Position', [100, 100, 800, 600]);
        subplot(2, 1, 1); % Create subplot with 2 rows, 1 column, and this is the first plot
        plot(t, u(1, :), 'r', 'LineWidth', 1.5);
        title('Control Input u1 (F_xp)', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Control Signal', 'FontSize', 12);
        grid on;
        set(gca, 'FontSize', 12);
        
        % Plot u2
        subplot(2, 1, 2); % Create subplot with 2 rows, 1 column, and this is the second plot
        plot(t, u(2, :), 'g', 'LineWidth', 1.5);
        title('Control Input u2 (F_yp)', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Control Signal', 'FontSize', 12);
        grid on;
        set(gca, 'FontSize', 12);
        
        
        % Plotting error in X and Y
        figure('Position', [100, 100, 800, 600]);
        subplot(2, 1, 1);
        plot(t, error_x, 'r', 'LineWidth', 1.5);
        title('Error in X Position', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Error (m)', 'FontSize', 12);
        grid on;
        set(gca, 'FontSize', 12);
        
        subplot(2, 1, 2);
        plot(t, error_y, 'g', 'LineWidth', 1.5);
        title('Error in Y Position', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Error (m)', 'FontSize', 12);
        grid on;
        set(gca, 'FontSize', 12);

        % Combined 3D Plot of Payload and Drone
        figure('Position', [100, 100, 800, 600]);
        plot3(x(1, :), x(3, :), x(5, :), 'r', 'LineWidth', 1.5);
        hold on;
        plot3(x_des, y_des, z_des, 'b--', 'LineWidth', 1.5);
        plot3(x_drone_positions, y_drone_positions, z_drone_positions, 'k--', 'LineWidth', 1.5);
        
        % Wind disturbance at the final position of the drone
        wind_disturbance_final = normrnd(wind_disturbance_mean_mps, wind_disturbance_std_mps, 2, 1); % Final wind disturbance
        
        % Calculate wind intensity and direction
        wind_intensity = norm(wind_disturbance_final); % Euclidean norm of the wind vector
        wind_direction = atan2d(wind_disturbance_final(2), wind_disturbance_final(1)); % Angle in degrees
        
        % Plot wind disturbance vector
        quiver3(x_drone_positions(end), y_drone_positions(end), z_drone_positions(end), ...
                wind_disturbance_final(1), wind_disturbance_final(2), 0, 2, 'c', 'LineWidth', 2);
        
        % Construct wind parameters text for the legend
        wind_params_text = sprintf('Wind: Intensity = %.2f m/s, Direction = %.2f°', wind_intensity, wind_direction);
        
        % Include wind parameters in the legend
        legend('Actual Payload Path', 'Desired Payload Path', 'Drone Path', ['Wind Disturbance (' wind_params_text ')'], 'FontSize', 10);
        
        grid on;
        title('3D Trajectory of Payload and Drone with Wind Effect', 'FontSize', 14);
        xlabel('X Position (m)', 'FontSize', 12);
        ylabel('Y Position (m)', 'FontSize', 12);
        zlabel('Z Position (m)', 'FontSize', 12);
        set(gca, 'FontSize', 12);

        figure('Position', [100, 100, 800, 600]);
        subplot(4, 1, 1);
        plot(t, x(1, :), 'r', 'LineWidth', 1.5);
        hold on;
        plot(t, x_des, 'b--', 'LineWidth', 1.5);
        plot(t, x_drone_positions, 'k--', 'LineWidth', 1.5);
        title('X Trajectory', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('X Position (m)', 'FontSize', 12);
        legend('Actual Payload Path', 'Desired Payload Path', 'Drone Path', 'FontSize', 10);
        grid on;
        set(gca, 'FontSize', 12);
        
        subplot(4, 1, 2);
        plot(t, x(3, :), 'g', 'LineWidth', 1.5);
        hold on;
        plot(t, y_des, 'b--', 'LineWidth', 1.5);
        plot(t, y_drone_positions, 'k--', 'LineWidth', 1.5);
        title('Y Trajectory', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Y Position (m)', 'FontSize', 12);
        legend('Actual Payload Path', 'Desired Payload Path', 'Drone Path', 'FontSize', 10);
        grid on;
        set(gca, 'FontSize', 12);
        
        subplot(4, 1, 3);
        plot(t, x(5, :), 'k', 'LineWidth', 1.5);
        hold on;
        plot(t, z_des, 'b--', 'LineWidth', 1.5);
        plot(t, z_drone_positions, 'k--', 'LineWidth', 1.5);
        title('Z Trajectory', 'FontSize', 14);
        xlabel('Time (s)', 'FontSize', 12);
        ylabel('Z Position (m)', 'FontSize', 12);
        legend('Actual Payload Path', 'Desired Payload Path', 'Drone Path', 'FontSize', 10);
        grid on;
        set(gca, 'FontSize', 12);
    end
end

% Plot histogram of Transient to Steady State RMS errors
figure('Position', [100, 100, 800, 600]);
histogram(rms_errors_transient, 'Normalization', 'pdf', 'BinWidth', 0.01);
title('Histogram of Transient to Steady State RMS Errors (0-8 seconds)', 'FontSize', 14);
xlabel('RMS Error (m)', 'FontSize', 12);
ylabel('Frequency', 'FontSize', 12);
grid on;

% Fit a Gaussian distribution to the Transient to Steady State RMS errors
mu_transient = mean(rms_errors_transient);
sigma_transient = std(rms_errors_transient);

% Plot the Gaussian curve
hold on;
x_values_transient = linspace(min(rms_errors_transient), max(rms_errors_transient), 100);
gaussian_curve_transient = (1/(sigma_transient*sqrt(2*pi))) * exp(-((x_values_transient-mu_transient).^2)/(2*sigma_transient^2));
plot(x_values_transient, gaussian_curve_transient, 'r-', 'LineWidth', 2);
legend('RMS Error Distribution', 'Fitted Gaussian Curve', 'FontSize', 10);

% Plot histogram of Steady State Movement RMS errors
figure('Position', [100, 100, 800, 600]);
histogram(rms_errors_steady_state, 'Normalization', 'pdf', 'BinWidth', 0.01);
title('Histogram of Steady State RMS Errors (8-30 seconds)', 'FontSize', 14);
xlabel('RMS Error (m)', 'FontSize', 12);
ylabel('Frequency', 'FontSize', 12);
grid on;

% Fit a Gaussian distribution to the Steady State Movement RMS errors
mu_steady_state = mean(rms_errors_steady_state);
sigma_steady_state = std(rms_errors_steady_state);

% Plot the Gaussian curve
hold on;
x_values_steady_state = linspace(min(rms_errors_steady_state), max(rms_errors_steady_state), 100);
gaussian_curve_steady_state = (1/(sigma_steady_state*sqrt(2*pi))) * exp(-((x_values_steady_state-mu_steady_state).^2)/(2*sigma_steady_state^2));
plot(x_values_steady_state, gaussian_curve_steady_state, 'r-', 'LineWidth', 2);
legend('RMS Error Distribution', 'Fitted Gaussian Curve', 'FontSize', 10);
