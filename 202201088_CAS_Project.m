% Simulation Parameters
time_step = 0.1; % Time step (s)
total_time = 30; % Total simulation time (s)
time_points = 0:time_step:total_time;
% Arena and Balloons (Static Positions)
arena_size = 50; % Arena dimensions (50x50x10 meters)
num_balloons = 10; % Number of balloons
balloon_positions = [arena_size * rand(num_balloons, 2), 5 * rand(num_balloons, 1)]'; % Random balloon positions (column format)
balloon_status = ones(num_balloons, 1); % 1 = unburst, 0 = burst
% Drone Physical Properties
mass = 1.5; % Mass (kg)
g = 9.81; % Gravity (m/s^2)
I = diag([0.02, 0.02, 0.04]); % Moment of inertia matrix (kg·m^2)
% Drone Initial Conditions
position = [0; 0; 2]; % Initial position (column vector, m)
velocity = [0; 0; 0]; % Initial velocity (column vector, m/s)
orientation = [0; 0; 0]; % Initial roll, pitch, yaw (column vector, rad)
angular_velocity = [0; 0; 0]; % Initial angular velocity (column vector, rad/s)
% Control Gains (Increased gain to improve movement)
kp_position = 1.0; % Increased Proportional gain for position control
kp_velocity = 2.5; % Proportional gain for velocity control
% Storage Arrays
drone_positions = zeros(length(time_points), 3); % Row format for plotting
burst_balloons = []; % Store burst balloon positions
% Simulation Loop
for t_idx = 1:length(time_points)
% Current Time
t = time_points(t_idx);
% Find nearest unburst balloon
unburst_balloons = find(balloon_status == 1);
if isempty(unburst_balloons)
disp('All balloons burst! Ending simulation.');
break;
end
distances = vecnorm(balloon_positions(:, unburst_balloons) - position, 2, 1);
[min_distance, closest_idx] = min(distances);
target_balloon = balloon_positions(:, unburst_balloons(closest_idx)); % Extract position as column vector
% Debugging: Print distance to closest balloon
disp(['Time: ', num2str(t), 's | Distance to closest balloon: ', num2str(min_distance)]);
% Check if the drone bursts the balloon
if min_distance < 3.0 % Burst threshold increased to 1 meter
balloon_status(unburst_balloons(closest_idx)) = 0; % Mark balloon as burst
burst_balloons = [burst_balloons, target_balloon]; % Log burst position
disp(['Balloon burst at time ', num2str(t), 's at position ', mat2str(target_balloon')]);
continue; % Skip to next time step after bursting
end
% Position Control (Proportional control to approach balloon)
position_error = target_balloon - position;
desired_velocity = kp_position * position_error;
% Velocity Control (Proportional)
velocity_error = desired_velocity - velocity;
force = mass * (g * [0; 0; 1] + kp_velocity * velocity_error); % Include gravity compensation
% Update Translational Dynamics (Acceleration and Velocity)
acceleration = force / mass;
velocity = velocity + acceleration * time_step;
position = position + velocity * time_step;
% Rotational Dynamics (Keep drone level and yaw aligned with target)
target_yaw = atan2(position_error(2), position_error(1));
yaw_error = target_yaw - orientation(3);
angular_acceleration = I \ [0; 0; yaw_error]; % Simplified rotational control
angular_velocity = angular_velocity + angular_acceleration * time_step;
orientation = orientation + angular_velocity * time_step;
% Debugging: Output drone velocity and position
disp(['Drone Position: ', mat2str(position')]);
disp(['Drone Velocity: ', mat2str(velocity')]);
% Store Drone Position (convert to row for storage)
drone_positions(t_idx, :) = position';
end
% Visualization
figure('Name', 'Drone Balloon Bursting Simulation');
% 3D Arena Plot
subplot(1, 2, 1);
scatter3(balloon_positions(1, :), balloon_positions(2, :), balloon_positions(3, :), 100, 'g', 'filled');
hold on;
if ~isempty(burst_balloons)
scatter3(burst_balloons(1, :), burst_balloons(2, :), burst_balloons(3, :), 100, 'r', 'filled');
end
plot3(drone_positions(:, 1), drone_positions(:, 2), drone_positions(:, 3), 'b-');
title('Drone Trajectory and Balloons');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Unburst Balloons', 'Burst Balloons', 'Drone Trajectory');
grid on;
% Distance to Target Over Time
subplot(1, 2, 2);
distances_to_balloons = zeros(length(time_points), 1);
for t_idx = 1:length(time_points)
active_balloons = balloon_status == 1;
if any(active_balloons)
distances_to_balloons(t_idx) = min(vecnorm(balloon_positions(:, active_balloons) - drone_positions(t_idx, :)', 2, 1));
else
distances_to_balloons(t_idx) = NaN; % No balloons left
end
end
plot(time_points, distances_to_balloons, 'r-');
title('Distance to Closest Balloon');
xlabel('Time (s)'); ylabel('Distance (m)');
grid on;