%% Load Data
clear, clc;
close all;

% Load the ROS bag
bag = rosbag('2024-12-28-data.bag');

% Extract topics
mocap_twist_msg = select(bag, 'Topic', '/mavros/local_position/velocity_body');
mocap_twist_data = readMessages(mocap_twist_msg);

mocap_imu_msg = select(bag, 'Topic', '/mavros/imu/data');
mocap_imu_data = readMessages(mocap_imu_msg);

start_time = bag.StartTime;

% Initialize arrays to store the timestamps and velocity components
times_vel = zeros(length(mocap_twist_data), 1);
vx = zeros(length(mocap_twist_data), 1);
vy = zeros(length(mocap_twist_data), 1);
vz = zeros(length(mocap_twist_data), 1);

% Initialize arrays to store the body rate (angular velocity) components
times_rate = zeros(length(mocap_imu_data), 1);
wx = zeros(length(mocap_imu_data), 1);
wy = zeros(length(mocap_imu_data), 1);
wz = zeros(length(mocap_imu_data), 1);

% Loop through each message for velocity data and extract timestamp and components
for i = 1:length(mocap_twist_data)
    times_vel(i) = (mocap_twist_data{i}.Header.Stamp.Sec + mocap_twist_data{i}.Header.Stamp.Nsec / 1e9) - start_time;
    
    vx(i) = mocap_twist_data{i}.Twist.Linear.X;
    vy(i) = mocap_twist_data{i}.Twist.Linear.Y;
    vz(i) = mocap_twist_data{i}.Twist.Linear.Z;
end

% Loop through each IMU message and extract body rates
for i = 1:length(mocap_imu_data)
    times_rate(i) = (mocap_imu_data{i}.Header.Stamp.Sec + mocap_imu_data{i}.Header.Stamp.Nsec / 1e9) - start_time;
    
    wx(i) = mocap_imu_data{i}.AngularVelocity.X;
    wy(i) = mocap_imu_data{i}.AngularVelocity.Y;
    wz(i) = mocap_imu_data{i}.AngularVelocity.Z;
end

%% plot

line_width = 1.5;

% Plot the linear velocity
figure('Position', [100, 100, 1100, 100]);
plot(times_vel, vx, 'Color', '#A50044', 'LineWidth', line_width);  % forca barca
hold on;
plot(times_vel, vy, 'Color', '#004D98', 'LineWidth', line_width);
plot(times_vel, vz, 'Color', '#EDBB00', 'LineWidth', line_width);
set(gca, 'FontName', 'Times New Roman');
%xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 8);
ylabel('Vel (m/s)', 'FontName', 'Times New Roman', 'FontSize', 8);
grid on;
axis([36 39 -1 3]); % Adjust axis limits as needed

% Plot the body rate
figure('Position', [100, 300, 1100, 100]);
plot(times_rate, wx, 'Color', '#A50044', 'LineWidth', line_width);
hold on;
plot(times_rate, wy, 'Color', '#004D98', 'LineWidth', line_width);
plot(times_rate, wz, 'Color', '#EDBB00', 'LineWidth', line_width);
set(gca, 'FontName', 'Times New Roman');
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 8);
ylabel('Rate (rad/s)', 'FontName', 'Times New Roman', 'FontSize', 8);
grid on;
axis([36 39 -4 4]); % Adjust axis limits as needed
