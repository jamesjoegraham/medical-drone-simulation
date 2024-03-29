clear all;
close all;
clc;

bag = rosbag("after_tweak.bag");
bag.AvailableTopics;
bagselect1 = select(bag,"Topic", "/ground_truth_to_tf/pose");
bagselect2 = select(bag,"Topic", "/current_force");

% msgs = readMessages(bagselect2, 1:10, 'DataFormat', 'struct')
% msgs2 = readMessages(bagselect1, 1:10, 'DataFormat', 'struct')

ts_position = timeseries(bagselect1,"Pose.Position.X", "Pose.Position.Y", "Pose.Position.Z");
ts_orientation = timeseries(bagselect1,"Pose.Orientation.W","Pose.Orientation.X", "Pose.Orientation.Y", "Pose.Orientation.Z");
ts_wrench_force = timeseries(bagselect2, "Wrench.Force.X", "Wrench.Force.Y", "Wrench.Force.Z");
ts_wrench_torque = timeseries(bagselect2, "Wrench.Torque.X", "Wrench.Torque.Y", "Wrench.Torque.Z");

[yaw, pitch, roll] = quat2angle(ts_orientation.Data);
orientation_deg = [pitch, roll].*180./pi;
ts_orientation.Data = orientation_deg;


figure(1);
subplot(2,1,1);
grid on;
hold on;
plot(ts_position)
legend("X Position", "Y position", "Z Position");
title("XYZ Position of Quadrotor in Trial");
xlabel("Time (s)");
ylabel("Position (m)");


subplot(2,1,2);
grid on;
hold on;
plot(ts_orientation)
legend("Pitch", "Roll");
title("Roll and Pitch of Quadrotor During Trial");
xlabel("Time (s)");
ylabel("Orientation (deg)");

figure(2);
% subplot(2,1,1);
grid on;
hold on;
plot(ts_wrench_force)
legend("Force X", "Force Y", "Force Z");
title("Force Applied to Quadrotor During Trial");
xlabel("Time (s)");
ylabel("Force (N)");

% subplot(2,1,2);
grid on;
hold on;
plot(ts_wrench_torque)
legend("Torque X", "Torque Y", "Torque Z");
title("Torque Applied to Quadrotor During Trial");
xlabel("Time (s)");
ylabel("Torque (Nm)");

% % Calculating xyz velocity
% dxdt = gradient(ts_position.Data(:,1))./gradient(ts_position.Time);
% dydt = gradient(ts_position.Data(:,2))./gradient(ts_position.Time);
% dzdt = gradient(ts_position.Data(:,3))./gradient(ts_position.Time);
% % dxdt = gradient(ts_position.Data(:,1))
% % dydt = gradient(ts_position.Data(:,2))
% % dzdt = gradient(ts_position.Data(:,3))
% ts_velocity = timeseries([dxdt dydt dzdt], ts_position.Time);
% 
% figure(2);
% hold on;
% grid on;
% plot(ts_velocity)
% legend("X Velocity", "Y Velocity", "Z Velocity");
% title("Velocity of Quadrotor During Trial");
% xlabel("Time (s)");
% ylabel("Velocity (m/s)");
% 
% % tring to calculate derivative of pitch and roll (angular velocity)
% dydt_pitch = gradient(pitch)./gradient(ts_orientation.Time);
% dydt_roll = gradient(roll)./gradient(ts_orientation.Time);
% ts_derivative = timeseries([dydt_pitch dydt_roll], ts_orientation.Time);
% 
% %tryin to calculate angular acceleration
% dydt2_pitch = gradient(dydt_pitch)./gradient(ts_orientation.Time);
% dydt2_roll = gradient(dydt_roll)./gradient(ts_orientation.Time);
% ts_derivative2 = timeseries([dydt2_pitch dydt2_roll], ts_orientation.Time);
% 
% figure(3);
% subplot(2,1,1);
% grid on;
% hold on;
% plot(ts_derivative);
% legend("Pitch Velocity", "Roll Velocity");
% title("Velocity of Roll and Pitch During Trial");
% xlabel("Time (s)");
% ylabel("Angular Velocity(deg/s)");
% 
% subplot(2,1,2);
% grid on;
% hold on;
% plot(ts_derivative2);
% legend("Pitch Accelertaion", "Roll Acceleration");
% title("Acceleration of Roll and Pitch During Trial");
% xlabel("Time (s)");
% ylabel("Angular Acceleration(deg/s^2)");

