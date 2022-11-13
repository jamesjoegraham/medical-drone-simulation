clear all;
close all;
clc;

bag = rosbag("FirstAttempt.bag");
bag.AvailableTopics;
bagselect1 = select(bag,"Topic", "/ground_truth_to_tf/pose");

ts_position = timeseries(bagselect1,"Pose.Position.X", "Pose.Position.Y", "Pose.Position.Z");
ts_orientation = timeseries(bagselect1,"Pose.Orientation.X", "Pose.Orientation.Y", "Pose.Orientation.Z","Pose.Orientation.W");

[yaw, pitch, roll] = quat2angle(ts_orientation.Data);
orientation_deg = [pitch, roll].*180./pi;
ts_orientation.Data = orientation_deg

msgs = readMessages(bagselect1,"DataFormat","struct");

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