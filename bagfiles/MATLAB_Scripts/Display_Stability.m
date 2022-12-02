clear all;
close all;
clc;

bag = rosbag("full_mass_final_test.bag");
bag.AvailableTopics;
bagselect1 = select(bag,"Topic", "/ground_truth_to_tf/pose");

ts_position = timeseries(bagselect1,"Pose.Position.X", "Pose.Position.Y", "Pose.Position.Z");
ts_orientation = timeseries(bagselect1,"Pose.Orientation.W","Pose.Orientation.X", "Pose.Orientation.Y", "Pose.Orientation.Z");

[yaw, pitch, roll] = quat2angle(ts_orientation.Data);
orientation_deg = [pitch, roll].*180./pi;
ts_orientation.Data = orientation_deg;

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

% Calculating xyz velocity
dxdt = gradient(ts_position.Data(:,1))./gradient(ts_position.Time);
dydt = gradient(ts_position.Data(:,2))./gradient(ts_position.Time);
dzdt = gradient(ts_position.Data(:,3))./gradient(ts_position.Time);
ts_velocity = timeseries([dxdt dydt dzdt], ts_position.Time);

% Calculating xyz acceleration
% Filtering out inf values and downsampling while taking the average so
% that acceleration can be accuratly calculated
Time_x = []
Time_y = []
Time_z = []
dxdt_fix = [];
dydt_fix = [];
dzdt_fix = [];
filt = 15;
for i=1:size(dxdt,1)
    if ~isinf(dxdt(i))
        dxdt_fix = [dxdt_fix dxdt(i)];
        Time_x = [Time_x ts_position.Time(i)];
    end
    if ~isinf(dydt(i))
        dydt_fix = [dydt_fix dydt(i)];
        Time_y = [Time_y ts_position.Time(i)];
    end
    if ~isinf(dzdt(i))
        dzdt_fix = [dzdt_fix dzdt(i)];
        Time_z = [Time_z ts_position.Time(i)];
    end
end
n = 30;
dxdt_mean = mean(reshape([dxdt_fix zeros(1, mod(-numel(dxdt_fix),n))],n,[]));
Time_xmean = mean(reshape([Time_x nan(1, mod(-numel(Time_x),n))],n,[]));
dydt_mean = mean(reshape([dydt_fix nan(1, mod(-numel(dydt_fix),n))],n,[]));
Time_ymean = mean(reshape([Time_y nan(1, mod(-numel(Time_y),n))],n,[]));
dzdt_mean = mean(reshape([dzdt_fix nan(1, mod(-numel(dzdt_fix),n))],n,[]));
Time_zmean = mean(reshape([Time_z nan(1, mod(-numel(Time_z),n))],n,[]));
        
dxdt2 = gradient(dxdt_mean)./gradient(Time_xmean);
dydt2 = gradient(dydt_mean)./gradient(Time_ymean);
dzdt2 = gradient(dzdt_mean)./gradient(Time_zmean);
% ts_acceleration = timeseries([dxdt2 dydt2 dzdt2], ts_position.Time);

figure(2);
% subplot(2,1,1);
hold on;
grid on;
plot(ts_position.Time, [dxdt, dydt, dzdt]);
legend("X Velocity", "Y Velocity", "Z Velocity");
title("Velocity of Quadrotor During Trial");
xlabel("Time (s)");
ylabel("Velocity (m/s)");

% plotting acceleration
figure(3)
subplot(3,1,1);
hold on;
grid on;
plot(Time_xmean, dxdt2)
title("X Acceleration of Quadrotor During Trial");
xlabel("Time (s)");
ylabel("Accelertaion (m/s^2)");

subplot(3,1,2);
hold on;
grid on;
plot(Time_ymean, dydt2)
title("Y Acceleration of Quadrotor During Trial");
xlabel("Time (s)");
ylabel("Accelertaion (m/s^2)");

subplot(3,1,3);
hold on;
grid on;
plot(Time_zmean, dzdt2);
title("Z Acceleration of Quadrotor During Trial");
xlabel("Time (s)");
ylabel("Accelertaion (m/s^2)");


% tring to calculate derivative of pitch and roll (angular velocity)
dydt_pitch = gradient(pitch)./gradient(ts_orientation.Time);
dydt_roll = gradient(roll)./gradient(ts_orientation.Time);
ts_derivative = timeseries([dydt_pitch dydt_roll], ts_orientation.Time);

%tryin to calculate angular acceleration
dydt2_pitch = gradient(dydt_pitch)./gradient(ts_orientation.Time);
dydt2_roll = gradient(dydt_roll)./gradient(ts_orientation.Time);
ts_derivative2 = timeseries([dydt2_pitch dydt2_roll], ts_orientation.Time);

figure(4);
% subplot(2,1,1);
grid on;
hold on;
plot(ts_derivative);
legend("Pitch Velocity", "Roll Velocity");
title("Velocity of Roll and Pitch During Trial");
xlabel("Time (s)");
ylabel("Angular Velocity(deg/s)");

% subplot(2,1,2);
% grid on;
% hold on;
% plot(ts_derivative2);
% legend("Pitch Accelertaion", "Roll Acceleration");
% title("Acceleration of Roll and Pitch During Trial");
% xlabel("Time (s)");
% ylabel("Angular Acceleration(deg/s^2)");

