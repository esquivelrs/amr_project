close all
clear

% Load ROS bag file
BAG_FILENAME = '2024-02-21-19-04-28.bag';
bagselect = rosbag(BAG_FILENAME);

% Select messages related to pose ground truth
bSel_gt = select(bagselect, 'Topic', '/bluerov2/pose_gt');
msgStructs_gt = readMessages(bSel_gt, 'DataFormat', 'struct');
time_gt = bSel_gt.MessageList.Time;

quat_x_gt = cellfun(@(m) double(m.Pose.Pose.Orientation.X), msgStructs_gt);
quat_y_gt = cellfun(@(m) double(m.Pose.Pose.Orientation.Y), msgStructs_gt);
quat_z_gt = cellfun(@(m) double(m.Pose.Pose.Orientation.Z), msgStructs_gt);
quat_w_gt = cellfun(@(m) double(m.Pose.Pose.Orientation.W), msgStructs_gt);

eul_gt = quat2eul([quat_w_gt quat_x_gt, quat_y_gt, quat_z_gt],"XYZ");
roll_gt = eul_gt(:,1);
pitch_gt = eul_gt(:,2);
yaw_gt = eul_gt(:,3);

yaw_rate_gt = cellfun(@(m) double(m.Twist.Twist.Angular.Z), msgStructs_gt);

% Select messages related to pose reference
bSel_pose_ref = select(bagselect, 'Topic', '/bluerov2/cmd_pose');
msgStructs_pose_ref = readMessages(bSel_pose_ref, 'DataFormat', 'struct');
time_pose_ref = bSel_pose_ref.MessageList.Time;

quat_x_pose_ref = cellfun(@(m) double(m.Pose.Orientation.X), msgStructs_pose_ref);
quat_y_pose_ref = cellfun(@(m) double(m.Pose.Orientation.Y), msgStructs_pose_ref);
quat_z_pose_ref = cellfun(@(m) double(m.Pose.Orientation.Z), msgStructs_pose_ref);
quat_w_pose_ref = cellfun(@(m) double(m.Pose.Orientation.W), msgStructs_pose_ref);

eul_pose_ref = quat2eul([quat_w_pose_ref quat_x_pose_ref, quat_y_pose_ref, quat_z_pose_ref],"XYZ");
roll_pose_ref = eul_pose_ref(:,1);
pitch_pose_ref = eul_pose_ref(:,2);
yaw_pose_ref = eul_pose_ref(:,3);


% Select messages related to pose reference
bSel_vel_ref = select(bagselect, 'Topic', '/bluerov2/cmd_vel');
msgStructs_vel_ref = readMessages(bSel_vel_ref, 'DataFormat', 'struct');
time_vel_ref = bSel_vel_ref.MessageList.Time;

yaw_rate_ref = cellfun(@(m) double(m.Angular.Z), msgStructs_vel_ref);

% Plot 

figure;
plot(time_gt, yaw_gt, 'b');
hold on;
plot(time_pose_ref, yaw_pose_ref, 'r');
title('Yaw control');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Ground Truth', 'Reference');
grid on;
hold off;

figure;
plot(time_gt, yaw_rate_gt, 'b');
hold on;
plot(time_vel_ref, yaw_rate_ref, 'r');
title('Yaw rate control');
xlabel('Time (s)');
ylabel('Angle Rate (rad/s)');
legend('Ground Truth', 'Reference');
grid on;
hold off;


