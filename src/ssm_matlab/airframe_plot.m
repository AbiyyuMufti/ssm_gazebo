
pose_missile = out.pose_missile.signals.values;
% pose_missile2d = out.pose_missile_2d.signals.values;

% plot(pose_missile(1:end,1), pose_missile(1:end,3));
plot3(pose_missile(1:end,1), pose_missile(1:end,2), pose_missile(1:end,3));

% hold on;
% plot(pose_missile(1:end,1), pose_missile(1:end,3));
% plot(pose_missile2d(1:end,1), pose_missile2d(1:end,2));
grid on;
axis equal;