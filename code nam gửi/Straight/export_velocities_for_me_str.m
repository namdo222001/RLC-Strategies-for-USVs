% Tạo cấu trúc dữ liệu để lưu
LinearVel  = squeeze(ctrl_all(:,1,:));   % (num_followers x time)
AngularVel = squeeze(ctrl_all(:,2,:));   % (num_followers x time)

% Lưu file .mat
save('FollowersVelocityDataStr_for_me.mat', ...
     'sim_time', 'LinearVel', 'AngularVel', 'num_robots');
