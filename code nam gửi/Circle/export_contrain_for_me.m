% giả sử sim_time, ctrl_all, a_input đã có sẵn
num_followers = size(ctrl_all,1);
violations = zeros(length(sim_time), num_followers);
for j = 1:num_followers
    violations(:,j) = vecnorm([squeeze(ctrl_all(j,1,:))'/a_input; ...
                               squeeze(ctrl_all(j,2,:))'/a_input], 1);
end

% Lưu vào file mat
save('InputConstraints_followers.mat', 'sim_time', 'violations');
