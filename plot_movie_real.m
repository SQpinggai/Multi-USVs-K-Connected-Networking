
TOANG = 180 / pi;
width2 = 1.5;
ftsize = 10;
head_ship = 5; w_ship = 2; l_ship = 2;
yaw_length = 15;
num_followers = size(stack_eta);
hold off
% 遍历所有follower
for i = 1:length(follower)
%     clf; % 清除当前图形
    state = follower{i}(1:loop, 1:3);
    tar_yaw_d_i = tar_yaw{i}(loop, :);

    % 绘制轨迹
    tar_state = tar_path(1:loop,:);
    plot(tar_state(:, 1), tar_state(:, 2), '--r', "LineWidth", width2);
    hold on
    plot(state(:, 1), state(:, 2), '-k', "LineWidth", width2);
    hold on
    % 绘制小艇

    xt = state(loop, 1);
    yt = state(loop, 2);
    yaw_t = state(loop, 3);
    phi_d = tar_yaw_d_i;
    ship = plot_ship1(xt, yt, yaw_t, l_ship, w_ship, head_ship);
    fill(ship(:, 1), ship(:, 2), '-r');

    pt1 = [xt, yt];
    pt2 = [xt + yaw_length * cos(phi_d), yt + yaw_length * sin(phi_d)];
    plot_arrow(pt1, pt2, 'r'); % 箭头

end
% 显示时间
text(0.1, 0.9, ['当前时间: ',  num2str(t)], 'Units', 'normalized');
%% 绘制障碍物
% 遍历障碍物
    for i = 1:length(obstacle)
        obstacleCenter = obstacle{i}(1:2); % 障碍物中心的坐标
        obstacleRadius = obstacle{i}(5); % 障碍物的半径
        
        % 画出圆形障碍物
        theta = linspace(0, 2*pi, 100); % 创建一系列角度
        x = obstacleRadius * cos(theta) + obstacleCenter(1); % 计算 x 坐标
        y = obstacleRadius * sin(theta) + obstacleCenter(2); % 计算 y 坐标
        plot(x, y, 'r'); % 绘制圆形障碍物
        hold on; % 保持图形打开，以便继续添加内容
    end

% drawnow;
axis equal
pause(0.00001);

