function plot_uav_communication(follower, hierarchy, A_history, loop, k, radius, span, store_obstacles)
    % 获取无人艇数量
    global obstacle;
    N = numel(follower);
    
    % 获取当前时刻的位置、层级信息和邻接矩阵
    positions = zeros(N, 2); % 初始化位置矩阵
    for i = 1:N
        positions(i, :) = follower{i}(loop + 1, 1:2); % 提取每个无人艇的位置
    end
    current_hierarchy = hierarchy{loop}; % 当前时间步的层级信息
    Adj_matrix = A_history{loop}; % 当前时间步的邻接矩阵

    % 创建图窗
    figure('Position', [100, 100, 800, 600]);
    hold on;
    %Obstacle
    for i = 1:3
    obstacleCenter = obstacle{i}(1:2); % 障碍物中心的坐标
    obstacleRadius = obstacle{i}(5); % 障碍物的半径

    % 画出实心圆代表的障碍物
    theta = linspace(0, 2*pi, 100); % 创建一系列角度
    x = obstacleRadius * cos(theta) + obstacleCenter(1); % 计算 x 坐标
    y = obstacleRadius * sin(theta) + obstacleCenter(2); % 计算 y 坐标
    obstacle_handles(i) = fill(x, y, [0.6,0,0]); % 绘制填满的实心圆，颜色为红色
    %     obstacle_handles(i) = plot(x, y, 'ro', 'MarkerFaceColor', 'r'); % 绘制实心圆
    obstacle_legend_entries = sprintf('障碍物 '); % 存储障碍物图例条目
    hold on; % 保持图形打开，以便继续添加内容
end
%% 四号动态障碍物 
    obstacleCenter = store_obstacles{4}(18300, 1:2); % 障碍物中心的坐标
    obstacleRadius = store_obstacles{4}(18300, 5); % 障碍物的半径

    % 画出实心圆代表的障碍物
    theta = linspace(0, 2*pi, 100); % 创建一系列角度
    x = obstacleRadius * cos(theta) + obstacleCenter(1); % 计算 x 坐标
    y = obstacleRadius * sin(theta) + obstacleCenter(2); % 计算 y 坐标
    obstacle_handles(4) = fill(x, y, [0.6,0,0]); % 绘制填满的实心圆，颜色为红色
    %     obstacle_handles(i) = plot(x, y, 'ro', 'MarkerFaceColor', 'r'); % 绘制实心圆
    obstacle_legend_entries = sprintf('障碍物 '); % 存储障碍物图例条目
    hold on; % 保持图形打开，以便继续添加内容
%% 五号动态障碍物
    obstacleCenter = store_obstacles{5}(18300, 1:2); % 障碍物中心的坐标
    obstacleRadius = store_obstacles{5}(18300, 5); % 障碍物的半径

    % 画出实心圆代表的障碍物
    theta = linspace(0, 2*pi, 100); % 创建一系列角度
    x = obstacleRadius * cos(theta) + obstacleCenter(1); % 计算 x 坐标
    y = obstacleRadius * sin(theta) + obstacleCenter(2); % 计算 y 坐标
    obstacle_handles(4) = fill(x, y, [0.6,0,0]); % 绘制填满的实心圆，颜色为红色
    %     obstacle_handles(i) = plot(x, y, 'ro', 'MarkerFaceColor', 'r'); % 绘制实心圆
    obstacle_legend_entries = sprintf('障碍物 '); % 存储障碍物图例条目
    hold on; % 保持图形打开，以便继续添加内容

    % 先检查所有无人艇的邻接关系并绘制连接线（箭头）
    for i = 1:N
        % 获取当前无人艇的邻居的层级值
        neighbor_indices = find(Adj_matrix(i, :));

        % 处理邻居存在的情况
        if ~isempty(neighbor_indices)
            neighbor_hierarchies = current_hierarchy(neighbor_indices);
            [sorted_hierarchy, sorted_indices] = sort(neighbor_hierarchies); % 排序

            % 获取前k个重要邻居
            num_neighbors = numel(sorted_hierarchy);
            important_neighbors = neighbor_indices(sorted_indices(1:min(k, num_neighbors))); 
        else
            important_neighbors = []; % 没有邻居的情况
        end

        % 绘制连接线（箭头）
        for j = 1:numel(neighbor_indices)
            neighbor_index = neighbor_indices(j);
            neighbor_pos = positions(neighbor_index, :);
            if ismember(neighbor_index, important_neighbors)
                % 绘制重要邻居连接线（单向箭头）
                draw_arrow1(neighbor_pos, positions(i, :), radius+0.5);
                % draw_arrow1(neighbor_pos, positions(i, :), radius);
            else
                % 绘制一般邻居连接线（灰色虚线）
                % plot([positions(i, 1), neighbor_pos(1)], [positions(i, 2), neighbor_pos(2)], 'k--', 'LineWidth', 1);
            end
        end
    end

    % 在这里绘制圆和文本
    circles = gobjects(N, 1); % 用于存储圆圈对象
    for i = 1:N
        % 获取无人艇位置
        pos = positions(i, :);
        % 绘制空心圆
        theta = linspace(0, 2*pi, 100); % 创建一系列角度
        x_circle = radius * cos(theta) + pos(1);
        y_circle = radius * sin(theta) + pos(2);
        % 绘制圆
        circles(i) = plot(x_circle, y_circle, 'Color', [0.39,0.83,0.07], 'LineWidth', 2); % 空心圆
        % 在圆内标注无人艇的层级信息
        text(pos(1), pos(2), num2str(current_hierarchy(i)), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'FontSize', 15, 'Color', 'k');
    end

    % 将圆圈和数字置于顶层
    for i = 1:N
        uistack(circles(i), 'top');
    end

    % 计算所有无人艇位置的均值
    mean_position = mean(positions, 1);
    
    % 设置xlim和ylim，确保图形稳定
    xlim([mean_position(1) - span/3, mean_position(1) + span/3]);
    ylim([mean_position(2) - span/2, mean_position(2) + span/2]);

    % 设置坐标轴
    axis equal; % 保持坐标轴比例
    xlabel('$x$ (m)', 'Interpreter', 'latex'); 
    ylabel('$y$ (m)', 'Interpreter', 'latex');
    % title(sprintf('UAV Cluster Communication at Loop %d', loop)); % 可以根据需要打开标题

    % 只添加三个图例项
        
        h_hierarchy = plot(nan, nan, 'o', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'none', 'DisplayName', 'Followers'); 
        h_arrow = plot(nan, nan, 'r', 'DisplayName', 'Hierarchy Arrows'); 
        
        % 使用手动指定位置的方法
    legend([ h_hierarchy, h_arrow]); % 更新为您想要的x, y位置及大小
    set(gca, 'FontSize', 14);
    hold off;
end


function draw_arrow1(start_pos, end_pos, radius)
    % 计算箭头的长度和宽度
    arrow_len = 4; % 箭头长度
    arrow_width = 2; % 箭头宽度
    % arrow_len = 1.5; % 箭头长度
    % arrow_width = 0.6; % 箭头宽度

    % 计算箭头的起始和结束坐标
    x1 = start_pos(1); y1 = start_pos(2);
    x2 = end_pos(1); y2 = end_pos(2);

    % 计算起始点和结束点沿着指向线缩进半径
    angle = atan2(y2 - y1, x2 - x1); % 计算方向角
    arrow_start = [x1 + radius * cos(angle), y1 + radius * sin(angle)]; % 起始点缩进
    arrow_end = [x2 - radius * cos(angle), y2 - radius * sin(angle)]; % 结束点缩进

    % 计算箭头的方向
    dx = arrow_end(1) - arrow_start(1);
    dy = arrow_end(2) - arrow_start(2);
    theta = atan2(dy, dx); % 方向角

    % 计算箭头的三个关键点
    Point4 = [0, 0]; Point5 = [0, 0]; Point3 = [0, 0];

    % 箭头的尖端位置
    Point4(1) = arrow_end(1) - arrow_len * cos(theta);
    Point4(2) = arrow_end(2) - arrow_len * sin(theta);

    % 箭头的两侧点
    Point3(1) = Point4(1) + arrow_width * sin(theta);
    Point3(2) = Point4(2) - arrow_width * cos(theta);

    Point5(1) = Point4(1) - arrow_width * sin(theta);
    Point5(2) = Point4(2) + arrow_width * cos(theta);

    % 定义箭头的六个点
    dot = zeros(6, 2);
    dot(1, 1) = arrow_start(1);  dot(1, 2) = arrow_start(2); % 起始点
    dot(2, 1) = Point4(1);       dot(2, 2) = Point4(2);      % 箭头尖端
    dot(3, 1) = Point3(1);       dot(3, 2) = Point3(2);      % 箭头一侧
    dot(4, 1) = arrow_end(1);    dot(4, 2) = arrow_end(2);   % 结束点
    dot(5, 1) = Point5(1);       dot(5, 2) = Point5(2);      % 箭头另一侧
    dot(6, 1) = Point4(1);       dot(6, 2) = Point4(2);      % 回到箭头尖端

    % 绘制箭头
    plot(dot(:, 1), dot(:, 2), 'r','LineWidth', 1.5); % 绘制箭头轮廓
    fill(dot(:, 1), dot(:, 2),'r','EdgeColor','none'); % 填充箭头
end