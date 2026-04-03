hold off;
close all;

colors = [
    0.0000    0.4470    0.7410; % 深蓝色
    0.8500    0.3250    0.0980; % 橙色
    0.9290    0.6940    0.1250; % 明黄色
    0.4940    0.1840    0.5560; % 深紫色
    0.4660    0.6740    0.1880; % 明绿色
    0.3010    0.7450    0.9330; % 天蓝色
    0.6350    0.0780    0.1840; % 深红色
    0.6480    0.8150    0.2390; % 浅绿色
    0.9290    0.3880    0.1050; % 土黄色
    0.6000    0.6000    0.8000; % 灰色
];

% plot_observer_error; % 绘制event-based neighbors position observer error
% plot_ETM;
% plot_distance; %绘制无人艇之间距离 绘制无人艇与障碍物之间距离

% %% 新版全局轨迹+避障时刻+层级表现+拓扑关系（整合版）
% figure(10003)
% set(gcf, 'Position', [100, 100, 500, 300]);
% hold on; % 确保所有无人艇的轨迹绘制在同一个图中
% legend_entries = cell(1, N);
% plot_handles = gobjects(1, N);
% 
% % %leader
% % p1 = plot(tar_state(:,1), tar_state(:,2), '--r', 'LineWidth', 1);
% % hold on
% % 
% % %follower
% % for i = 1:N
% %     plot_handles(i) = plot(follower{i}(:, 1), follower{i}(:, 2), 'Color', colors(i, :), 'LineWidth', 0.5);
% %     hold on;
% %     legend_entries{i} = sprintf('Follower %d', i);
% % end
% 
% start_step = 14000;
% end_step = 15000;
% step_interval = 500;
% 
% % 颜色定义
% k = 3; % 你的k连通值
% radius = 2; % 圆圈半径
% arrow_len = 1.5; % 箭头参数
% arrow_width = 0.6;
% 
% % Leader轨迹
% p1 = plot(tar_state(:,1), tar_state(:,2), '--r', 'LineWidth', 1);
% hold on
% 
% plot_handles = gobjects(1, N);
% legend_entries = cell(1, N);
% % Follower Final
% for i = 1:N
%     plot_handles(i) = plot(follower{i}(:,1), follower{i}(:,2), 'Color', colors(i, :), 'LineWidth', 1);
% %     [xx, yy] = plot_ship_contour(follower{i}(num, :)', 6, 2, 4);
% %     fill(xx, yy, colors(i, :));
%     legend_entries{i} = sprintf('Follower %d', i); % 存储图例条目
% end
% 
% % ========== 新增：存储图形对象 ==========
% connection_lines = gobjects(0); % 存储连接线
% hierarchy_texts = gobjects(0); % 存储层级文本
% ship_contours = gobjects(0); % 存储船体轮廓
% 
% 
% 
% % ========== 关键修改部分：在指定时刻添加层级和连接 ==========
% % plot_timesteps = [5000,  21500, 44500]; % 你的五个示例时刻
% %------------太集中了，交错画两次
% plot_timesteps = [ 13000, 36500]; % 你的五个示例时刻
% for loop = plot_timesteps
%     % 获取当前时刻数据
%     current_hierarchy = hier_history{loop};
%     Adj_matrix = A_history{loop};
%     positions = zeros(N, 2); % 初始化位置矩阵
%     for i1 = 1:N
%         positions(i1, :) = follower{i1}(loop + 1, 1:2); % 提取每个无人艇的位置
%     end
% 
% 
%     for i = 1:N
%         current_level = current_hierarchy(i); % 当前无人艇的层级
%         neighbor_indices = find(Adj_matrix(i, :));% 所有邻居
%         % 筛选出比自己层级高的邻居（值更小的）
%         higher_neighbors = neighbor_indices(current_hierarchy(neighbor_indices) < current_level);
%         
%         if ~isempty(higher_neighbors)
%             % 计算层级差值
%             level_diffs = current_level - current_hierarchy(higher_neighbors);
% 
%             % 按差值升序排列（差值小表示层级接近）
%             [~, sorted_idx] = sort(level_diffs);
%             sorted_neighbors = higher_neighbors(sorted_idx);
% 
%             % 选择前k个最近层级邻居
%             important_neighbors = sorted_neighbors(1:min(k, length(sorted_neighbors)));
%         else
%             important_neighbors = [];
%         end
% 
%         % 绘制连接线（箭头）
%         for j = 1:numel(neighbor_indices)
%             neighbor_index = neighbor_indices(j);
%             neighbor_pos = positions(neighbor_index, :);
%             if ismember(neighbor_index, important_neighbors)
%                 % 绘制重要邻居连接线（单向箭头）
%                 %                 draw_arrow1(neighbor_pos, positions(i, :), radius+0.5);
%                 line = plot([positions(i,1), neighbor_pos(1)],...
%                     [positions(i,2), neighbor_pos(2)],...
%                     '-.','Color', [0.54,0.84,0.08], 'LineWidth', 1.5);
%                 connection_lines(end+1) = line;
%             else
%                connection_lines(end+1) = line;
%             end
%         end
%     end
%     % ===== 绘制无人艇轮廓和层级 =====
%     for i = 1:N
%         % 绘制船体轮廓
%         [xx, yy] = plot_ship_contour(follower{i}(loop, :)', 6, 2, 4);
%         contour = fill(xx, yy, colors(i, :), 'EdgeColor', 'none');
%         ship_contours(end+1) = contour;
% 
%         % 添加层级文本
%         txt = text(positions(i,1), positions(i,2), num2str(current_hierarchy(i)),...
%             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
%             'FontSize', 14, 'FontName','Times New Roman', 'Color', 'k', 'FontWeight', 'bold');
%         hierarchy_texts(end+1) = txt;
%     end
% end
% 
% %% 绘制障碍物的运动轨迹
% obs_handle = scatter(NaN, NaN, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.8 0.8 0.8],'LineWidth', 0.4);
% obstacle_handles = gobjects(1, length(obstacle)); % 初始化障碍物图例句柄数组
% for i = 1:numel(obstacle)
%     for t = start_step:step_interval:end_step
%         obstacleCenter = store_obstacles{i}(t, 1:2); % 当前时间步的中心
%         obstacleRadius = store_obstacles{i}(t, 5); % 当前时间步的半径
% 
%         % 生成圆的坐标
%         theta = linspace(0, 2*pi, 100);
%         x = obstacleRadius * cos(theta) + obstacleCenter(1);
%         y = obstacleRadius * sin(theta) + obstacleCenter(2);
%         plot(x, y, 'k-', 'LineWidth', 1);
%     end
% end
% %% 绘制动态避障主要时刻
% for i = 1:3
%     obstacleCenter = obstacle{i}(1:2); % 障碍物中心的坐标
%     obstacleRadius = obstacle{i}(5); % 障碍物的半径
% 
%     % 画出实心圆代表的障碍物
%     theta = linspace(0, 2*pi, 100); % 创建一系列角度
%     x = obstacleRadius * cos(theta) + obstacleCenter(1); % 计算 x 坐标
%     y = obstacleRadius * sin(theta) + obstacleCenter(2); % 计算 y 坐标
%     obstacle_handles(i) = fill(x, y, [0.8 0.8 0.8]); % 绘制填满的实心圆，颜色为红色
%     obstacle_legend_entries = sprintf('障碍物 '); % 存储障碍物图例条目
%     hold on; % 保持图形打开，以便继续添加内容
% end
% %% 四号动态障碍物 
%     obstacleCenter = store_obstacles{4}(15300, 1:2); % 障碍物中心的坐标
%     obstacleRadius = store_obstacles{4}(15300, 5); % 障碍物的半径
% 
%     % 画出实心圆代表的障碍物
%     theta = linspace(0, 2*pi, 100); % 创建一系列角度
%     x = obstacleRadius * cos(theta) + obstacleCenter(1); % 计算 x 坐标
%     y = obstacleRadius * sin(theta) + obstacleCenter(2); % 计算 y 坐标
%     obstacle_handles(4) = fill(x, y, [0.8 0.8 0.8]); % 绘制填满的实心圆，颜色为红色
%     %     obstacle_handles(i) = plot(x, y, 'ro', 'MarkerFaceColor', 'r'); % 绘制实心圆
%     obstacle_legend_entries = sprintf('障碍物 '); % 存储障碍物图例条目
%     hold on; % 保持图形打开，以便继续添加内容
% %% 五号动态障碍物
%     obstacleCenter = store_obstacles{5}(16300, 1:2); % 障碍物中心的坐标
%     obstacleRadius = store_obstacles{5}(16300, 5); % 障碍物的半径
% 
%     % 画出实心圆代表的障碍物
%     theta = linspace(0, 2*pi, 100); % 创建一系列角度
%     x = obstacleRadius * cos(theta) + obstacleCenter(1); % 计算 x 坐标
%     y = obstacleRadius * sin(theta) + obstacleCenter(2); % 计算 y 坐标
%     obstacle_handles(4) = fill(x, y, [0.8 0.8 0.8]); % 绘制填满的实心圆，颜色为红色
%     %     obstacle_handles(i) = plot(x, y, 'ro', 'MarkerFaceColor', 'r'); % 绘制实心圆
%     obstacle_legend_entries = sprintf('障碍物 '); % 存储障碍物图例条目
%     hold on; % 保持图形打开，以便继续添加内容
% 
%     
% 
% % ========== 调整图层顺序 ==========
% % 连接线置于底层
% arrayfun(@(x) uistack(x, 'bottom'), connection_lines);
% % 船体轮廓在中层
% arrayfun(@(x) uistack(x, 'up'), ship_contours); 
% % 文本置于顶层
% arrayfun(@(x) uistack(x, 'top'), hierarchy_texts);
% 
% % ========== 保持原有障碍物绘制 ==========
% % 绘制障碍物的运动轨迹（原有代码）
% for i = 1:numel(obstacle)
%     for t = start_step:step_interval:end_step
%         % ...（保持原有障碍物绘制代码不变）...
%     end
% end
% 
% % ========== 优化图例和坐标轴 ==========
% 
% lgd =legend([p1, plot_handles, obs_handle], [{'Leader'}, legend_entries, {'Obstacle'}], 'NumColumns', 5);
% axis equal
% margin = 1.5;
% xlim([min(cellfun(@(x) min(x(:, 1)), follower)) - margin, max(cellfun(@(x) max(x(:, 1)), follower)) + margin]);
% ylim([min(cellfun(@(x) min(x(:, 2)), follower)) - margin, max(cellfun(@(x) max(x(:, 2)), follower)) + 40]);
% 
% 
% % 设置专业学术图表样式
% set(gca,'FontSize',12);
% set(gca,'FontName','Times New Roman');
% set(lgd,'FontSize',12);
% set(lgd,'FontName','Times New Roman');
% xlabel('${x/m}$','Interpreter','latex');
% ylabel('${y/m}$','Interpreter','latex');
% 
% grid on;
