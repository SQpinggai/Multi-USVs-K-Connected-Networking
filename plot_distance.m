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



%% 绘制艇间距离 参数设置
safety_radius = R_c; % 最小安全距离
time_steps = 1:numel(Time); % 时间步数

% 初始化统计矩阵
min_distances = zeros(numel(time_steps), 1);
mean_distances = zeros(numel(time_steps), 1);
std_distances = zeros(numel(time_steps), 1);

for t = 1:numel(time_steps)
    positions = cell2mat(cellfun(@(x) x(t,1:2), follower, 'UniformOutput', false)');
    
    % 计算所有艇间距离
    D = pdist(positions);
    valid_dists = D(D > 0); % 排除自身距离
    
    % 统计指标
    min_distances(t) = min(valid_dists);
    mean_distances(t) = mean(valid_dists);
    std_distances(t) = std(valid_dists);
end
%% 艇间距离
figure('Position', [100 100 800 240]);
h_mean = plot(Time, mean_distances, '-','Color',[0.31, 0.71, 0.02], 'LineWidth', 1.5);
hold on;
h_min = plot(Time, min_distances, '-', 'Color',[1.00,0.41,0.16], 'LineWidth', 2);
h_std = fill([Time fliplr(Time)],...
            [mean_distances+std_distances; fliplr(mean_distances-std_distances)]',...
            [0.4, 0.71, 0], 'FaceAlpha',0.2, 'EdgeColor','none');

% 安全阈值参考线
h_thresh = plot([Time(1) Time(end)], [safety_radius safety_radius],...
               ':','Color',[0.64, 0.08, 0.18], 'LineWidth',1.5, 'DisplayName','安全阈值');


legend([h_mean, h_min, h_std,h_thresh],...
       {'Mean Distance', 'Minimum Distance', 'Standard Deviation Range','Minimum Collision-Avoidance Distance'},...
       'Location', 'northwest');
xlim([Time(1), Time(end)]);  % 确保 x 轴适应实际的数据长度

xlabel('${t(s)}$','Interpreter','latex');
ylabel({'Distance between ', 'vehicles (m)'}, 'Interpreter', 'latex');

grid on;
set(gca,'FontName','Times New Roman');
set(gca,'FontSize',12);


%% 避障
%% 避障分析 - 分阶段独立绘图

%% 避障分析 - 使用标准差分析
% 
% % 仿真参数
% dt = 0.01; 
% safety_margin = 2; 
% 
% % 阶段定义 定义截取的时间段和障碍物序号
% phases = {
%     struct('t_range',[30,70],   'obs',3);
%     struct('t_range',[100,140], 'obs',3);
%     struct('t_range',[310,380], 'obs',[4,5]);
%     struct('t_range',[425,465], 'obs',2);
% };
% 
% for p = 1:4
%     % 新建独立figure 定义宽高为500*300 
%     fig = figure('Position',[100, 100, 500, 300], 'Color','w');
%     
%     % 阶段参数提取
%     phase = phases{p};
%     t_start = phase.t_range(1);
%     t_end = phase.t_range(2);
%     obs_ids = phase.obs;
%     
%     % 时间步转换
%     step_start = floor(t_start/dt) + 1;
%     step_end = floor(t_end/dt);
%     step_start = 1;
%     step_end = num;
%     steps = step_start:step_end;
%     time_axis = (steps-1)*dt;  
%     
%     % 初始化存储
%     min_dists = zeros(length(steps),1);
%     mean_dists = zeros(length(steps),1);
%     std_dists = zeros(length(steps),1);
%     
%     % 数据计算
%     for s = 1:length(steps)
%         step = steps(s);
%         usv_pos = cell2mat(cellfun(@(x)x(step,1:2), follower, 'UniformOutput',false)');
%         
%         all_dists = [];
%         for o = obs_ids
%             obst_data = store_obstacles{o}(step,:);
%             obst_pos = obst_data(1:2);
%             obst_radius = obst_data(5);
%             dists = pdist2(usv_pos, obst_pos);
%             all_dists = [all_dists; dists(:)];
%         end
%         
%         min_dists(s) = min(all_dists);
%         mean_dists(s) = mean(all_dists);
%         std_dists(s) = std(all_dists);
%     end
%     
%     % ===== 核心绘图 =====
%     hold on;
%     
%     % 标准差范围
% %     obs_std = fill([time_axis fliplr(time_axis)], ...
% %         [mean_dists+std_dists; fliplr(mean_dists-std_dists)]', ...
% %         [0.7 0.7 0.9], 'FaceAlpha',0.5, 'EdgeColor','none');
% upper = mean_dists + std_dists;
% lower = mean_dists - std_dists;
% obs_std =fill([time_axis, fliplr(time_axis)], ...
%      [upper; flipud(lower)], ... % 使用 flipud 反转列向量
%      [0.7 0.7 0.9], 'FaceAlpha',0.5, 'EdgeColor','none');
%     
%     % 均值曲线
%     obs_mean = plot(time_axis, mean_dists, 'b-', 'LineWidth',2);
%     
%     % 最小值曲线
%     obs_min = plot(time_axis, min_dists, 'r--', 'LineWidth',2);
%     
%     % 安全阈值
%     obs_safety = plot([0 500], [obst_radius obst_radius], 'k:', 'LineWidth',1.5);
%     
% 
%     
%     % ===== 坐标轴 =====
%     xlim([t_start-2, t_end+2]);
%     ylim([5, 50]);
%     xlabel('${t (s)}$','Interpreter','latex');
%     ylabel({'Distance to obstacles (m)'}, 'Interpreter', 'latex');
%     grid on;
%     set(gca, 'FontSize',12);
%     set(gca,'FontName','Times New Roman');
% 
%     % ===== 图例设置 =====
%         legend([obs_mean, obs_min, obs_std, obs_safety], ...
%         {'Mean Distance', 'Minimum Distance', 'Standard Deviation Range', 'Minimum Obstacle Avoidance Distance'}, ...
%         'Location', 'northwest');
%     legend('FontSize',12);
%     
%     % 保存图表（可选）
%     % print(fig, sprintf('Phase%d.png',p), '-dpng', '-r300');
% end