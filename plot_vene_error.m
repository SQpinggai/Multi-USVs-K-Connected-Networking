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

width = 0.1;    %线条粗细
observer_errors = zeros(num,1);
num_steps = numel(Time); 
%% 挨个绘制 u一致性误差
 legend_entries = cell(1, N);
    figure('Position', [100 100 800 220]);
    hold on; % 在每个图窗中确保所有轨迹绘制在同一个图中
for i0 = 1:N
    for i1 = 1:num_steps

vene_errors(i1, :) = store_states{i0}(i1, 3)- tar_p0(i1, 3);
% vene_errors(i1, :) = store_virtual_vene{i0}(i1, 1)- tar_p0(i1, 3);
% vene_errors(i1, :) = store_states{i0}(i1, 3)-store_virtual_vene{i0}(i1, 1);
    end

    % 绘制每个无人艇的 'u', 'v', 'phi' 轨迹
    plot(Time, vene_errors(1:num_steps), 'Color', colors(i0, :)); % 'u' 轨迹
    hold on
    legend_entries{i0} =  plot(NaN, NaN, '-', 'Color', colors(i0, :), 'MarkerSize', 12, 'LineWidth', 1.5, 'DisplayName', sprintf('i = %d', i0));
end
    
    % 设置图例，每个图窗的图例只显示一次
    lgd = legend([legend_entries{:}], 'NumColumns', 5); % 设置图例，并将 NumColumns 改为 1
    grid on
    % 设置图形属性
    set(gca,'FontSize',12);
set(gca,'FontName','Times New Roman');
set(lgd,'FontSize',12);
set(lgd,'FontName','Times New Roman');
    xlim([0 480]);
    xlabel('${t (s)}$','Interpreter','latex');
    ylabel('$q_{i,1} - q_{0,1}   (m/s)$', 'Interpreter', 'latex');

    %% v一致性误差
    legend_entries = cell(1, N);
    figure('Position', [100 100 800 220]);
    hold on; % 在每个图窗中确保所有轨迹绘制在同一个图中
for i0 = 1:N
    for i1 = 1:num_steps

vene_errors(i1, :) = store_states{i0}(i1, 4)- tar_p0(i1, 4);
% vene_errors(i1, :) = store_virtual_vene{i0}(i1, 2)- tar_p0(i1, 4);
% vene_errors(i1, :) = store_states{i0}(i1, 4)-store_virtual_vene{i0}(i1, 2);
    end

    % 绘制每个无人艇的 'u', 'v', 'phi' 轨迹
    plot(Time, vene_errors(1:num_steps), 'Color', colors(i0, :)); % 'u' 轨迹
    hold on
    legend_entries{i0} =  plot(NaN, NaN, '-', 'Color', colors(i0, :), 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', sprintf('i = %d', i0));
end
    
    % 设置图例，每个图窗的图例只显示一次
%     legend([legend_entries{:}], 'NumColumns', 5); % 设置图例，并将 NumColumns 改为 1
    grid on
    % 设置图形属性
    set(gca,'FontSize',12);
    xlim([0 480]);
    xlabel('${t (s)}$','Interpreter','latex');
    ylabel('$q_{i,2} - q_{0,2}   (m/s)$', 'Interpreter', 'latex')

%% 观测误差统计分析

% safety_radius = epsilon; % 最大误差距离
% num_steps = numel(Time); 
% time_steps = 1:num_steps;% 时间步数
% % 初始化观测误差矩阵: [时间点 × 舰船数量]
% observer_errors_all = zeros(num_steps, N);
% 
% % 计算每艘艇的观测误差
% for i0 = 1:N 
%     for i1 = 1:num_steps
%         observer_errors_all(i1, i0) = pdist2(broadcast_history{i0}(i1, 1:2), store_states{i0}(i1, 1:2),'euclidean');
%     end
% end
% 
% % 统计每一时刻的误差最大值、平均值和标准差
% max_errors = max(observer_errors_all, [], 2);
% mean_errors = mean(observer_errors_all, 2);
% std_errors = std(observer_errors_all, 0, 2);
% 
% % 绘图
% figure('Position', [100 100 800 240]);
% hold on;
% 
% % 平均值曲线
% h_mean = plot(Time(1:num_steps), mean_errors, '-', 'Color', [0.31, 0.71, 0.02], 'LineWidth', 1.5);
% 
% % 最大值曲线
% h_max = plot(Time(1:num_steps), max_errors, '-', 'Color', [1.00, 0.41, 0.16], 'LineWidth', 2);
% 
% % 标准差阴影区域
% % h_std = fill([Time; flipud(Time)], ...
% %              [mean_errors + std_errors; flipud(mean_errors - std_errors)], ...
% %              [0.4, 0.71, 0], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
% 
% upper = mean_errors + std_errors;
% lower = mean_errors - std_errors;
% h_std =fill([Time, fliplr(Time)], ...
%      [upper; flipud(lower)], ... % 使用 flipud 反转列向量
%      [0.7 0.7 0.9], 'FaceAlpha',0.5, 'EdgeColor','none');
%  % 安全阈值
%     h_safety = plot([0 480], [safety_radius safety_radius], 'k:', 'LineWidth',1.5);
% 
% % 图例
% legend([h_mean, h_max, h_std, h_safety], ...
%        {'Mean Error', 'Max Error', 'Standard Deviation Range', 'Maxmum allowable Observer Error'}, ...
%        'Location', 'northwest');
% 
% xlabel('${t (s)}$', 'Interpreter', 'latex');
% ylabel(sprintf('Observation Error $(m)$', i0, i0), 'Interpreter', 'latex');
% 
% 
% grid on;
% set(gca, 'FontName', 'Times New Roman');
% set(gca, 'FontSize', 12);
% xlim([Time(1), Time(num_steps)]);






