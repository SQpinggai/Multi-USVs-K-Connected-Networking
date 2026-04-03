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
size = 10;  %事件触发散点圆大小
width = 0.1;    %线条粗细
Min_u = T_s/hbar;
% %% 控制输入
for i0 = 1:N
    figure('Position',[100, 100, 320, 220]);
    hold on; % 在每个图窗中确保所有轨迹绘制在同一个图中

    len = num;
    tmp_triggr_instant = zeros(len,1);
    count = 1;

    for i1 = 1:1:len
        if history_broadcast_time{i0}(i1) > 0
            tmp_triggr_instant(count) = history_broadcast_time{i0}(i1);%触发的时刻
            count = count + 1;
        end
    end
    trigger_num = count - 1;% 触发次数
    triggr_instant = tmp_triggr_instant(1:trigger_num);% 触发的时刻减去多余的空白一项，得到触发的各个瞬间
    event_x = zeros(trigger_num-1,1);
    event_y = zeros(trigger_num-1,1);
    for i = 1:1:trigger_num-1
        event_x(i) =   triggr_instant(i);% 触发的时刻
        event_y(i) = triggr_instant(i+1) - triggr_instant(i);%触发的时间间隔
        if event_y(i) < Min_u
            fprintf('USV%d triggered at t=%.2fs: interval =%.3fs\n',...
            i0, event_x(i), event_y(i))
        end
    end
    
    plot([0 470], [Min_u Min_u], 'k--', 'LineWidth',0.1);

    scatter(event_x,event_y,size, 'k','filled', 'Marker', 'd');
    for i=1:1:trigger_num-1
        h_line = plot([event_x(i),event_x(i)],[0,event_y(i)],'Color', colors(i0, :),'LineWidth',width);%目测应该是画线段
        % for i=1:1:length(out)
        %
        % end
    end



    hold on
    grid on
    % 设置图形属性
    set(gca,'FontSize',12);
    xlabel('${t (s)}$','Interpreter','latex');
    %     ylabel('${\tau \ [N]}$','Interpreter','latex');
%     ylabel(sprintf('$$\\t_{k_{%d} +1}^{{%d}} - \\t_{k_{%d}}^{{%d}} (s)$$', i0,i0,i0,i0), 'Interpreter', 'latex');

end
