function [stack_eta, stack_v] = Initialize_USV_positions(num_boats, R_min, R_max, x_range, y_range)
   

    % 初始化存储无人艇位置和速度的cell数组
    stack_eta = cell(1, num_boats);
    stack_v = cell(1, num_boats);

    % 随机生成第一艘无人艇的位置
    stack_eta{1} = [randi(x_range), randi(y_range), rand * 2 * pi];
    stack_v{1} = [0, 0, 0];

    for i = 2:num_boats
        discard = true; % 初始设为true以进入循环

        while discard
            % 随机生成位置
            random_x = randi(x_range);
            random_y = randi(y_range);

            % 构建新的位置信息
            new_eta = [random_x, random_y, rand * 2 * pi];
            new_v = [0, 0, 0];

            % 判断碰撞危险条件
            discard = false;
            for j = 1:i-1
                % 只计算 x 和 y 的距离
                distance = norm(new_eta(1:2) - stack_eta{j}(1:2));
                if distance < R_min + 20
                    discard = true;
                    break;
                end
            end

            % 判断通信关系条件
            if ~discard % 只有当碰撞危险条件不满足时才进行通信关系判断
                at_least_one_communication = false;
                for j = 1:i-1
                    % 只计算 x 和 y 的距离
                    distance = norm(new_eta(1:2) - stack_eta{j}(1:2));
                    if distance <= R_max - 20
                        at_least_one_communication = true;
                        break;
                    end
                end
                if ~at_least_one_communication
                    discard = true;
                end
            end
        end
        % 如果符合条件，则保存
        stack_eta{i} = new_eta;
        stack_v{i} = new_v;
    end

%     % 可视化结果
%     figure;
%     hold on;
%     for i = 1:num_boats
%         plot(stack_eta{i}(1), stack_eta{i}(2), 'bo');
%         text(stack_eta{i}(1) + 10, stack_eta{i}(2) + 10, sprintf('Boat %d', i));
%     end
%     xlabel('X Coordinate');
%     ylabel('Y Coordinate');
%     title('Initial Positions of Unmanned Boats');
%     grid on;
%     hold off;
end

