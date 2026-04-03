function A = calculate_communication_matrix(stack_eta, A, R_max, R_min)
    % 创建通信关系矩阵 A
    N = length(stack_eta);  % 无人艇数量


    % 判断两个无人艇之间的通信关系
    for i = 1:N
        y_i = stack_eta{i}(1:2);  % 获取第 i 个无人艇的坐标

        for j = 1:N
            if j ~= i
                y_j = stack_eta{j}(1:2);  % 获取第 j 个无人艇的坐标
                norm_pij = norm(y_i - y_j);  % 计算距离
                
                % 如果距离小于 R_max - 5，则标记为存在通信关系
                if norm_pij <= R_max+5
                    A(i, j) = 1;  % 存在通信关系
                else
                    A(i, j) = 0;  % 不存在通信关系
                end

                % 如果距离小于 R_min，则输出节点详细信息
                if norm_pij <= R_min
                    fprintf('Node %d is close to Node %d at distance %d\n', i, j, floor(norm_pij));
                end
            end
        end
    end
end
