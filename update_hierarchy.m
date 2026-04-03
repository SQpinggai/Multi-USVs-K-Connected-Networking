function new_hierarchy = update_hierarchy(A, hierarchy, k)
    % 获取无人艇的数量
    num_agents = size(A, 1);
    
    % 初始化新的层级值为当前层级
    new_hierarchy = hierarchy;

    % 第一阶段: 保持层级值为 0 和 1 的无人艇不变
    for index = 1:num_agents
        if hierarchy(index) <= k
            % 保持层级值为 0 和 1 的无人艇不变
            new_hierarchy(index) = hierarchy(index);
        end
    end

    % 第二阶段: 更新层级值大于 1 的无人艇
    for index = 1:num_agents
        if hierarchy(index) > k
            % 检查邻居
            neighbors = find(A(index, :)); 
            neighbor_hierarchy = hierarchy(neighbors);
            
            % 包含自身的层级并排序，从小到大排序
            levels_to_sort = [hierarchy(index); neighbor_hierarchy]; % 包括自身
            sorted_levels = sort(levels_to_sort);  % 从小到大排序
            
            % 获取前k个层级值
            top_k_values = sorted_levels(1:min(k, numel(sorted_levels)));
            
            % 检查自身层级是否在前k个中
            if ~ismember(hierarchy(index), top_k_values)
                % 如果不在前k个中，根据前k个的最大值加一
                new_level = max(top_k_values) + 1;
                new_hierarchy(index) = new_level;  % 更新层级
            end
        end
    end
end
