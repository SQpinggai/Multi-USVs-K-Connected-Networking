function is_k_connected = calculate_if_k_connected(stack_eta, k, r)
    % 检查给定的位置信息是否构成k连通图
    % 返回值为 true 表示 k 连通，否则为 false

    % 获取无人艇数量
    n = numel(stack_eta);

    % 基于通信范围 r 创建邻接矩阵
    adj_matrix = zeros(n);
    for i = 1:n
        for j = i+1:n
            if pdist2(stack_eta{i}(1:2)', stack_eta{j}(1:2)', 'euclidean') <= r
                adj_matrix(i, j) = 1;
                adj_matrix(j, i) = 1;
            end
        end
    end

    % 使用最大流算法检查k连通性
    is_connected = true;
    for i = 1:n
        for j = i+1:n
            if max_flow(adj_matrix, i, j) < k
                is_connected = false;
                return;
            end
        end
    end
end

function flow = max_flow(adj_matrix, s, t)
    % 使用 Ford-Fulkerson 算法计算从节点 s 到节点 t 的最大流
    n = size(adj_matrix, 1);
    flow = 0;
    residual_graph = adj_matrix;

    while true
        % 使用 BFS 查找增广路径
        [parent, found] = bfs(residual_graph, s, t);
        if ~found
            break;
        end

        % 找到路径中的最小流量
        path_flow = inf;
        v = t;
        while v ~= s
            u = parent(v);
            path_flow = min(path_flow, residual_graph(u, v));
            v = u;
        end

        % 更新残余图
        v = t;
        while v ~= s
            u = parent(v);
            residual_graph(u, v) = residual_graph(u, v) - path_flow;
            residual_graph(v, u) = residual_graph(v, u) + path_flow;
            v = u;
        end

        % 增加总流量
        flow = flow + path_flow;
    end
end

function [parent, found] = bfs(residual_graph, s, t)
    % 使用广度优先搜索 (BFS) 在残余图中查找增广路径
    n = size(residual_graph, 1);
    visited = false(n, 1);
    parent = -ones(n, 1);

    queue = [s];
    visited(s) = true;

    found = false;
    while ~isempty(queue)
        u = queue(1);
        queue(1) = [];

        for v = 1:n
            if ~visited(v) && residual_graph(u, v) > 0
                queue = [queue, v];
                visited(v) = true;
                parent(v) = u;
                if v == t
                    found = true;
                    return;
                end
            end
        end
    end
end