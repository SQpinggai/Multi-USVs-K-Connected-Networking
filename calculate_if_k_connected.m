function is_k_connected = calculate_if_k_connected(stack_eta, k, r)
    n = numel(stack_eta);


    adj_matrix = zeros(n);
    for i = 1:n
        for j = i+1:n
            if pdist2(stack_eta{i}(1:2)', stack_eta{j}(1:2)', 'euclidean') <= r
                adj_matrix(i, j) = 1;
                adj_matrix(j, i) = 1;
            end
        end
    end

    
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
    n = size(adj_matrix, 1);
    flow = 0;
    residual_graph = adj_matrix;

    while true
        [parent, found] = bfs(residual_graph, s, t);
        if ~found
            break;
        end

        path_flow = inf;
        v = t;
        while v ~= s
            u = parent(v);
            path_flow = min(path_flow, residual_graph(u, v));
            v = u;
        end

        v = t;
        while v ~= s
            u = parent(v);
            residual_graph(u, v) = residual_graph(u, v) - path_flow;
            residual_graph(v, u) = residual_graph(v, u) + path_flow;
            v = u;
        end

        flow = flow + path_flow;
    end
end

function [parent, found] = bfs(residual_graph, s, t)
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