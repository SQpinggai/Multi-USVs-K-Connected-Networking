function [target_positions, movements, target_relations, hierarchy, dangerous_positions, step_records] = establish_k_connectivity(CRV, stack_eta, r, k)
N = numel(stack_eta);

for i = 1:N
    initial_positions(i, 1:2) = stack_eta{i}(1:2);
end

final_positions = initial_positions; 
moved = false(N, 1); % 标记哪些无人艇已经被移动
hierarchy = -1*ones(N,1); % 层级信息
target_relations = []; % 记录通信关系
dangerous_positions = []; % 存储危险位置
step_records = struct(); % 存储每一步的变化
step_count = 0; % 记录步骤数

% 找到所有节点的几何中心
center = mean(initial_positions, 1);
center = CRV(1:2)';
% 找到距离中心最近的点作为起始点
[~, start_index] = min(pdist2(initial_positions, center, 'euclidean'));
moved(start_index) = true;
final_positions(start_index,:) = center;
hierarchy(start_index) = 0;

% 记录初始状态
step_count = step_count + 1;
step_records(step_count).positions = final_positions;
step_records(step_count).hierarchy = hierarchy;
step_records(step_count).target_relations = target_relations;
step_records(step_count).dangerous_positions = dangerous_positions;

% 选择与起始点最近的 k 个邻居，并通过同调变换将它们移动到一个位置
[final_positions, moved, hierarchy, target_relations] = ...
    form_initial_clique_v2(final_positions, start_index, r, k, moved, hierarchy, target_relations);

% 记录初始 clique 的状态
step_count = step_count + 1;
step_records(step_count).positions = final_positions;
step_records(step_count).hierarchy = hierarchy;
step_records(step_count).target_relations = target_relations;
step_records(step_count).dangerous_positions = dangerous_positions;

% 逐步扩展集群，使每个新加入的节点保持 k 连通性
while sum(moved) < N
    % 找到未加入集群的节点中距离集群最近的节点 v
    remaining_indices = find(~moved);  % 未移动节点的索引
    moved_indices = find(moved);       % 已移动节点的索引

    % 计算距离矩阵，D(i,j) 表示 remaining_indices(i) 到 moved_indices(j) 的距离
    D = pdist2(final_positions(remaining_indices, :), final_positions(moved_indices, :), 'euclidean');

    % 找到最近的已移动节点 a 和最近的未移动节点 v
    [min_distances, nearest_moved_indices] = min(D, [], 2);
    [~, nearest_index_in_remaining] = min(min_distances);

    % 获取对应的全局索引
    a_index = moved_indices(nearest_moved_indices(nearest_index_in_remaining)); % 最近的已移动节点 a 的索引
    v_index = remaining_indices(nearest_index_in_remaining); % 最近的未移动节点 v 的索引

    % 记录通信关系
    target_relations = [target_relations; v_index, a_index];
    
    %---新添加------------------------%
    % 层级连续邻居选择
    h_a = hierarchy(a_index);
    target_hs = max(0, h_a-(k-1)):h_a;
    
    selected = [];
    for ht = target_hs
        candidates = moved_indices(hierarchy(moved_indices) == ht);
        if ~isempty(candidates)
            [~, closest] = min(pdist2(final_positions(candidates, :), final_positions(v_index, :)));
            selected = [selected; candidates(closest)];
        end
        if numel(selected) >= k, break; end
    end
    % 补充机制
    if numel(selected) < k
        remaining_candidates = setdiff(moved_indices, selected);
        [~, supplement] = pdist2(final_positions(remaining_candidates, :),...
                          final_positions(v_index, :),...
                          'euclidean', 'Smallest', k-numel(selected));
        selected = [selected; remaining_candidates(supplement)'];
    end
    nearest_k_indices = selected(1:k);



    %----------------------------%
%     % 找到 v 的最近 k 个邻居（包括 a 自身）
%     [~, nearest_k_indices_in_moved] = pdist2(final_positions(moved_indices, :), final_positions(v_index, :), 'euclidean', 'Smallest', k);
%     % 将这些邻居转换为全局索引
%     nearest_k_indices = moved_indices(nearest_k_indices_in_moved);
    
    % 计算新成员的层级值
    new_member_hierarchy = max(hierarchy(nearest_k_indices)) + 1;  % 找到最大的层级并加1
    hierarchy(v_index) = new_member_hierarchy; % 更新新成员的层级

    % 记录通信关系
    for j = 1:k
        target_relations = [target_relations; v_index, nearest_k_indices(j)];
    end

    % 修改后（增加NaN过滤）：
valid_moved = final_positions(moved,:);
% valid_moved(any(isnan(valid_moved),:)) = []; % 清除无效位置
[final_positions(v_index, :), dangerous_position] = move_to_connect_qp(...
    final_positions(nearest_k_indices, :), final_positions(v_index, :), r, ...
    valid_moved);
    
    % 如果存在危险位置，存储该位置
    if ~isempty(dangerous_position)
        dangerous_positions(end + 1, :) = dangerous_position;  % 假设以行加
    end

    moved(v_index) = true; % 标记 v 为已移动

    % 记录当前步骤的状态
    step_count = step_count + 1;
    step_records(step_count).positions = final_positions;
    step_records(step_count).hierarchy = hierarchy;
    step_records(step_count).target_relations = target_relations;
    step_records(step_count).dangerous_positions = dangerous_positions;
end

for index=1:N
    target_positions{index} = final_positions(index,:)';
end

% 计算每个无人艇的移动距离
movements = sqrt(sum((final_positions - initial_positions).^2, 2));
end

function [positions, moved, hierarchy, target_relations] = form_initial_clique_v2(positions, start_index, r, k, moved, hierarchy, target_relations)
global obstacle R_c;

% 辅助函数：碰撞检测
function collision = is_collision(pos)
    collision = false;
    for o = 1:numel(obstacle)
        if norm(pos - obstacle{o}(1:2)') < (obstacle{o}(5) + R_c)
            collision = true;
            break;
        end
    end
end

% 改进的位似变换（带避障）
function new_pos = safe_homothetic(start, current, r)
    max_attempts = 5;    % 最大调整次数
    base_scale = 0.8;    % 初始缩放系数
    safety_margin = 0.1*r;% 安全余量
    
    direction = current - start;
    ideal_distance = r * base_scale;
    
    for attempt = 1:max_attempts
        % 计算当前缩放比例
        scale = base_scale - 0.1*(attempt-1);
        scale = max(scale, 0.3); % 最小缩放限制
        
        % 计算候选位置
        candidate = start + scale * direction;
        
        % 检查通信距离和碰撞
        if norm(candidate - start) > r - safety_margin
            continue; % 跳过不满足通信要求的
        end
        
        if ~is_collision(candidate)
            new_pos = candidate;
            return;
        end
    end
    
    % 最终仍碰撞则强制放置（后续步骤处理）
    new_pos = start + 0.3 * direction; 
end

% 主逻辑保持不变，仅替换变换函数
start_pos = positions(start_index, :);
[~, nearest_k_indices] = pdist2(positions, start_pos, 'euclidean', 'Smallest', k+1);
nearest_k_indices(nearest_k_indices == start_index) = [];

% 方向筛选（新增）
valid_directions = [];
for i = 1:length(nearest_k_indices)
    candidate = nearest_k_indices(i);
    dir_vector = positions(candidate,:) - start_pos;
    angle = atan2(dir_vector(2), dir_vector(1));
    
    % 排除障碍物方向（示例筛选逻辑）
    if ~is_collision(positions(candidate,:))
        valid_directions = [valid_directions; candidate];
    end
    if length(valid_directions) >= k
        break;
    end
end

% 补足数量
if length(valid_directions) < k
    remaining = setdiff(nearest_k_indices, valid_directions);
    valid_directions = [valid_directions; remaining(1:k-length(valid_directions))];
end

% 应用安全位似变换
for i = 1:k
    idx = valid_directions(i);
    original_pos = positions(idx, :);
    
    % 使用改进后的变换函数
    positions(idx, :) = safe_homothetic(start_pos, original_pos, r);
    
    % 二次修正（确保与起点连接）
    if pdist2(positions(idx,:), start_pos) > r
        positions(idx,:) = start_pos + (positions(idx,:)-start_pos)/...
            pdist2(positions(idx,:), start_pos)*r*0.9;
    end
    
    moved(idx) = true;
    hierarchy(idx) = i;
    target_relations = [target_relations; idx, start_index];
end

% 建立基底节点互连（新增）
for i = 1:k
    for j = i+1:k
        if pdist2(positions(valid_directions(i),:),...
                 positions(valid_directions(j),:)) <= r
            target_relations = [target_relations;...
                valid_directions(i), valid_directions(j)];
        end
    end
end
end

function new_position = homothetic_transform(start_pos, current_pos, r)

% 计算当前节点相对于起始点的方向
direction = current_pos - start_pos;
distance = norm(direction);

% 计算缩放因子，使得移动后的节点与起始点的距离符合通信半径
if distance > r
    scale = r / distance;
else
    scale = 1;
end

% 计算新位置：起始点 + 缩放后的方向向量
new_position = start_pos + scale * direction;
end

function [new_position, dangerous_pos] = move_to_connect_qp(neighbor_pos, current_pos, r, moved_positions)
global obstacle R_c;

% 初始化参数
dangerous_pos = [];
options = optimoptions('fmincon','Display','off','Algorithm','sqp');

% 过滤已移动节点（排除自身）
valid_moved = moved_positions(all(~isnan(moved_positions),2),:); 

try
    % 阶段1：原始方案（仅考虑邻居约束）
    basic_nonlcon = @(p) deal(...
        arrayfun(@(i) (p(1)-neighbor_pos(i,1))^2 + (p(2)-neighbor_pos(i,2))^2 - r^2,...
        (1:size(neighbor_pos,1))'),...
        []);
    
    temp_pos = fmincon(@(p) sum((p-current_pos).^2), current_pos,...
        [], [], [], [], [], [], basic_nonlcon, options);
    
    % 记录原始危险位置（仅检测障碍物）
    if ~isempty(obstacle)
        obstacle_violation = arrayfun(@(i) norm(temp_pos - obstacle{i}(1:2)') < (obstacle{i}(5)+R_c),...
            1:numel(obstacle));
        if any(obstacle_violation)
            dangerous_pos = temp_pos; % 独立记录原始方案危险位置
        end
    end

    % 阶段2：安全方案（全约束）
    full_nonlcon = @(p) deal(...
        [% 邻居约束
        arrayfun(@(i) (p(1)-neighbor_pos(i,1))^2 + (p(2)-neighbor_pos(i,2))^2 - r^2,...
            (1:size(neighbor_pos,1))');
        % 障碍物约束（含R_c）
        arrayfun(@(i) (obstacle{i}(5)+R_c)^2 - (p(1)-obstacle{i}(1))^2 - (p(2)-obstacle{i}(2))^2,...
            (1:numel(obstacle))');
        % 有效成员约束 
        arrayfun(@(i) (2*R_c)^2 - (p(1)-valid_moved(i,1))^2 - (p(2)-valid_moved(i,2))^2,...
            (1:size(valid_moved,1))')...
        ],...
        []);
    
    new_position = fmincon(@(p) sum((p-temp_pos).^2), temp_pos,...
        [], [], [], [], [], [], full_nonlcon, options);

catch
    % 异常处理优先使用阶段1结果
    new_position = temp_pos;
    if isempty(new_position)
        new_position = current_pos;
    end
end

% 最终安全验证（不覆盖dangerous_pos）
final_violation = false;
if ~isempty(obstacle)
    final_violation = final_violation || ...
        any(arrayfun(@(i) norm(new_position - obstacle{i}(1:2)') < (obstacle{i}(5)+R_c*0.9),...
        1:numel(obstacle)));
end
if ~isempty(valid_moved)
    final_violation = final_violation || ...
        any(sqrt(sum((new_position - valid_moved).^2,2)) < 2*R_c*0.9);
end

% 仅当原始方案未标记且最终位置危险时补充标记
if final_violation && isempty(dangerous_pos)
    dangerous_pos = new_position;
end
end
