function [target_positions, movements, target_relations, hierarchy, dangerous_positions, step_records] = establish_k_connectivity(CRV, stack_eta, r, k)
N = numel(stack_eta);

for i = 1:N
    initial_positions(i, 1:2) = stack_eta{i}(1:2);
end

final_positions = initial_positions; 
moved = false(N, 1);
hierarchy = -1*ones(N,1);
target_relations = [];
dangerous_positions = [];
step_records = struct();
step_count = 0;

center = mean(initial_positions, 1);
center = CRV(1:2)';
[~, start_index] = min(pdist2(initial_positions, center, 'euclidean'));
moved(start_index) = true;
final_positions(start_index,:) = center;
hierarchy(start_index) = 0;

step_count = step_count + 1;
step_records(step_count).positions = final_positions;
step_records(step_count).hierarchy = hierarchy;
step_records(step_count).target_relations = target_relations;
step_records(step_count).dangerous_positions = dangerous_positions;

[final_positions, moved, hierarchy, target_relations] = ...
    form_initial_clique_v2(final_positions, start_index, r, k, moved, hierarchy, target_relations);

step_count = step_count + 1;
step_records(step_count).positions = final_positions;
step_records(step_count).hierarchy = hierarchy;
step_records(step_count).target_relations = target_relations;
step_records(step_count).dangerous_positions = dangerous_positions;

while sum(moved) < N
    remaining_indices = find(~moved);
    moved_indices = find(moved);

    D = pdist2(final_positions(remaining_indices, :), final_positions(moved_indices, :), 'euclidean');

    [min_distances, nearest_moved_indices] = min(D, [], 2);
    [~, nearest_index_in_remaining] = min(min_distances);

    a_index = moved_indices(nearest_moved_indices(nearest_index_in_remaining));
    v_index = remaining_indices(nearest_index_in_remaining);

    target_relations = [target_relations; v_index, a_index];
    
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

    if numel(selected) < k
        remaining_candidates = setdiff(moved_indices, selected);
        [~, supplement] = pdist2(final_positions(remaining_candidates, :),...
                          final_positions(v_index, :),...
                          'euclidean', 'Smallest', k-numel(selected));
        selected = [selected; remaining_candidates(supplement)'];
    end
    nearest_k_indices = selected(1:k);

    new_member_hierarchy = max(hierarchy(nearest_k_indices)) + 1;
    hierarchy(v_index) = new_member_hierarchy;

    for j = 1:k
        target_relations = [target_relations; v_index, nearest_k_indices(j)];
    end

    valid_moved = final_positions(moved,:);
    [final_positions(v_index, :), dangerous_position] = move_to_connect_qp(...
        final_positions(nearest_k_indices, :), final_positions(v_index, :), r, ...
        valid_moved);
    
    if ~isempty(dangerous_position)
        dangerous_positions(end + 1, :) = dangerous_position;
    end

    moved(v_index) = true;

    step_count = step_count + 1;
    step_records(step_count).positions = final_positions;
    step_records(step_count).hierarchy = hierarchy;
    step_records(step_count).target_relations = target_relations;
    step_records(step_count).dangerous_positions = dangerous_positions;
end

for index=1:N
    target_positions{index} = final_positions(index,:)';
end

movements = sqrt(sum((final_positions - initial_positions).^2, 2));
end

function [positions, moved, hierarchy, target_relations] = form_initial_clique_v2(positions, start_index, r, k, moved, hierarchy, target_relations)
global obstacle R_c;

function collision = is_collision(pos)
    collision = false;
    for o = 1:numel(obstacle)
        if norm(pos - obstacle{o}(1:2)') < (obstacle{o}(5) + R_c)
            collision = true;
            break;
        end
    end
end

function new_pos = safe_homothetic(start, current, r)
    max_attempts = 5;
    base_scale = 0.8;
    safety_margin = 0.1*r;
    
    direction = current - start;
    ideal_distance = r * base_scale;
    
    for attempt = 1:max_attempts
        scale = base_scale - 0.1*(attempt-1);
        scale = max(scale, 0.3);
        
        candidate = start + scale * direction;
        
        if norm(candidate - start) > r - safety_margin
            continue;
        end
        
        if ~is_collision(candidate)
            new_pos = candidate;
            return;
        end
    end
    
    new_pos = start + 0.3 * direction; 
end

start_pos = positions(start_index, :);
[~, nearest_k_indices] = pdist2(positions, start_pos, 'euclidean', 'Smallest', k+1);
nearest_k_indices(nearest_k_indices == start_index) = [];

valid_directions = [];
for i = 1:length(nearest_k_indices)
    candidate = nearest_k_indices(i);
    dir_vector = positions(candidate,:) - start_pos;
    angle = atan2(dir_vector(2), dir_vector(1));
    
    if ~is_collision(positions(candidate,:))
        valid_directions = [valid_directions; candidate];
    end
    if length(valid_directions) >= k
        break;
    end
end

if length(valid_directions) < k
    remaining = setdiff(nearest_k_indices, valid_directions);
    valid_directions = [valid_directions; remaining(1:k-length(valid_directions))];
end

for i = 1:k
    idx = valid_directions(i);
    original_pos = positions(idx, :);
    
    positions(idx, :) = safe_homothetic(start_pos, original_pos, r);
    
    if pdist2(positions(idx,:), start_pos) > r
        positions(idx,:) = start_pos + (positions(idx,:)-start_pos)/...
            pdist2(positions(idx,:), start_pos)*r*0.9;
    end
    
    moved(idx) = true;
    hierarchy(idx) = i;
    target_relations = [target_relations; idx, start_index];
end

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

direction = current_pos - start_pos;
distance = norm(direction);

if distance > r
    scale = r / distance;
else
    scale = 1;
end

new_position = start_pos + scale * direction;
end

function [new_position, dangerous_pos] = move_to_connect_qp(neighbor_pos, current_pos, r, moved_positions)
global obstacle R_c;

dangerous_pos = [];
options = optimoptions('fmincon','Display','off','Algorithm','sqp');

valid_moved = moved_positions(all(~isnan(moved_positions),2),:); 

try
    basic_nonlcon = @(p) deal(...
        arrayfun(@(i) (p(1)-neighbor_pos(i,1))^2 + (p(2)-neighbor_pos(i,2))^2 - r^2,...
        (1:size(neighbor_pos,1))'),...
        []);
    
    temp_pos = fmincon(@(p) sum((p-current_pos).^2), current_pos,...
        [], [], [], [], [], [], basic_nonlcon, options);
    
    if ~isempty(obstacle)
        obstacle_violation = arrayfun(@(i) norm(temp_pos - obstacle{i}(1:2)') < (obstacle{i}(5)+R_c),...
            1:numel(obstacle));
        if any(obstacle_violation)
            dangerous_pos = temp_pos;
        end
    end

    full_nonlcon = @(p) deal(...
        [...
        arrayfun(@(i) (p(1)-neighbor_pos(i,1))^2 + (p(2)-neighbor_pos(i,2))^2 - r^2,...
            (1:size(neighbor_pos,1))');
        arrayfun(@(i) (obstacle{i}(5)+R_c)^2 - (p(1)-obstacle{i}(1))^2 - (p(2)-obstacle{i}(2))^2,...
            (1:numel(obstacle))');
        arrayfun(@(i) (2*R_c)^2 - (p(1)-valid_moved(i,1))^2 - (p(2)-valid_moved(i,2))^2,...
            (1:size(valid_moved,1))')...
        ],...
        []);
    
    new_position = fmincon(@(p) sum((p-temp_pos).^2), temp_pos,...
        [], [], [], [], [], [], full_nonlcon, options);

catch
    new_position = temp_pos;
    if isempty(new_position)
        new_position = current_pos;
    end
end

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

if final_violation && isempty(dangerous_pos)
    dangerous_pos = new_position;
end
end