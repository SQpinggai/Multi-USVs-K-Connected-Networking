function [control_output, s] = calculate_control_output(states, last_broadcast_states, stack_q,hierarchy, stack_F,stack_G,A, b, etar,detar, stack_tao_0, stack_v0_hat,d_v0_hat, index)
global max_tao max_heading l s_max s_r obstacle;
global R_sen R_c k;

p_i = states{index}(1:2);  % 当前无人艇的位置信息
q_i = states{index}(3:4);  % 当前无人艇的速度信息


% % % % 加如避障 欠驱滑膜改
% H = diag([1, 1.5]);  % 原有的控制力最小化项
% alpha = 0.1;  % 惩罚项系数，越大则控制输入的变化率越小
% H = H + alpha * eye(2);  % 将控制变化率的惩罚项加入H矩阵
% f = -2 * alpha * stack_tao_0{index};  % 对应的线性项
% d_ij = 30;
ah = [0;0];
dV_p = [0;0];
% N = numel(last_broadcast_states);

% 邻居索引
neighbors = find(A(index, :));
% 根据邻居索引提取邻居的层级
neighbor_hierarchy = hierarchy(neighbors);
% 对邻居按层级进行排序
[sorted_hierarchy, sorting_indices] = sort(neighbor_hierarchy);
sorted_neighbors = neighbors(sorting_indices);


% 只选择层级值前 k 个的邻居 强联通保持
for ind=1:k
    j = sorted_neighbors(ind);
    %  通信保持
    p_j = last_broadcast_states{j}(1:2);
    q_j = last_broadcast_states{j}(3:4);
    norm_pio = norm(p_i-p_j);
    w_connect = A(index, j) * (2*R_sen - norm_pio -R_c)*(norm_pio -R_c)  / (norm_pio*((R_sen -norm_pio)^2));
    w_avoide = A(index, j) * (norm_pio + R_c -2*R_sen)  / (norm_pio*((norm_pio - R_c)^3));
    w_ij= w_connect + w_avoide;
    dV_p = dV_p + w_ij * (p_i - p_j);

    %  速度一致性
    ah = ah + A(index, j) * (q_i- q_j);
    % ah = ah + w_ij * (stack_q{index}(1:2) - stack_q{j}(1:2));% 感觉不是很好用

end

for j2=sorted_neighbors(k+1:end)
    %  通信保持
    p_j = last_broadcast_states{j2}(1:2);
    q_j = last_broadcast_states{j2}(3:4);
    norm_pio = norm(p_i-p_j);
    w_connect = A(index, j2);
    w_connect =0 ;
    w_avoide = A(index, j2) * (norm_pio + R_c -2*R_sen)  / (norm_pio*((norm_pio - R_c)^3));
    w_ij= w_connect + w_avoide;
    dV_p = dV_p + w_ij * (p_i - p_j);

    %  速度一致性
    ah = ah + A(index, j2) * (q_i - q_j);
    % ah = ah + w_ij * (stack_q{index}(1:2) - stack_q{j}(1:2));% 感觉不是很好用

end

alpha = ah + b(index) * (q_i - detar(1:2));
dV_p_leader =  2*(p_i - etar(1:2));
dV_p = dV_p + b(index) * dV_p_leader;

%% 避障
v_avoid_alpha = [0;0];
dV_avoid = [0;0];
for j=1:numel(obstacle)
    p_o = obstacle{j}(1:2);
    v_o = obstacle{j}(3:4);  % 障碍物速度
    R_dan = obstacle{j}(5)+2;
    norm_pio = norm(p_i-p_o);
    p_io = p_i - p_o;
    q_io = q_i - v_o;
    p_j_hat = [0;0];
    if norm_pio <= R_dan+20
        miu = R_dan/norm_pio;
        alpha_k = (p_i - p_o)/norm_pio;
        P = eye(2) - alpha_k * alpha_k';
        p_j_hat = miu * p_i + eye(2)*(1-miu)*p_o;
        norm_pij = norm(p_i-p_j_hat);
        vene_j = P * q_i;
        w_avoide = ((-2)/ (norm_pio * (norm_pio -R_dan)^3));
%         w_avoide = (-2)/(norm_pij *(1+norm_pij)^3);


    else

        w_avoide = 0;
        vene_j = q_i;
    end

    dV_avoid = dV_avoid + w_avoide * (p_i - p_o);
    v_avoid_alpha = v_avoid_alpha + w_avoide * (q_i - vene_j);
end


k2 = 10;
% k2 = 0.5;% 太小艏向摇晃
s = stack_q{index}-stack_v0_hat{index};
% tao = inv(stack_G{index})*(-stack_F{index}  +d_v0_hat{index}- k*s - tanh(s) -dV_p - 2*alpha);
% tao = inv(stack_G{index})*(-stack_F{index}  -dV_p - 2*alpha);
k1 = diag([5 5]);
k2 = diag([1; 0.5]);
tao = inv(stack_G{index})*(-stack_F{index} -dV_p - k1*dV_avoid - k2* v_avoid_alpha  - 4*alpha);
%  无人艇直勾勾跟：是饱和限制太低跟不上，外加势场偏导引力太大，最后只能直线追击
% 将饱和限制改成150 就可以跟上啦
control_output = tao;
control_output = sat(control_output,max_tao,-max_tao, max_heading, -max_heading);


end