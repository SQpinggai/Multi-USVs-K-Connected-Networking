function [control_output, s] = calculate_control_output(states, last_broadcast_states, stack_q,hierarchy, stack_F,stack_G,A, b, etar,detar, stack_tao_0, stack_v0_hat,d_v0_hat, index)
global max_tao max_heading l s_max s_r obstacle;
global R_sen R_c k;

p_i = states{index}(1:2);  
q_i = states{index}(3:4);  


ah = [0;0];
dV_p = [0;0];



neighbors = find(A(index, :));

neighbor_hierarchy = hierarchy(neighbors);

[sorted_hierarchy, sorting_indices] = sort(neighbor_hierarchy);
sorted_neighbors = neighbors(sorting_indices);



for ind=1:k
    j = sorted_neighbors(ind);
    p_j = last_broadcast_states{j}(1:2);
    q_j = last_broadcast_states{j}(3:4);
    norm_pio = norm(p_i-p_j);
    w_connect = A(index, j) * (2*R_sen - norm_pio -R_c)*(norm_pio -R_c)  / (norm_pio*((R_sen -norm_pio)^2));
    w_avoide = A(index, j) * (norm_pio + R_c -2*R_sen)  / (norm_pio*((norm_pio - R_c)^3));
    w_ij= w_connect + w_avoide;
    dV_p = dV_p + w_ij * (p_i - p_j);
    ah = ah + A(index, j) * (q_i- q_j);
end

for j2=sorted_neighbors(k+1:end)
    p_j = last_broadcast_states{j2}(1:2);
    q_j = last_broadcast_states{j2}(3:4);
    norm_pio = norm(p_i-p_j);
    w_connect = A(index, j2);
    w_connect =0 ;
    w_avoide = A(index, j2) * (norm_pio + R_c -2*R_sen)  / (norm_pio*((norm_pio - R_c)^3));
    w_ij= w_connect + w_avoide;
    dV_p = dV_p + w_ij * (p_i - p_j);
    ah = ah + A(index, j2) * (q_i - q_j);
end

alpha = ah + b(index) * (q_i - detar(1:2));
dV_p_leader =  2*(p_i - etar(1:2));
dV_p = dV_p + b(index) * dV_p_leader;


v_avoid_alpha = [0;0];
dV_avoid = [0;0];
for j=1:numel(obstacle)
    p_o = obstacle{j}(1:2);
    v_o = obstacle{j}(3:4);  
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
    else
        w_avoide = 0;
        vene_j = q_i;
    end

    dV_avoid = dV_avoid + w_avoide * (p_i - p_o);
    v_avoid_alpha = v_avoid_alpha + w_avoide * (q_i - vene_j);
end


k2 = 10;

s = stack_q{index}-stack_v0_hat{index};
k1 = diag([5 5]);
k2 = diag([1; 0.5]);
tao = inv(stack_G{index})*(-stack_F{index} -dV_p - k1*dV_avoid - k2* v_avoid_alpha  - 4*alpha);
control_output = tao;
control_output = sat(control_output,max_tao,-max_tao, max_heading, -max_heading);


end