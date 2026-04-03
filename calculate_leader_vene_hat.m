function [d_v0_hat,v0_hat] = calculate_leader_vene_hat(composite_p, last_broadcast_positions, A, b, etar, detar, stack_v0_hat, dt, index)
global k_beta l;% 控制参数
global R_sen R_c k; % 最大通信半径 连通度k
N = numel(composite_p);
ah = [0;0];
dV_p = [0;0];
d_ij = 40;
p_i = composite_p{index}(1:2);  % 当前无人艇的位置信息
q_i = composite_p{index}(3:4);  % 当前无人艇的速度信息
for j=1:N
    if j ~= index
    %  速度一致性
    ah = ah + A(index, j) * (q_i - last_broadcast_positions{j}(3:4));
    %  通信保持
    
        p_i = composite_p{index}(1:2);
        p_j = last_broadcast_positions{j}(1:2);
        p_j = stack_v0_hat{j};
        norm_pij = norm(p_i-p_j);
        w_connect = A(index, j) * (2*R_sen - norm_pij -R_c)*(norm_pij -R_c)  / (norm_pij*((R_sen -norm_pij)^2));
%         w_connect = A(index, j);
        dV_p = dV_p + w_connect * (p_i - p_j);
    end
end
alpha = ah + b(index) * (q_i- detar(1:2));

dV_p_leader =  2*(p_i - etar(1:2));
dV_p = dV_p + b(index) * dV_p_leader;

d_v0_hat =  -dV_p - k_beta*alpha;
% d_v0_hat =- k_beta*alpha;
v0_hat = stack_v0_hat{index} + d_v0_hat*dt;
u_max = 50;
r_max = 20;
% v0_hat = sat(v0_hat,u_max,-u_max, r_max, -r_max);