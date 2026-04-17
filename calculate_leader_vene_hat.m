function [d_v0_hat,v0_hat] = calculate_leader_vene_hat(composite_p, last_broadcast_positions, A, b, etar, detar, stack_v0_hat, dt, index)
global k_beta l;
global R_sen R_c k; 
N = numel(composite_p);
ah = [0;0];
dV_p = [0;0];
d_ij = 40;
p_i = composite_p{index}(1:2);  
q_i = composite_p{index}(3:4);  
for j=1:N
    if j ~= index
    ah = ah + A(index, j) * (q_i - last_broadcast_positions{j}(3:4));
    
        p_i = composite_p{index}(1:2);
        p_j = last_broadcast_positions{j}(1:2);
        p_j = stack_v0_hat{j};
        norm_pij = norm(p_i-p_j);
        w_connect = A(index, j) * (2*R_sen - norm_pij -R_c)*(norm_pij -R_c)  / (norm_pij*((R_sen -norm_pij)^2));
        dV_p = dV_p + w_connect * (p_i - p_j);
    end
end
alpha = ah + b(index) * (q_i- detar(1:2));

dV_p_leader =  2*(p_i - etar(1:2));
dV_p = dV_p + b(index) * dV_p_leader;

d_v0_hat =  -dV_p - k_beta*alpha;

v0_hat = stack_v0_hat{index} + d_v0_hat*dt;
u_max = 50;
r_max = 20;
