hold off
clc
clear all
close all

t = 480; num = 100*t; dt = t/num;
rec_loop = 0;

N = 10; dis_min = 5;  dis_max = 200; 
x_range = [0, 300]; y_range = [0, 500]; 

Adj_matrix = zeros(N);
b_leader = zeros(N,1);
b_leader(4,1) = 1;
b_leader(9,1) = 1;
b_leader(N,1) = 1;

global R_sen R_c k;
R_sen = 80; R_c = 10; k = 3; 

global k_beta; k_beta = 2;
global max_tao max_heading l s_max s_r ;
l = 1.8;
max_tao = 45; max_heading = 45; s_max = 15; s_r = 15;

global epsilon kappa0;
T_s = 0.04; 
epsilon = 0.8; 
kappa0 = 30;    
hbar = 3; 
a_max = 8*max_tao/25.8; 
T_1 = (2*kappa0 * epsilon*sqrt(2*kappa0 - 1))/a_max;
T_2 = sqrt(1 + T_1) - 1;
T_s = T_2/kappa0;

global obstacle;
obstacle{1} = [220;90;0;0;20];
obstacle{2} = [100;140;0;0;30];
obstacle{3} = [220;200;0;0;30];
obstacle{4} = [100;275;0;0;10];
obstacle{5} = [80;215;0;0;10];


% [stack_eta, stack_v] = Initialize_USV_positions(N, dis_min, dis_max, x_range, y_range);
CRV = [200; 150; 00*pi/180];
etar = [200; 140; 00*pi/180];

stack_v{1} = [0; 0; 0];
stack_v{2} = [0; 0; 0];
stack_v{3} = [0; 0; 0];
stack_v{4} = [0; 0; 0];
stack_v{5} = [0; 0; 0];
stack_v{6} = [0; 0; 0];
stack_v{7} = [0; 0; 0];
stack_v{8} = [0; 0; 0];
stack_v{9} = [0; 0; 0];
stack_v{10} = [0; 0; 0];

stack_eta{1} = [95; 65; pi/3];
stack_eta{2} = [125; 23; -pi/2];
stack_eta{3} = [159; 22; pi/2];
stack_eta{4} = [206; 46; pi/2];
stack_eta{5} = [48; 228; pi/2];
stack_eta{6} = [105; 206; pi/3];
stack_eta{7} = [145; 250; -pi/2];
stack_eta{8} = [60; 140; pi/2];
stack_eta{9} = [120; 105; pi/2];
stack_eta{10} = [180; 127; pi/2];

[target_positions, movements, target_relations, hierarchy, dangerous_positions, step_records] = establish_k_connectivity(CRV,stack_eta, R_sen, k);

for index=1:N
    stack_eta{index}(1:2) = target_positions{index};
end
Adj_matrix = calculate_communication_matrix(stack_eta, Adj_matrix, R_sen, R_c);


stack_tao = cell(1,N);
stack_tao_0 = cell(1,N);
stack_v0_hat = cell(1,N); d_v0_hat = cell(1,N); 
stack_p = cell(1,N); stack_q = cell(1,N);
stack_F = cell(1,N); stack_G = cell(1,N);
stack_s = cell(1,N);
for index=1:N
    stack_tao_0{index} = [0; 0];
    stack_v0_hat{index} = [0; 0];
end


last_broadcast_time = zeros(1,N);           
history_broadcast_time = cell(1,N);         
last_broadcast_states = cell(1,N);          
next_trigger_time = zeros(1,N);             
delta_integrals = repmat({[0; 0]}, 1, N);   


window_end_time = zeros(1,N);               
slot_len = T_s / hbar;                     

for i0 = 1:N
    [stack_p{i0}, stack_q{i0}, stack_F{i0}, stack_G{i0}] = calculate_underactuated_model(stack_eta, stack_v, l, i0);
    last_broadcast_states{i0} = [stack_p{i0};stack_q{i0}];
end


for i1 = 1:N
    group_offset = mod(hierarchy(i1), hbar);    
    phase_shift  = group_offset * slot_len;     
    next_trigger_time(i1) = phase_shift;       
    window_end_time(i1)   = phase_shift + slot_len; 
end


store_tao = cell(1,N);
store_virtual_vene = cell(1,N);
store_s = cell(1,N);
store_states = cell(1,N);
follower = cell(1,N); 
tar_yaw = cell(1,N); 
A_history = cell(1, num); 
hier_history = cell(1, num); 
tar_path(1,:) = etar;
store_obstacles = cell(1,numel(obstacle));


for i2 = 1:N
    follower{i2}(1,:) = stack_eta{i2};
    store_states{i2}(1,:) = [stack_p{i2}',stack_q{i2}'];
    tar_yaw{i2}(:) = 0;
end


broadcast_history = cell(1,N); 
for i3 = 1:N
    broadcast_history{i3}(1,:) = last_broadcast_states{i3};
end

%% 积分
for loop=1:1:num
    for index = 1:N
        [stack_p{index}, stack_q{index}, stack_F{index}, stack_G{index}] = calculate_underactuated_model(stack_eta, stack_v, l, index);

        composite_p{index} = [stack_p{index};stack_q{index}];
    end

    t = loop*dt;
    [stack_delta, detar] = trajectory(t, etar);
    etar = etar + detar*dt;
    [stack_p0, stack_q0, stack_F0, stack_G0] = calculate_underactuated_model(etar, detar, l, 0);

    if t >= 150 && t <= 190
        for n = 1:numel(obstacle)
            obstacle{n}(1:2) = obstacle{n}(1:2) + obstacle{n}(3:4) * dt;
        end
    end

    for i4 = 1:N

        in_window = (t >= next_trigger_time(i4) - 0.5*dt) && (t < window_end_time(i4) - 0.5*dt);

        if in_window
            should_trigger = calculate_event_function(T_s, t, last_broadcast_time(i4), ...
                composite_p{i4}, last_broadcast_states{i4}, delta_integrals{i4}, dt);

            if should_trigger
                last_broadcast_states{i4} = [stack_p{i4}; stack_q{i4}];
                delta_integrals{i4} = [0;0];

                history_broadcast_time{i4}(loop,1) = t;
                last_broadcast_time(i4) = t;

                next_trigger_time(i4) = next_trigger_time(i4) + T_s;
                window_end_time(i4)   = window_end_time(i4)   + T_s;

            else
                delta_integrals{i4} = delta_integrals{i4} + kappa0 * (last_broadcast_states{i4}(3:4) - stack_q{i4}) * dt;
                last_broadcast_states{i4}(1:2) = last_broadcast_states{i4}(1:2) + last_broadcast_states{i4}(3:4) * dt;
                history_broadcast_time{i4}(loop,1) = -1;
            end

        elseif t >= window_end_time(i4) - 0.5*dt
            last_broadcast_states{i4} = [stack_p{i4}; stack_q{i4}];
            delta_integrals{i4} = [0;0];

            history_broadcast_time{i4}(loop,1) = t;   
            last_broadcast_time(i4) = t;

            next_trigger_time(i4) = next_trigger_time(i4) + T_s;
            window_end_time(i4)   = window_end_time(i4)   + T_s;

        else
            delta_integrals{i4} = delta_integrals{i4} + kappa0 * (last_broadcast_states{i4}(3:4) - stack_q{i4}) * dt;
            last_broadcast_states{i4}(1:2) = last_broadcast_states{i4}(1:2) + last_broadcast_states{i4}(3:4) * dt;
            history_broadcast_time{i4}(loop,1) = -1;
        end
    end

    for i5 = 1:N
        broadcast_history{i5}(loop+1,:) = last_broadcast_states{i5};
    end
    Adj_matrix = calculate_communication_matrix(stack_p, Adj_matrix, R_sen, R_c);

    for index = 1:N
        [d_v0_hat{index},stack_v0_hat{index}] = calculate_leader_vene_hat(composite_p, last_broadcast_states, Adj_matrix, b_leader, etar, detar, stack_v0_hat, dt, index);
        [stack_tao{index},stack_s{index}] = calculate_control_output(composite_p, last_broadcast_states, stack_q,hierarchy,stack_F,stack_G, Adj_matrix, b_leader, etar,detar, stack_tao_0, stack_v0_hat,d_v0_hat, index);
        stack_tao_0{index} = stack_tao{index}';
        [stack_deta{index}, stack_dv{index}] = usv_plant(stack_eta, stack_v, stack_tao, stack_v0_hat, index);
        stack_eta{index} = stack_eta{index} + stack_deta{index}*dt;
        stack_v{index} = stack_v{index} + stack_dv{index}*dt;
    end

    for i = 1:N
        follower{i}(loop+1,:) = stack_eta{i};
        store_states{i}(loop+1, :) = [stack_p{i}',stack_q{i}'];
        tar_yaw{i}(loop+1,:) = atan2(stack_v0_hat{i}(2), stack_v0_hat{i}(1));
        store_tao{i}(loop,:) = stack_tao{i};
        store_virtual_vene{i}(loop,:)  = stack_v0_hat{i};
        store_s{i}(loop,:)  = stack_s{i};
    end

    for num_Obs = 1:numel(obstacle)
        store_obstacles{num_Obs}(loop+1,:) = obstacle{num_Obs};
    end

    tar_path(loop+1,:) = etar;
    tar_p0(loop,:)=[stack_p0;stack_q0];
    A_history{loop} = Adj_matrix;
    hier_history{loop} = hierarchy;
    Time(loop) = t;

    if ((loop - rec_loop) * dt > 0.5 )
        plot_movie_real;
        rec_loop = loop;
    end
end

hold off;
close all;
plot_mian;
