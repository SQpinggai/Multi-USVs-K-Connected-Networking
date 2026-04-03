hold off
clc
clear all
close all

%% 仿真时长
t = 480; num = 100*t; dt = t/num;
rec_loop = 0;

%% 无人艇位置生成
N = 10; dis_min = 5;  dis_max = 200; % 随机生成无人艇时的距离限制
x_range = [0, 300]; y_range = [0, 500]; % 随机生成无人艇的范围

%% 拓扑图
Adj_matrix = zeros(N);
b_leader = zeros(N,1);
% b_leader(:,1) = 1;
b_leader(4,1) = 1;
b_leader(9,1) = 1;
b_leader(N,1) = 1;

%% 通信半径 算法连通度需求
global R_sen R_c k;
R_sen = 80; R_c = 10; k = 3;  % 最大通信半径 无人艇威胁半径 连通度

%% 算法参数
global k_beta; k_beta = 2;
global max_tao max_heading l s_max s_r ;
l = 1.8; % 欠驱转换参数
max_tao = 45; max_heading = 45; s_max = 15; s_r = 15;

%% 分组周期触发参数
global epsilon kappa0;
T_s = 0.04; % 触发周期1秒
epsilon = 0.8;      % 触发条件常数
kappa0 = 30;       % 积分增益系数 7.12对应最大值
hbar = 3; % 每组的层级跨度
a_max = 8*max_tao/25.8; % 最大加速度
T_1 = (2*kappa0 * epsilon*sqrt(2*kappa0 - 1))/a_max;
T_2 = sqrt(1 + T_1) - 1;
T_s = T_2/kappa0;

%% 障碍物信息
global obstacle;
obstacle{1} = [220;90;0;0;20];
obstacle{2} = [100;140;0;0;30];
obstacle{3} = [220;200;0;0;30];
obstacle{4} = [100;275;0;0;10];
obstacle{5} = [80;215;0;0;10];

%% 初始化无人艇位置
% [stack_eta, stack_v] = Initialize_USV_positions(N, dis_min, dis_max, x_range, y_range);
% calculate_if_k_connected(stack_eta, k, R_sen); % 检验系统是否满足指定的连通度要求

%% 自定义凝聚点
CRV = [200; 150; 00*pi/180];

%% 领导者初始位置
etar = [200; 140; 00*pi/180];

%% 初始位置
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

%% 构筑k连通期望构型
[target_positions, movements, target_relations, hierarchy, dangerous_positions, step_records] = establish_k_connectivity(CRV,stack_eta, R_sen, k);
%将k连通构型赋值给无人艇
for index=1:N
    stack_eta{index}(1:2) = target_positions{index};
end
% 初始阶段计算邻接矩阵
Adj_matrix = calculate_communication_matrix(stack_eta, Adj_matrix, R_sen, R_c);

%% 初始化变量
stack_tao = cell(1,N);
stack_tao_0 = cell(1,N);% 上一时刻的控制输入
stack_v0_hat = cell(1,N); d_v0_hat = cell(1,N); % 观测速度 观测速度导数
stack_p = cell(1,N); stack_q = cell(1,N);
stack_F = cell(1,N); stack_G = cell(1,N);
stack_s = cell(1,N);
for index=1:N
    stack_tao_0{index} = [0; 0];
    stack_v0_hat{index} = [0; 0];
end

%-------- 初始化事件触发部分变量
last_broadcast_time = zeros(1,N);           % 上次触发时间
history_broadcast_time = cell(1,N);         % 历史触发事件
last_broadcast_states = cell(1,N);          % 上次广播的状态
next_trigger_time = zeros(1,N);             % 当前“时间窗起点”（下一次允许检测区间的开始）
delta_integrals = repmat({[0; 0]}, 1, N);   % 积分项累积值，用于事件触发函数

% ======== 新增：时间窗终点（窗尾强制触发用）
window_end_time = zeros(1,N);               % 当前时间窗终点
slot_len = T_s / hbar;                      % 每个时间窗长度（T_s 分成 hbar 段）

% ----------- 初始化初始的通信网络状态
for i0 = 1:N
    [stack_p{i0}, stack_q{i0}, stack_F{i0}, stack_G{i0}] = calculate_underactuated_model(stack_eta, stack_v, l, i0);
    last_broadcast_states{i0} = [stack_p{i0};stack_q{i0}];
end

% ======== 初始化每个体的时间窗起点/终点（组内交错）
for i1 = 1:N
    group_offset = mod(hierarchy(i1), hbar);     % 层级模hbar得到组内偏移
    phase_shift  = group_offset * slot_len;      % 时间窗起点
    next_trigger_time(i1) = phase_shift;         % 窗起点
    window_end_time(i1)   = phase_shift + slot_len; % 窗终点
end

%% 要记录的变量
store_tao = cell(1,N);% 控制器输出
store_virtual_vene = cell(1,N);
store_s = cell(1,N);
store_states = cell(1,N);
follower = cell(1,N); % 无人艇轨迹
tar_yaw = cell(1,N); % 期望艏向  没啥用
A_history = cell(1, num); % 保存每个时刻的通信关系矩阵
hier_history = cell(1, num); % 保存每个时刻的层级信息
tar_path(1,:) = etar;
store_obstacles = cell(1,numel(obstacle));

% 初始化follower信息为初始位置信息
for i2 = 1:N
    follower{i2}(1,:) = stack_eta{i2};
    store_states{i2}(1,:) = [stack_p{i2}',stack_q{i2}'];
    tar_yaw{i2}(:) = 0;
end

%% 添加广播位置记录器（在初始化部分）
broadcast_history = cell(1,N); % 每个元素存储[时间, x, y]
for i3 = 1:N
    broadcast_history{i3}(1,:) = last_broadcast_states{i3}; % 初始位置（保持你原写法）
end

%% 积分
for loop=1:1:num
    for index = 1:N
        [stack_p{index}, stack_q{index}, stack_F{index}, stack_G{index}] = calculate_underactuated_model(stack_eta, stack_v, l, index);
        % 将层次信息整合到位置信息
        composite_p{index} = [stack_p{index};stack_q{index}];
    end

    t = loop*dt;

    % 领导者更新
    [stack_delta, detar] = trajectory(t, etar);
    etar = etar + detar*dt;
    [stack_p0, stack_q0, stack_F0, stack_G0] = calculate_underactuated_model(etar, detar, l, 0);

    if t >= 150 && t <= 190
        for n = 1:numel(obstacle)
            % 更新障礙物位置
            obstacle{n}(1:2) = obstacle{n}(1:2) + obstacle{n}(3:4) * dt;
        end
    end

    % ============================================================
    % --- 分组交错事件触发处理：窗内检测 + 窗尾强制触发（你要的改动） ---
    % 说明：
    %   next_trigger_time(i) : 当前时间窗起点
    %   window_end_time(i)   : 当前时间窗终点
    %   窗内：每个 dt 检测 should_trigger
    %   窗尾：若一直不满足，则强制触发一次
    %   触发后：整体平移一个周期 T_s（对应下一轮该组时间窗）
    % ============================================================
    for i4 = 1:N

        % 窗内（允许检测/触发）
        in_window = (t >= next_trigger_time(i4) - 0.5*dt) && (t < window_end_time(i4) - 0.5*dt);

        if in_window
            % --- 窗内：检测事件触发函数 ---
            should_trigger = calculate_event_function(T_s, t, last_broadcast_time(i4), ...
                composite_p{i4}, last_broadcast_states{i4}, delta_integrals{i4}, dt);

            if should_trigger
                % 满足条件：立即广播
                last_broadcast_states{i4} = [stack_p{i4}; stack_q{i4}];
                delta_integrals{i4} = [0;0];

                history_broadcast_time{i4}(loop,1) = t;
                last_broadcast_time(i4) = t;

                % 进入下一周期该组时间窗
                next_trigger_time(i4) = next_trigger_time(i4) + T_s;
                window_end_time(i4)   = window_end_time(i4)   + T_s;

            else
                % 未满足：继续积累（但仍在窗内，下一步仍会检测）
                delta_integrals{i4} = delta_integrals{i4} + kappa0 * (last_broadcast_states{i4}(3:4) - stack_q{i4}) * dt;
                last_broadcast_states{i4}(1:2) = last_broadcast_states{i4}(1:2) + last_broadcast_states{i4}(3:4) * dt;
                history_broadcast_time{i4}(loop,1) = -1;
            end

        elseif t >= window_end_time(i4) - 0.5*dt
            % --- 窗尾：还没触发 -> 强制触发一次 ---
            last_broadcast_states{i4} = [stack_p{i4}; stack_q{i4}];
            delta_integrals{i4} = [0;0];

            history_broadcast_time{i4}(loop,1) = t;   % 如需区分“强制触发”，可改成 -2
            last_broadcast_time(i4) = t;

            % 进入下一周期该组时间窗
            next_trigger_time(i4) = next_trigger_time(i4) + T_s;
            window_end_time(i4)   = window_end_time(i4)   + T_s;

        else
            % --- 窗外：不允许触发，只做预测/积分 ---
            delta_integrals{i4} = delta_integrals{i4} + kappa0 * (last_broadcast_states{i4}(3:4) - stack_q{i4}) * dt;
            last_broadcast_states{i4}(1:2) = last_broadcast_states{i4}(1:2) + last_broadcast_states{i4}(3:4) * dt;
            history_broadcast_time{i4}(loop,1) = -1;
        end
    end

    % --- 周期触发处理 ---
    for i5 = 1:N
        broadcast_history{i5}(loop+1,:) = last_broadcast_states{i5};
    end

    % 更新通信关系
    Adj_matrix = calculate_communication_matrix(stack_p, Adj_matrix, R_sen, R_c);

    % 更新层级信息
    % hierarchy = update_hierarchy(Adj_matrix, hierarchy, k)

    %% 计算期望速度 QP计算控制输出 积分计算当前速度位置
    for index = 1:N
        [d_v0_hat{index},stack_v0_hat{index}] = calculate_leader_vene_hat(composite_p, last_broadcast_states, Adj_matrix, b_leader, etar, detar, stack_v0_hat, dt, index);
        [stack_tao{index},stack_s{index}] = calculate_control_output(composite_p, last_broadcast_states, stack_q,hierarchy,stack_F,stack_G, Adj_matrix, b_leader, etar,detar, stack_tao_0, stack_v0_hat,d_v0_hat, index);
        stack_tao_0{index} = stack_tao{index}';
        [stack_deta{index}, stack_dv{index}] = usv_plant(stack_eta, stack_v, stack_tao, stack_v0_hat, index);
        stack_eta{index} = stack_eta{index} + stack_deta{index}*dt;
        stack_v{index} = stack_v{index} + stack_dv{index}*dt;
    end

    %% 记录信息
    for i = 1:N
        follower{i}(loop+1,:) = stack_eta{i};
        store_states{i}(loop+1, :) = [stack_p{i}',stack_q{i}'];
        tar_yaw{i}(loop+1,:) = atan2(stack_v0_hat{i}(2), stack_v0_hat{i}(1));
        store_tao{i}(loop,:) = stack_tao{i};
        store_virtual_vene{i}(loop,:)  = stack_v0_hat{i};
        store_s{i}(loop,:)  = stack_s{i};
    end

    % 记录障碍物位置信息
    for num_Obs = 1:numel(obstacle)
        store_obstacles{num_Obs}(loop+1,:) = obstacle{num_Obs};
    end

    tar_path(loop+1,:) = etar;
    tar_p0(loop,:)=[stack_p0;stack_q0];
    A_history{loop} = Adj_matrix;
    hier_history{loop} = hierarchy;
    Time(loop) = t;

    if ((loop - rec_loop) * dt > 0.5 )
        plot_movie_real; % 画出实时的轨迹图
        rec_loop = loop;
    end
end

hold off;
close all;
plot_mian;
