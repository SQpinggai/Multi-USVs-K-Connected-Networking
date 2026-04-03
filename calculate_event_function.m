function should_trigger = calculate_event_function(T_s, t, last_trigger_time,...
        composite_p, last_broadcast_states, delta_integral,dt)
    
    global epsilon kappa0;
    
    % 计算当前时刻的q_bar（可能需要根据实际物理意义调整）
    current_p = composite_p(1:2);
    current_q = composite_p(3:4);
    last_p = last_broadcast_states(1:2);
    last_q = last_broadcast_states(3:4);

    q_bar = last_q - current_q; 
    
    % 更新积分项（梯形法积分）
    new_integral = delta_integral + kappa0 * q_bar * dt;
    
    % 计算delta(t)
    delta = q_bar + new_integral;
    
    % 触发条件判断
    threshold = sqrt(2*(2*kappa0-1)*epsilon^2);
    should_trigger = (norm(delta) >= threshold);

%     if norm(current_p - last_p) >0.4
%         should_trigger = true ;
%     end
%     
%     if (t-last_trigger_time) > T_s
%         should_trigger = true;
%     end



    
    % 调试输出（可选）
%     if should_trigger
%         fprintf('USV%d triggered at t=%.2fs: delta=%.3f, threshold=%.3f\n',...
%             i, t, delta, threshold);
%     end
end