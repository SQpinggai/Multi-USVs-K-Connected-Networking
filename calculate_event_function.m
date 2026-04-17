function should_trigger = calculate_event_function(T_s, t, last_trigger_time,...
        composite_p, last_broadcast_states, delta_integral,dt)
    
    global epsilon kappa0;
    current_p = composite_p(1:2);
    current_q = composite_p(3:4);
    last_p = last_broadcast_states(1:2);
    last_q = last_broadcast_states(3:4);

    q_bar = last_q - current_q; 
    new_integral = delta_integral + kappa0 * q_bar * dt;
    delta = q_bar + new_integral;
    

    threshold = sqrt(2*(2*kappa0-1)*epsilon^2);
    should_trigger = (norm(delta) >= threshold);
end