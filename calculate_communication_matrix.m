function A = calculate_communication_matrix(stack_eta, A, R_max, R_min)

    N = length(stack_eta);  

    for i = 1:N
        y_i = stack_eta{i}(1:2);  

        for j = 1:N
            if j ~= i
                y_j = stack_eta{j}(1:2);  
                norm_pij = norm(y_i - y_j); 

                if norm_pij <= R_max+5
                    A(i, j) = 1;  
                else
                    A(i, j) = 0;  
                end
                
                if norm_pij <= R_min
                    fprintf('Node %d is close to Node %d at distance %d\n', i, j, floor(norm_pij));
                end
            end
        end
    end
end
