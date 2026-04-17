function [stack_eta, stack_v] = Initialize_USV_positions(num_boats, R_min, R_max, x_range, y_range)
   
    stack_eta = cell(1, num_boats);
    stack_v = cell(1, num_boats);

    stack_eta{1} = [randi(x_range), randi(y_range), rand * 2 * pi];
    stack_v{1} = [0, 0, 0];

    for i = 2:num_boats
        discard = true;

        while discard
            random_x = randi(x_range);
            random_y = randi(y_range);

            new_eta = [random_x, random_y, rand * 2 * pi];
            new_v = [0, 0, 0];

            discard = false;
            for j = 1:i-1
                distance = norm(new_eta(1:2) - stack_eta{j}(1:2));
                if distance < R_min + 20
                    discard = true;
                    break;
                end
            end

            if ~discard
                at_least_one_communication = false;
                for j = 1:i-1
                    distance = norm(new_eta(1:2) - stack_eta{j}(1:2));
                    if distance <= R_max - 20
                        at_least_one_communication = true;
                        break;
                    end
                end
                if ~at_least_one_communication
                    discard = true;
                end
            end
        end

        stack_eta{i} = new_eta;
        stack_v{i} = new_v;
    end
end