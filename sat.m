function out = sat(val, max_val, min_val, max_heading_val, min_heading_val)
    len = length(val);
    out = zeros(size(val));
    
    for i = 1:1:len
        if i == len
            if val(i) > max_heading_val
                out(i) = max_heading_val;
            elseif val(i) < min_heading_val
                out(i) = min_heading_val;
            else
                out(i) = val(i);
            end
        else
            if val(i) > max_val
                out(i) = max_val;
            elseif val(i) < min_val
                out(i) = min_val;
            else
                out(i) = val(i);
            end
        end
    end
end