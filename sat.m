function out = sat(val, max_val, min_val, max_heading_val, min_heading_val)
    % 饱和函数
    len = length(val);
    out = zeros(size(val));
    
    % 处理控制输入
    for i = 1:1:len
        if i == len  % 最后一位为艏向
            if val(i) > max_heading_val
                out(i) = max_heading_val;  % 应用艏向的最大饱和阈值
            elseif val(i) < min_heading_val
                out(i) = min_heading_val;  % 应用艏向的最小饱和阈值
            else
                out(i) = val(i);
            end
        else  % 对于其他控制输入
            if val(i) > max_val
                out(i) = max_val;  % 应用其他控制输入的最大饱和阈值
            elseif val(i) < min_val
                out(i) = min_val;  % 应用其他控制输入的最小饱和阈值
            else
                out(i) = val(i);
            end
        end
    end
end
