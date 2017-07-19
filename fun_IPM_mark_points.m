function [ CC_rgb ] = fun_IPM_mark_points( XY_points_t, CC_rgb, rgb_value_t, camera_parameter  )
    x_min = camera_parameter.x_min;
    x_max = camera_parameter.x_max;
    y_min = camera_parameter.y_min;
    y_max = camera_parameter.y_max;
    H1 = camera_parameter.H1;
    W1 = camera_parameter.W1;
    
    XY_points = [XY_points_t(2, :);
                 XY_points_t(1, :)]; 
    [n, NUM] = size(XY_points);
    for i = 1:NUM
        x = XY_points(1, i);
        y = XY_points(2, i);
        M = (-x + x_max)*H1/x_max; % 减少重复计算
        N = (y + y_max)*W1/(2*y_max);
        u = round(N);
        v = round(M);
        if u>2 && v>2 && u<W1-1 && v<H1-1
            CC_rgb(v, u, :) = rgb_value_t;
            CC_rgb(v+1, u, :) = rgb_value_t;
            CC_rgb(v, u+1, :) = rgb_value_t;
            CC_rgb(v+1, u+1, :) = rgb_value_t;
        end
    end
    
          
end

