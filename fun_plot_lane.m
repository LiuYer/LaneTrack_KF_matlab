function [ CC_rgb ] = fun_plot_lane( lane_coeff, CC_rgb, rgb_value_t, camera_parameter  )
    x_min = camera_parameter.x_min;
    x_max = camera_parameter.x_max;
    y_min = camera_parameter.y_min;
    y_max = camera_parameter.y_max;
    H1 = camera_parameter.H1;
    W1 = camera_parameter.W1;
    
    c0 = lane_coeff(1,1);
    c1 = lane_coeff(1,2);
    c2 = lane_coeff(1,3);
    c3 = lane_coeff(1,4);
    
    for x_line = x_min:0.3:x_max
        y_line = c0 + c1*x_line + c2*x_line^2 + c3*x_line^3;       
        M = (-x_line + x_max)*H1/x_max; % 减少重复计算
        if x_line<x_max && x_line>x_min && y_line>y_min && y_line<y_max  
            N = (y_line + y_max)*W1/(2*y_max);
            u = round(N);
            v = round(M);
            if u>0.5 && v>0.5 && u<W1 && v<H1
                CC_rgb(v, u, :) = rgb_value_t; 
                CC_rgb(v+1, u, :) = rgb_value_t;
%                 CC_rgb(v, u+1, :) = rgb_value_t;
%                 CC_rgb(v+1, u+1, :) = rgb_value_t;
            end  
        end
    end
   
end

