%% 输入
% lane_coeff: 车道线参数1*4
% dL: 采样步长
% lane_length: 车道线长度

%% 输出
% lane_sample_points.cunter 采样点数量
% lane_sample_points.point 点坐标 N*1 列向量
function [ lane_sample_points ] = fun_lane_sample( lane_coeff, dL, lane_length )
    c0 = lane_coeff(1,1);
    c1 = lane_coeff(1,2);
    c2 = lane_coeff(1,3);
    c3 = lane_coeff(1,4);
    
    points_counter = 0;
    for i = 1:lane_length/dL
        x = dL*i;
        y = c0 + c1*x + c2*x^2 + c3*x^3;   
        lane_sample_points.point(i, 1) = y; % x固定步长采样 就不需要就行跟踪了
%         lane_sample_points.point(i*2, 1) = y;
        points_counter = points_counter + 1;
    end
    lane_sample_points.counter = points_counter;
end

