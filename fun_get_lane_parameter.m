% 用lane参数获取螺旋曲线参数
function [ lane_parameter ] = fun_get_lane_parameter( lane_coeff )
% lane_coeff = [c0 c1 c2 c3]
% y = c0 + c1*x + c2*x^2 + c3*x^3
%  | X
%  | 
%  |------> Y

c0 = lane_coeff(1);
c1 = lane_coeff(2);
c2 = lane_coeff(3);
c3 = lane_coeff(4);
    
y0 = c0;
fai = c1;

index = 0;
for x = 1:1:10
    index = index + 1;
    y = c0 + c1*x + c2*x^2 + c3*x^3;  
    L = sqrt(x^2 + (y-y0)^2);
    C_tmp(index) = (y - y0- fai*L)/(0.5*L^2);
end
C = mean(C_tmp);
lane_parameter = [y0 fai C]';

end

