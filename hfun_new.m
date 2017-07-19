% 观测状态方程
% x: 状态变量[x, vx, y, vy, z, vz]'  6*n
% z: 3*L
function [z] = hfun_new(x)
    [L, n] = size(x);  % 2n+1进行解算
    H_vector = ones(1, L);
    H = diag(H_vector);
    z = H*x;
end








