% �۲�״̬����
% x: ״̬����[x, vx, y, vy, z, vz]'  6*n
% z: 3*L
function [z] = hfun(x)
    L = length(x(1, :));  % 2n+1���н���
    H_vector = ones(1, (L-1)/2);
    H = diag(H_vector);
    z = H*x;
end








