% 利用螺旋曲线模型，家务车道宽度作为装填量，跟车辆运动建立联合运动方程
% X = [y vy ay fai C]
% U = [w] % 角速度
function [Xk, Pk, Xk_predict, lamuda] = fun_KF_clothiod(Xk_1, Pk_1, U,  Z, Q, R, V, dt)
    % 输入的数据
    % 1. F
    F = [ 1   dt  0.5*dt^2   V*dt   0;
          0    1      dt       0    0;
          0    0       1       0    0;
          0    0       0       1   V*dt;
          0    0       0       0    1];

    % 2. 构造量测, H
    % z = [c0 c1 c2 c3] % lane_coeff_measure
    c0 = Z(1);
    c1 = Z(2);
    c2 = Z(3);
    c3 = Z(4);
    index = 0;
    for x = 0:1:10
        index = index + 1;
        y = c0 + c1*x + c2*x^2 + c3*x^3;  
        L = sqrt(x^2 + (y-c0)^2);
        H(index, :) = [1, 0, 0, L, L^2/2];
        Z_new(index, :) = y;  
    end
    R_new = diag(ones(1, index)*R);
    
    % 2 计算 u
    w = U;   
    U_new = [V*dt*(0.5*(-w*dt))  0  0  -w*dt  0]'; % w取负数，主要是因为fai角度的正方向定义(车道线-->车辆纵轴)

    % KF main
    L = length(Xk_1); 
    In = eye(L);
    
    Xk_predict = F*Xk_1 + U_new;           % 1.状态一步预测    
    P_tmp = F*Pk_1*F';
    Pk_predict = F*Pk_1*F' + Q;        % 2.预测误差协方差阵
    S = H*Pk_predict*H' + R_new;           %  信息协方差阵
    Kk = Pk_predict*H'*(S^-1);         % 3. 增益矩阵
    Z_predict = H*Xk_predict;    
    rk = Z_new - Z_predict;
    lamuda = rk'*inv(S)*rk;
    tt1 = Kk*rk;
    Xk = Xk_predict + Kk*rk;         % 4.状态估计
    V_new = Xk(2) + V*Xk(4); % 车辆相对车道线的横向速度
    Pk = (In - Kk*H)*Pk_predict;     % 5.协方差估计
end