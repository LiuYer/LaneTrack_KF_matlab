% 一步EKF
function [Xk, Pk, Xk_predict, lamuda] = fun_KF(Xk_1, Pk_1, Uk,  z, Q, R)
    global g_head_offset; % 跟踪车头前方多远的点

    % 输入的数据
    dP = Uk(1); % speed*T
    dFai = Uk(2);
    tx = dP*sin(dFai);
    ty = dP*cos(dFai);
    
    z_x = Xk_1(1);
    z_y = Xk_1(2);
    z_fai = Xk_1(3);
    
     % 1.根据状态方程更新Xk
    Rv = [cos(dFai), -sin(dFai) 0;
          sin(dFai), cos(dFai) 0;
               0         0     1]; 
    F = Rv;
    
    % 2 计算 u
    m_BC = 1/tan(z_fai);
    c_BC = z_y - m_BC*z_x;
    c_AB = ty + g_head_offset*cos(dFai) + tan(dFai)*(tx + g_head_offset*sin(dFai));
    m_AB = -tan(dFai);
    d_x = (c_BC - c_AB)/(m_AB - m_BC) - z_x;
    d_y = m_AB*(c_BC - c_AB)/(m_AB - m_BC) + c_AB - z_y;
    
    u = [d_x - tx;
         d_y - ty;
         -dFai];
     
    % H
    L = length(Xk_1);
    H_vector = ones(1, L);
    H = diag(H_vector);    
    In = eye(L);
    
    Xk_predict = F*Xk_1 + F*u;               % 1.状态一步预测    
    P_tmp = F*Pk_1*F';
    Pk_predict = F*Pk_1*F' + Q;        % 2.预测误差协方差阵
    S = H*Pk_predict*H' + R;           % 信息协方差阵
    Kk = Pk_predict*H'*(S^-1);          %3. 增益矩阵
    Z_predict = H*Xk_predict;    
    rk = z - Z_predict;
    lamuda = rk'*inv(S)*rk;
%     tt1 = Kk*residual;
    Xk = Xk_predict + Kk*rk;% 4.状态估计
    Pk = (In - Kk*H)*Pk_predict;     % 5.协方差估计
end