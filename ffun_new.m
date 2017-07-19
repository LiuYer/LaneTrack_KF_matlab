% 系统状态方程 只跟踪截距
% x: 
function [Xk1] = ffun_new(Xk, Uk, T)
    head_offset = 0; % 跟踪车头前方多远的点

    % 输入的数据
    dP = Uk(1); % speed*T
    dFai = Uk(2);
    tx = dP*sin(dFai);
    ty = dP*cos(dFai);
    
    z_x = Xk(1);
    z_y = Xk(2);
    z_fai = 1/Xk(3);
    
     % 1.根据状态方程更新Xk
    Rv = [cos(dFai), -sin(dFai) 0;
          sin(dFai), cos(dFai) 0;
               0         0     1];    
    
    % 2 计算 u
    m_BC = tan(z_fai);
    c_BC = z_y - m_BC*z_x;
    c_AB = ty + head_offset*cos(dFai) + tan(dFai)*(tx + head_offset*sin(dFai));
    m_AB = -tan(dFai);
    d_x = (c_BC - c_AB)/(m_AB - m_BC) - z_x;
    d_y = m_AB*(c_BC - c_AB)/(m_AB - m_BC) + c_AB - z_y;
    
    u = [d_x - tx;
         d_y - ty;
         dFai];
     
     Xk1 = Rv*Xk + Rv*u;
    
end
