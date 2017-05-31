% 一步UKF
function [Xk, Pk, Xk_predict ] = Fukf(Xk_1, Pk_1, z, u, Q, R, alpha, beta, kappa, T, fun, hfun)
    % 计算sigma点 xSigma: n*（2n+1） Wm, Wc:1*(2n+1)
    [ xSigma, Wm, Wc ] = mySigma( Xk_1, Pk_1, alpha, beta, kappa);
    [ Xkk_1, XkSigma, dX1, Pkk_1 ] = myUT( xSigma, u, Wm, Wc, Q, T, fun);
    [ Zkk_1, dZ1, Ps ] = myUTz( XkSigma, Wm, Wc, R, hfun);
    Pxz = dX1*diag(Wc)*dZ1';
    Kk = Pxz*inv(Ps); % 卡尔曼增益
    dZk = z - Zkk_1;
    Xk = Xkk_1 + Kk*(z - Zkk_1); % 状态滤波值
    Xk_predict = Xkk_1;
    Pk = Pkk_1 - Kk*Ps*Kk';  % 滤波误差值
end

