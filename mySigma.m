function [ Xsigma, Wm, Wc ] = mySigma( x, P, alpha, belta, ki)

    n    = numel(x);  % 生成sigma点的数量
    lamuda = alpha^2*( n + ki ) - n;
    c = n + lamuda;
    A = sqrt(c)*chol(P)';%R'*R=P     R=chol(P);
    Y = repmat(x, 1, n);
    Xsigma = [x Y+A Y-A];
    Wm = [lamuda/c, 0.5/c+zeros(1, 2*n)];     % weights for means 
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + belta);  % weights for covariance
end
