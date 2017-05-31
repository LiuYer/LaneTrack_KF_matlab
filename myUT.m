function [Ey, Y, dY, Py] = myUT( Xsigma, u, Wm, Wc, R, T, fun1)
    L = size(Xsigma, 2);%L  should equal 2n+1
    n = size(Xsigma, 1);  %  行数
    Ey = zeros(n, 1);
    Y = zeros(n, L);

    %Y = feval('ffun',Xsigma, T);
%     Y = feval(fun1,Xsigma, T);
    for i = 1:(2*n + 1)
        x_sigma =  Xsigma(:,i);
        y = feval(fun1,x_sigma, u, T);
        Y(:, i) = y;
    end
    Ey = Y*Wm';
    for k = 1 : L   % 计算效率比下文高
        dY (:, k) = Y(:, k) - Ey ;
    end
    %dY = Y - Ey(:,ones(1, L));%actually y'dimension is n×1，but y is a constant ,so though y(:,ones(1,L))  to meet the dimension of Y
    Py = dY*diag(Wc)*dY' + R;
    % Py = zeros(6,6);
    % for i = 1:L
    %     ii1 = dY(:,i)*dY(:,i)';
    %     Py = Py + Wc(i)*dY(:,i)*dY(:,i)';
    % end
    % Py = Py + R;
end