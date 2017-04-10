function [Ey, dY, Py] = myUTz( Xsigma, Wm, Wc, R, hfun1)
    L = size(Xsigma, 2);%L  should equal 2n+1
    n = size(R, 1);  %  ����
%     Ey = zeros(n, 1);
%     Y = zeros(n, L);

    Y = feval(hfun1, Xsigma);
    Ey = Y*Wm';
    for k = 1 : L   % ����Ч�ʱ����ĸ�
        dY (:, k) = Y(:, k) - Ey ;
    end

%     Py = zeros(n, n);
%     for i = 1:L
%         %ii1 = dY(:,i)*dY(:,i)';
%         Py = Py + Wc(i)*dY(:,i)*dY(:,i)';
%     end
%     Py = Py + R;
    Py = dY*diag(Wc)*dY' + R;
end