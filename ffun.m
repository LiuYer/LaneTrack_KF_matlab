% ϵͳ״̬����
% x: ״̬����[xi, yi]*12= 24*1
function [Xk1] = ffun(Xk,Uk,T)
    dL = 5;  % �����߲�������
    dx = Uk(1);
    dfai = Uk(2);
    NUM = length(Xk);
    
    % 1.����״̬���̸���Xk
    R = [cos(dfai), sin(dfai);
         -sin(dfai), cos(dfai)];     
    for i = 1:NUM
        xk_i_0 = [dL*(i-1); Xk(i)]; % [x0 y0]'
        dX = [dx, 0]';
        xk_tmp = R*(xk_i_0 - dX); % 2*1
        xy_tmp(:,i) = xk_tmp; % 2*12
    end
    
    % 2.Ϊ��ͳһ��dL��x���Ĳ��� ���в�ֵ������
    step_extra = round(dx/dL) + 1; % ��Ҫ���Ƶ����ݳ��� Ŀǰ��ΪӦ�ò��ᳬ��1
    for i= 1:NUM        
        if i<= (NUM - step_extra)
            xi = xy_tmp(1, i);
            yi = xy_tmp(2, i);
            xi1 = xy_tmp(1, i+1); % xi+1
            yi1 = xy_tmp(2, i+1);
            Xk1(1,i) = yi + (dL*(i-1) - xi)*((yi1 - yi)/(xi1 - xi));
        else
            xi = xy_tmp(1, i-1);
            yi = xy_tmp(2, i-1);
            xi1 = xy_tmp(1, i); % xi+1
            yi1 = xy_tmp(2, i);
            Xk1(1,i) = yi + (dL*(i-1) - xi)*((yi1 - yi)/(xi1 - xi));            
        end
        
    end

end
