function [xy] = linecross(k1, b1, k2, b2)
    x=[];
    y=[];
    if k1==k2&b1==b2

    elseif k1==k2&b1~=b2

    else
        x=(b2-b1)/(k1-k2);
        y=k1*x+b1;
    end
    
    xy = [x, y]';
 end