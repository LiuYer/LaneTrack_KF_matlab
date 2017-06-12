%Cb2n  q0=w
% Q1 = [0.7123 -0.0077 0.01049 0.70175];
function [ Q ] = funCb2n2Q( Cb2n)
    T11 = Cb2n(1, 1);
    T12 = Cb2n(1, 2);
    T13 = Cb2n(1, 3);
    T21 = Cb2n(2, 1);
    T22 = Cb2n(2, 2);
    T23 = Cb2n(2, 3);
    T31 = Cb2n(3, 1);
    T32 = Cb2n(3, 2);
    T33 = Cb2n(3, 3);

    abs_q0 = 1/2*(sqrt( 1 + T11 + T22 + T33 ));
    abs_q1 = 1/2*(sqrt( 1 + T11 - T22 - T33 ));
    abs_q2 = 1/2*(sqrt( 1 - T11 + T22 - T33 ));
    abs_q3 = 1/2*(sqrt( 1 - T11 - T22 + T33 ));
    
    sign_q1 = sign(T32 - T23);
    sign_q2 = sign(T13 - T31);
    sign_q3 = sign(T21 - T12);


    Q(1) = abs_q0;
    Q(2) = sign_q1*abs_q1;
    Q(3) = sign_q2*abs_q2;
    Q(4) = sign_q3*abs_q3;
   
end

