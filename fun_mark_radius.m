% 最新修改 有摄像头所在平面为0平面
function [ CC_rgb ] = fun_mark_radius( I_rgb, camera_parameter )
    x_min = camera_parameter.x_min;
    x_max = camera_parameter.x_max;
    y_min = camera_parameter.y_min;
    y_max = camera_parameter.y_max;
    H1 = camera_parameter.H1;
    W1 = camera_parameter.W1;    
    m = camera_parameter.m;
    n = camera_parameter.n;
    
    roll = camera_parameter.roll;
    pitch = camera_parameter.pitch;
    yaw = camera_parameter.yaw;
    
    Pc = camera_parameter.Pc;
    M1 = camera_parameter.M1;    
  
    % 相机姿态矩阵
    Ratt_roll = [1        0        0;
                 0   cos(roll)  sin(roll);
                 0   -sin(roll)  cos(roll);];
             
    Ratt_pitch = [cos(pitch)  0  -sin(pitch);
                      0        1     0;
                  sin(pitch)  0  cos(pitch)];
    Ratt_yaw = [cos(yaw)   sin(yaw)  0;
                -sin(yaw)  cos(yaw)  0;
                     0        0      1;];
    Ratt = Ratt_roll*Ratt_pitch*Ratt_yaw;     
    Rc12c = [0 1 0;% 相机-图像坐标系  
             0 0 1;
             1 0 0];         
    I3 = diag([1,1,1]);
%      Rc12c = [0 0 1;% 相机-图像坐标系  
%              0 1 0;
%              -1 0 0]; 

    
    % 相机位置的偏移   
    CC_rgb = I_rgb;
%     R_w2i =  M1*Rc12c*Ratt*[I3 -Pc];

    R_w2i = M1*Rc12c*Ratt;
    
        %% 根据半径计算点
    R = -100;
    R_show = min(abs(R), 100);
    for x_t = 0:0.1:R_show 
        if(R > 0)
            y_t = R - sqrt(R*R - x_t*x_t);
        elseif(R < 0)
            y_t = R + sqrt(R*R - x_t*x_t);
        else
            y_t = 0;
        end

        % 投影变化，相机成像模型
        Point_xyz = [x_t, y_t, Pc(3)]';
        uv_tmp = R_w2i*Point_xyz;
        s = uv_tmp(3);
        uv_new = uv_tmp/s;% z方向深度归一化
        u = round(uv_new(1));
        v = round(uv_new(2));                
        if u>0.5 && v>0.5 && u<n && v<m
            %CC1(M, N) = I_g(v,u);
            uv = [u v]' 
            CC_rgb(v, u, :) = [200, 10, 10];
            CC_rgb(v+1, u, :) = [200, 10, 10];
            CC_rgb(v, u+1, :) = [200, 10, 10];
            CC_rgb(v+1, u+1, :) = [200, 10, 10];
        end  
    end

end

