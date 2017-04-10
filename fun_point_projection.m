function [ uv ] = fun_point_projection( xy_point, camera_parameter )
    
    roll = camera_parameter.roll;
    pitch = camera_parameter.pitch;
    yaw = camera_parameter.yaw;
    
    Pc = camera_parameter.Pc;
    h = camera_parameter.h;
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
       
%     R_w2i =  M1*Rc12c*Ratt*[I3 -Pc];
    R_w2i =  M1*Rc12c*Ratt;
    
    uv_tmp = R_w2i*[xy_point; h];
    uv = round(uv_tmp(1:2, 1)/uv_tmp(3));
    
    % 如果uv中出现小于0的，则整定为[1 1]
    if uv(1) <= 0 || uv(1)>1280 || uv(2) <= 0 || uv(2)>=720
        uv = [1, 1]';
    end
end

