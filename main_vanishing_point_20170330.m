%% 2017.03.02： 开始编写UKF跟踪模块
%% 2017.03.30 测试用车道线计算pitch
clc 
clear all
close all

SHOW_IPM = 1; % 显示IPM图
%% 数据导入
source_addr = 'F:/数据/lane_KF/20170227_data';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

% log_addr = [source_addr, '/log-4225.txt'];
log_addr = [source_addr, '/log-1647-line-lane.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];

gsensor_addr = [source_addr, '/log-gsensor.ini'];
gsensor_data = load(gsensor_addr)';
data_gensor_raw = [gsensor_data(2, :); gsensor_data(3:8, :)]; 
% imu数据
w_drift = [ 0.0095873, -0.02130, 0.015978]';
data_imu = fun_imu_data_trans( data_gensor_raw );
data_gyro = data_imu(5:7, :) - w_drift;

fid_log = fopen(log_addr,'r');
fid_lan_coeff = fopen(lane_coeff_addr,'r');

% 需要进行ipm显示的image文件夹名
ipm_image_file_name = 'rec_20161009_092201';
% 4240 开始空旷路段的变道
% 1647 车道是直线
ipm_index = 1647; %4240; %1305; % 从哪一帧图片开始ipm
ipm_step = 3; % 步长


%% 初始化参数
camera_parameter.n = 1280; % u (width)
camera_parameter.m= 720; % v (height)
camera_parameter.yaw = -1.74688*pi/180; % (我定义的是NE,yaw右为正，车道哪边定义左为正)
camera_parameter.pitch = -0.824437*pi/180; % 标定出来的角度跟这使用的是相反的
camera_parameter.roll = 0; % 水平倾角
camera_parameter.h = 1.22; %1.2; % Distance camera was above the ground (meters)
camera_parameter.dl = 0; % 横向偏移 向右为正
camera_parameter.d = 0;
camera_parameter.Pc =  [camera_parameter.dl 0 camera_parameter.h]'; % NED坐标系  以摄像头所在平面为0平面
fx = 1423.51504;
fy = 1424.32153;
cx = 660.36155;
cy = 375.73916;
camera_parameter.M1 = [ fx  0 cx;  
                        0  fy cy;  
                        0  0  1 ];
% 俯视图 参数
camera_parameter.x_min = 1; % 摄像头pitch向上，导致近距离看不见。
camera_parameter.x_max = 70; % 纵向
camera_parameter.y_min = -5;
camera_parameter.y_max = 5; % 横向
camera_parameter.H1 = 400;
camera_parameter.W1 = 350;  %需要显示图像的高和宽
camera_parameter.zoom = 50;
% 汽车参数
car_parameter.L = 2.637;
car_parameter.K_s2w = 0.0752;% 方向盘转角->前轮转角

%% 车道线采样点
dL = 2;
lane_length = 40; % 车道线长度
lane_sapmle_points_num = round(lane_length/dL);

%% KF 参数 初始化
% X:  [pxi, pyi]*12
% G = [ T^2/2  0;
%       T      0;
%       0    T^2/2;
%       0      T;];
% 目前对远近点的方差都是一致对待，但是实际应该考虑近的方差小，远的方差大 
Q_var = 0.4;
Q_vector = ones(1, lane_sapmle_points_num)*Q_var;
Q0 = diag(Q_vector);  
% Q0 = diag([Q_var,Q_var,Q_var,Q_var,Q_var]);  
% Q = G*q*G';

R_var = 0.2;         % R方向观测误差方差
R_vector = ones(1, lane_sapmle_points_num)*R_var;
R0 = diag(R_vector); 
% R0 = diag([R_var,R_var,R_var,R_var,R_var]); 
    
P0_var = 1;
P_vector = ones(1, lane_sapmle_points_num)*P0_var;
P0 = diag(P_vector);  
% P0 = diag([P0_var,P0_var,P0_var,P0_var,P0_var]);  
    
X0 = zeros(lane_sapmle_points_num,1);      % 状态向量初值(这个得地题词进入循环后再赋值)  

alpha = 0.1; % sigma点在x均值附近的分布程度 [0.0001, 1]
belta = 2; % x正态分布时，最优beta = 2
kalpha = 0;

is_first_run_UKF = 1; % 

%% 循环计算
gyro_fliter = [0 0 0]';
k_camera = 0;
k_imu = 0;
line_index_t = 0; % 用来调试，看当前读到第几行

% 对coeff相关结构体进行初始化
lane_coeff_struct.index = 0;
lane_coeff_struct.timestamp = 0;
lane_coeff_struct.NUM = 0;
% KF计算相关变量初始化
struct_gyro_d.data = [0, 0, 0]'; % 记录两帧之间采样到的imu数据，用于求平均
struct_gyro_d.counter = 0;
struct_speed.data = 0; % 记录两帧之间采样到的速度数据，用于求平均
struct_speed.counter = 0; % 计数
iamge_timestamp_pre = 0; % 上一帧图像来的时刻
is_first_step_KF = 1; % 是否是第一次进入KF

save_i_index = 0;
pitch_imu = 0;
while ~feof(fid_lan_coeff)
    is_lane_coeff_index_ok = 0;
    while(~is_lane_coeff_index_ok)
        % 读取lane coeff数据
        lane_coeff_data = fgetl(fid_lan_coeff);
        str_line_raw = regexp(lane_coeff_data,' ','split'); %以空格为特征分割字符串
        % lane_coeff_struct结构体
        lane_coeff_struct.index = str2num(str_line_raw{1,1});
        if(lane_coeff_struct.index == ipm_index)
            is_lane_coeff_index_ok = 1;
        end
        
        lane_coeff_struct.timestamp = str2num(str_line_raw{1,2})/1000; % s
        lane_coeff_struct.NUM = str2num(str_line_raw{1,3});
        % 如果是没有车道线量测，那么系统只进行基于IMU的predict
        if lane_coeff_struct.NUM >0
            for i = 1:lane_coeff_struct.NUM
                for j = 1:4
                    % 实际使用coeff作为观测输入的时候，先数据做match,看属于哪边的车道线
                    % 车只跟踪以车中心为轴的左右两边车道线（变道时需要跟踪三根车道线再讨论）
                    lane_coeff_struct.lane_coeff(i,j) = str2double(str_line_raw{1, 3+4*(i-1)+j});
                end
            end       
        end
    end

 %%  读取log数据    
    is_Lane_Camera_matched = 0; % 视频的文件名和log中的frame名对应，来区分log中不同的视频段
    while ~is_Lane_Camera_matched && ~feof(fid_log)
        line_index_t = line_index_t+1;
        % 到文件末尾就退出
        if feof(fid_log)
           break;
        end

        lineData = fgetl(fid_log);
        str_line_raw = regexp(lineData,' ','split'); %以空格为特征分割字符串
        time_s = str2num(str_line_raw{1,1});
        time_us = str2num(str_line_raw{1,2});
        time = time_s + time_us *1e-6;
        str_line_data_flag = str_line_raw(3);
        % Gsensor
        if  strcmp(str_line_data_flag, 'Gsensor')
            for i = 1:6
                imu_data_t(i, 1) = str2num(str_line_raw{1, i+3});
            end
            data_gensor_raw = [time; imu_data_t]; 
            k_imu = k_imu + 1;
            data_imu = fun_imu_data_trans( data_gensor_raw );
            gyro_new = data_imu(5:7) - w_drift;
            gyro_fliter = funLowpassFilterVector3f( gyro_fliter, gyro_new, 0.01, 10 );
            % 记录两帧计算的图片之间的所有数据
            struct_gyro_d.data = struct_gyro_d.data + gyro_fliter;
            struct_gyro_d.counter =  struct_gyro_d.counter + 1;
        % speed
        elseif strcmp(str_line_data_flag, 'brake_signal')
            speed_cur = str2num(str_line_raw{1, 24})/3.6; 
            struct_speed.data = struct_speed.data + speed_cur;
            struct_speed.counter =  struct_speed.counter + 1;
        % camera
        elseif strcmp(str_line_data_flag, 'cam_frame')                              
            % 获取数据
            t_s = str2num(str_line_raw{1, 1});
            t_us = str2num(str_line_raw{1, 2});
            image_timestamp = t_s + t_us*1e-6;
            mp4_file_name_log = str_line_raw{1, 4}; % mp4文件路径
            length_tmp = length(mp4_file_name_log);
            mp4_file_name_log = mp4_file_name_log(length_tmp-22 : length_tmp-4); 
            
            image_index_str = str_line_raw{1, 5};
            image_index_num = str2num(image_index_str) + 1; % log中图像index编号是从0开始

             % 比对当前图像的时间戳
            if SHOW_IPM
                if strcmp(mp4_file_name_log, ipm_image_file_name) && ipm_index == image_index_num                       
              %%  执行KF计算
                %% 1. 计算输入量 gyro_d_average, speed_average, dt_iamge
                    if struct_gyro_d.counter > 0
                        gyro_d_average = struct_gyro_d.data/struct_gyro_d.counter;
                    else
                        gyro_d_average = [0 0 0]';
                    end
                    
                    if struct_speed.counter > 0
                        speed_average = struct_speed.data/struct_speed.counter;
                    else
                        speed_average = 0;
                    end
                    
                    if(is_first_step_KF)
                        iamge_timestamp_pre = image_timestamp;
                        is_first_step_KF = 0;
                    end
                    dt_iamge = image_timestamp - iamge_timestamp_pre;
                    iamge_timestamp_pre = image_timestamp;
                    
                    % 输入数据清零
                    struct_gyro_d.data = 0;
                    struct_gyro_d.counter = 0;
                    struct_speed.data = 0;
                    struct_speed.counter = 0;
                    
                 %% 2. 车道线采样 
                    % 步长：dL 目前有效观测距离设置为55m,所以每条车道线此采样后是[xi; yi]
                    % 作为一个列向量(因为X是固定步长，所以x的状态变量就不需要再里面估计了)

                    if lane_coeff_struct.NUM > 0
                        for i = 1:lane_coeff_struct.NUM
                           lane_sample_points.lane(i) = fun_lane_sample( lane_coeff_struct.lane_coeff(i,:), dL, lane_length );                           
                        end
                    end
                    
                    NUM = round(lane_length/dL);
                    for i = 1:NUM
                        X_vector_step(i,1) = dL*i;
                    end      
                    
                 %% 3.UKF 现在先只做一条车道线的跟踪（左车道线）
                    % 1) 量测
                    % 首先寻找初始化时候的左车道线
                    for i = 1:lane_coeff_struct.NUM
                        lane_offset(i) = lane_sample_points.lane(i).point(1, 1); % 取出所有lane的offset
                    end
                    
                    if(lane_coeff_struct.NUM <= 1)
                        ipm_index = ipm_index + ipm_step;
                        is_Lane_Camera_matched = 1;
                        continue;
                    elseif(lane_coeff_struct.NUM == 2)
                        max_offset = max(lane_offset);
                        for i = 1: lane_coeff_struct.NUM
                            if(lane_offset(i) < max_offset)
                                X0_left = lane_sample_points.lane(i).point;
                            else
                                X0_right = lane_sample_points.lane(i).point;
                            end          
                        end
                    else
                        max_offset = max(abs(lane_offset));
                        for i = 1: lane_coeff_struct.NUM
                            if(abs(lane_offset(i)) ~= max_offset && abs(lane_offset(i))<4.5)
                                if(lane_offset(i) <= 0)
                                    X0_left = lane_sample_points.lane(i).point;
                                else
                                    X0_right = lane_sample_points.lane(i).point;
                                end
                            end
                        end        
                    end
                    
                    % 变量初始化
                    if(is_first_run_UKF && lane_coeff_struct.NUM>=2)                        
                        X0 = X0_left;
                        xEst = X0;
                        z_pre = X0;
                        Xk_predict = X0;
                        Pk = P0;
                        Q = Q0;
                        R = R0;
                        d_pitch = 0;
                        is_first_run_UKF = 0;
                    else       
                        % 滤波
                        u = [speed_average*dt_iamge, gyro_d_average(3)*dt_iamge]';
                        z = X0_left;
                        [xEst, Pk, Xk_predict] = Fukf(xEst, Pk, z, u, Q, R, alpha, belta, kalpha, dt_iamge,'ffun','hfun');
                        % 打印调试
                        d_pitch = gyro_d_average(2)*dt_iamge*180/pi;
                        d_yaw = gyro_d_average(3)*dt_iamge*180/pi;
                    end                   
                    XY_est = [X_vector_step';xEst'];  % [x ;y]*N             
                     
                  %% 图片 IPM      
                    image_name = sprintf('/%s_%08d.jpg',ipm_image_file_name, ipm_index);
                    image_addr = [source_addr, image_file_name, image_name];
                    I_rgb = imread(image_addr);
                    
%% 把检测出来的车道线投影回图像坐标 计算pitch
                    I_rgb_t = I_rgb;

                    uv_points_NUM = length(X0_left);
                    i_left_index = 0;
                    i_right_index = 0;
                    for i = 1 : uv_points_NUM
                       % 左车道线
                       xy_point = [X_vector_step(i), X0_left(i)]';
                       uv_point = fun_point_projection( xy_point, camera_parameter );
                       if uv_point(1) > 1 && uv_point(2)>1
                            i_left_index = i_left_index + 1;
                            save_uv_left(:, i_left_index) = uv_point;
                            % 画点
                            rgb_value_t = [200, 10, 10]; % 红
                            I_rgb_t(uv_point(2), uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2), uv_point(1)+1, :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1)+1, :) = rgb_value_t;
                       end

                        % 右车道线
                        xy_point = [X_vector_step(i), X0_right(i)]';
                        uv_point = fun_point_projection( xy_point, camera_parameter );
                        if uv_point(1) > 1 && uv_point(2)>1
                            i_right_index = i_right_index + 1;
                            save_uv_right(:, i_right_index) = uv_point;
                            % 画点
                            rgb_value_t = [200, 10, 10]; % 红
                            I_rgb_t(uv_point(2), uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2), uv_point(1)+1, :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1)+1, :) = rgb_value_t;
                        end
                    end

                    % 图像坐标系下车道线拟合
                    ployfit_uv_point_left = [save_uv_left(1, :); -(save_uv_left(2, :)-camera_parameter.m)];
                    ployfit_uv_point_right = [save_uv_right(1, :); -(save_uv_right(2, :)-camera_parameter.m)];
                    uv_lane_coeff_left = polyfit(save_uv_left(1, :), -(save_uv_left(2, :)-camera_parameter.m), 2);
                    uv_lane_coeff_right = polyfit(save_uv_right(1, :), -(save_uv_right(2, :)-camera_parameter.m), 2);

                    % 求pitch 
                    vanish_point_xy =  linecross(uv_lane_coeff_left(2), uv_lane_coeff_left(3), uv_lane_coeff_right(2), uv_lane_coeff_right(3));
                    dy_vanish = vanish_point_xy(2) - camera_parameter.m/2; % 
                    pitch_vanish_pre = atan2d(dy_vanish, fy);
                    d_angle_imu = gyro_d_average*ipm_step*1/30;
                    pitch_imu = pitch_imu + d_angle_imu;
                    pitch_vanish = -(pitch_vanish_pre)
                    if(abs(pitch_vanish_pre)>3)
                        pitch_vanish = -0.8;
                    end
                    
                    camera_parameter_tmp = camera_parameter;
                    camera_parameter_tmp.pitch = pitch_vanish/180*pi;
                   
                    figure(3);
                    imshow(I_rgb_t);            
%%%% --end--  计算pitch %% 
%%
                    % IPM变换
                    if(1)
%                         % 因为前后两帧有dpitch  尝试恢复到前面那一帧的角度
%                         camera_parameter_tmp = camera_parameter;
%                         camera_parameter_tmp.pitch = camera_parameter.pitch + d_pitch/180*pi;
                        [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
                        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
                        CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% 中值滤波
                        CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% 中值滤波
                        
                        [ CC_rgb_new ] = fun_IPM( I_rgb, camera_parameter_tmp );   
                        CC_rgb_new(:,:, 1) = medfilt2(CC_rgb_new(:,:, 1),[2,2]);% 中值滤波
                        CC_rgb_new(:,:, 2) = medfilt2(CC_rgb_new(:,:, 2),[2,2]);% 中值滤波
                        CC_rgb_new(:,:, 3) = medfilt2(CC_rgb_new(:,:, 3),[2,2]);% 中值滤波
                    else
                        % gray
                        I_g = rgb2gray(I_rgb);
                        [ CC_rgb ] = fun_IPM( I_g, camera_parameter );   
                        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
                    end
                 %% 画线
                    if (lane_coeff_struct.NUM >0)
                        rgb_value_t = [200, 10, 10]; % 红
                        for i=1:lane_coeff_struct.NUM
%                             [ CC_rgb ] = fun_plot_lane( lane_coeff_struct.lane_coeff(i,:), CC_rgb, rgb_value_t, camera_parameter);   
                        end
                    end
                    % 画KF估计的point点
                    rgb_value_t = [10, 10, 200];
                    XY_z_pre = [X_vector_step';z_pre'];  % 上一帧的量测点                    
                    [ CC_rgb ] = fun_IPM_mark_points( XY_z_pre, CC_rgb, rgb_value_t, camera_parameter); 
                    z_pre = X0_left;
                    
                    % 预测的点
                    rgb_value_t = [10, 200, 10];
                    XY_predict = [X_vector_step';Xk_predict'];  % [x ;y]*N
                    [ CC_rgb ] = fun_IPM_mark_points( XY_predict, CC_rgb, rgb_value_t, camera_parameter); 
                    
                    % 当前跟踪车道线的量测点X0_left
                    rgb_value_t = [200, 10, 10];
                    XY_z = [X_vector_step';X0_left'];  % [x ;y]*N
               
                    [ CC_rgb ] = fun_IPM_mark_points( XY_z, CC_rgb, rgb_value_t, camera_parameter);                
                    
                    figure(1);
                    imshow(CC_rgb); 
                    str_name = sprintf('frame %d 俯视图', ipm_index);
                    %text(3,4,str_name,'horiz','left','color','r')
                    title(str_name);     
                    
                    figure(2);
                    imshow(CC_rgb_new); 
                    str_name = sprintf('新pitch frame %d 俯视图', ipm_index);
                    %text(3,4,str_name,'horiz','left','color','r')
                    title(str_name); 

                    ipm_index = ipm_index + ipm_step;
                    is_Lane_Camera_matched = 1;
                    
                  %% 保存数据
                    save_i_index = save_i_index + 1;
                    save_pitch_vanish(:, save_i_index) = [pitch_vanish];
                    save_pitch_imu(:, save_i_index) = [pitch_imu];          
                    
                    clear lane_coeff_struct lane_offset;
                end
            end
        end
    end

end
fclose(log_addr);
fclose(lane_coeff_addr);
