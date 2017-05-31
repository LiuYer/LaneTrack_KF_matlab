%% 2017.03.16: 比较SVO计算的和IMU积分的两帧之间的变化
clc 
clear all
close all

SHOW_IPM = 1; % 显示IPM图
%% 数据导入
source_addr = 'F:/数据/Camera+IMU time';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

% svo att
svo_att_addr = [source_addr, '/svo_att.txt'];

% log_addr = [source_addr, '/log-4225.txt'];
log_addr = [source_addr, '/log.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];

% imu数据
w_drift = [ 0.0095873, -0.02130, 0.015978]';

fid_log = fopen(log_addr,'r');
fid_svo_att = fopen(svo_att_addr,'r');

% 需要进行ipm显示的image文件夹名
ipm_image_file_name = 'rec_20161117_095159';
% 4240 开始空旷路段的变道
ipm_index = 1; %4240; %1305; % 从哪一帧图片开始ipm
ipm_step = 1; % 步长


%% 初始化参数
camera_parameter.n = 1280; % u (width)
camera_parameter.m= 720; % v (height)
camera_parameter.yaw = -1.74688*pi/180; % (我定义的是NE,yaw右为正，车道哪边定义左为正)
camera_parameter.pitch = -0.824437*pi/180; % 标定出来的角度跟这使用的是相反的
camera_parameter.roll = 0; % 水平倾角
camera_parameter.h = 1.22; %1.2; % Distance camera was above the ground (meters)
camera_parameter.dl = 0; % 横向偏移 向右为正
camera_parameter.d = 0;
camera_parameter.Pc =  [camera_parameter.dl 0 -camera_parameter.h]'; % 地理坐标系下相机中心坐标点  ;
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

%% KF 参数 初始化
% X:  [pxi, pyi]*12
% G = [ T^2/2  0;
%       T      0;
%       0    T^2/2;
%       0      T;];
% 目前对远近点的方差都是一致对待，但是实际应该考虑近的方差小，远的方差大 
Q_var = 0.4;
Q0 = diag([Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,... 
        Q_var,Q_var]);  
% Q0 = diag([Q_var,Q_var,Q_var,Q_var,Q_var]);  
% Q = G*q*G';

R_var = 0.2;         % R方向观测误差方差
R0 = diag([R_var,R_var,R_var,R_var,R_var,R_var,R_var,R_var,R_var,R_var,... 
        R_var,R_var]); 
% R0 = diag([R_var,R_var,R_var,R_var,R_var]); 
    
P0_var = 1;
P0 = diag([P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,... 
        P0_var,P0_var]);  
% P0 = diag([P0_var,P0_var,P0_var,P0_var,P0_var]);  
    
X0 = zeros(12,1);      % 状态向量初值(这个得地题词进入循环后再赋值)  

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
% 计算svo姿态
svo_att_index = 0;
Rbn_pre = diag([1 1 1]);

save_index = 0;

while (1)
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
            
           %% 读一组svo数据
            while svo_att_index <= image_index_num && ~feof(fid_svo_att)
                lineData = fgetl(fid_svo_att);
                str_line_raw = regexp(lineData,' ','split'); %以空格为特征分割字符串
                svo_att_index = str2num(str_line_raw{1,1})+1;
                if(svo_att_index == svo_att_index)
                    % 
                    for i = 1:9
                        att_R_data(i,1) = str2num(str_line_raw{1,1+i});
                    end
                    Rbn = reshape(att_R_data, [3,3]);
                    break;
                end
                
            end
             % 比对当前图像的时间戳
            if SHOW_IPM
                if strcmp(mp4_file_name_log, ipm_image_file_name) && ipm_index == image_index_num                       
                %% 1. 计算输入量 gyro_d_average, speed_average, dt_iamge
                    if struct_gyro_d.counter > 0
                        gyro_d_average = struct_gyro_d.data/struct_gyro_d.counter;
                    else
                        gyro_d_average = [0 0 0]';
                    end
                                        
                    if(is_first_step_KF)
                        iamge_timestamp_pre = image_timestamp;
                        is_first_step_KF = 0;
                    end
                    dt_iamge = image_timestamp - iamge_timestamp_pre;
                    iamge_timestamp_pre = image_timestamp;
                    
                    % 计算IMU diff_euler
                    diff_euler_imu = gyro_d_average*dt_iamge*180/pi;
                    
                    % 输入数据清零
                    struct_gyro_d.data = 0;
                    struct_gyro_d.counter = 0;
                    struct_speed.data = 0;
                    struct_speed.counter = 0;
                    
                  %% 计算svo相对姿态
                    diff_R = Rbn_pre'*Rbn;                    
                    Rbn_pre = Rbn;% 更新数据
                    diff_euler = funRnb2Euler( diff_R );
                    
                  %% 保存数据
                    save_index = save_index + 1;
                    save_svo_diff_euler(:, save_index) = [image_timestamp; diff_euler];
                    save_diff_euler_imu(:, save_index) = [image_timestamp; diff_euler_imu];

                    ipm_index = ipm_index + ipm_step;
                    is_Lane_Camera_matched = 1;
                end
            end
        end
    end

end
fclose(log_addr);
fclose(lane_coeff_addr);
