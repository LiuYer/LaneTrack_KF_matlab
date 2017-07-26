%% 2017.03.02： 开始编写UKF跟踪模块
%% 2017.06.05 重新开始
%% 2017.06.12 只跟踪车道线的截距和方向，用于LDW
%% 2017.07.19 
% 增加当前车的左右车道线的同时KF

clc 
clear all
close all

SHOW_IPM = 1; % 显示IPM图
%% 数据导入
source_addr = '/media/yj/Data4T/data/Lane/20170227_data';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

log_addr = [source_addr, '/log-wandao.txt'];
% log_addr = [source_addr, '/log.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];

gsensor_addr = [source_addr, '/log-gsensor.ini'];
gsensor_data = load(gsensor_addr)';
data_gensor_raw = [gsensor_data(2, :); gsensor_data(3:8, :)]; 

%--------------------------- imu数据 ---------------------------
w_drift = [ 0.009470 -0.020900 0.015947]'; % 0.015947
data_imu = fun_imu_data_trans( data_gensor_raw );
data_gyro = data_imu(5:7, :) - w_drift;

fid_log = fopen(log_addr,'r');
fid_lan_coeff = fopen(lane_coeff_addr,'r');

% 需要进行ipm显示的image文件夹名
ipm_image_file_name = 'rec_20161009_092201';
% ipm_index = 4240; % 开始空旷路段的变道
ipm_index = 6190; % 弯道

ipm_step = 3; % 步长
dL = 2;
lane_length = 50; % 车道线长度
points_num = round(lane_length/dL) + 1;

%% 初始化参数
camera_parameter.n = 1280; % u (width)
camera_parameter.m= 720; % v (height)
camera_parameter.yaw = -1.74688*pi/180; % (我定义的是NE,yaw右为正，车道哪边定义左为正)
camera_parameter.pitch = -0.824437*pi/180; % 标定出来的角度跟这使用的是相反的
camera_parameter.roll = 0; % 水平倾角
camera_parameter.h = 1.22; %1.2; % Distance camera was above the ground (meters)
camera_parameter.dl = 0; % 横向偏移 向右为正
camera_parameter.d = 0;
camera_parameter.Pc =  [camera_parameter.dl 0 camera_parameter.h]'; % 地理坐标系下相机中心坐标点  ;
fx = 1423.51504;
fy = 1424.32153;
cx = 660.36155;
cy = 375.73916;
camera_parameter.M1 = [ fx  0 cx;  
                        0  fy cy;  
                        0  0  1 ];
% 俯视图 参数
camera_parameter.x_min = 1; % 摄像头pitch向上，导致近距离看不见。
camera_parameter.x_max = 40; % 纵向
camera_parameter.y_min = -5;
camera_parameter.y_max = 5; % 横向
camera_parameter.H1 = 400;
camera_parameter.W1 = 400;  %需要显示图像的高和宽
camera_parameter.zoom = 50;
% 汽车参数
car_parameter.L = 2.637;
car_parameter.K_s2w = 0.0752;% 方向盘转角->前轮转角

%% KF 参数 初始化
% X:  y,x,fai
% G = [ T^2/2  0;
%       T      0;
%       0    T^2/2;
%       0      T;];
% 目前对远近点的方差都是一致对待，但是实际应该考虑近的方差小，远的方差大 

Q0 = diag([0.02, 0.02, 9/57.3]);  

R0 = diag([0.01, 0.01, 9/57.3]);  

P0 = diag([0.01, 0.01, 9/57.3]);  
    
X0_l = zeros(3,1);   % 状态向量初值(这个得地题词进入循环后再赋值)  
X0_r = zeros(3,1);      

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

is_new_lane = 0; % 是否是新的车道线
lane_offset_left_pre = 0; % 上一时刻的left offset， 用于判断是否是新的左车道线量测
lane_offset_right_pre = 0;

global g_head_offset;  % 固定跟踪车头前方多远的点
g_head_offset = 0;

save_index = 0;
time_start = 0;

while ~feof(fid_lan_coeff)
    is_lane_coeff_index_ok = 0;
    while(~is_lane_coeff_index_ok)
        % 读取lane coeff数据
        lane_coeff_data = fgetl(fid_lan_coeff);
        if(lane_coeff_data == -1)
            break
        end
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
            if 1
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
                        time_start = image_timestamp;
                        is_first_step_KF = 0;
                    end
                    dt_iamge = image_timestamp - iamge_timestamp_pre;
%                     dt_iamge = 5*60/7734*ipm_step;
                    iamge_timestamp_pre = image_timestamp;
                    
                    % 输入数据清零
                    struct_gyro_d.data = 0;
                    struct_gyro_d.counter = 0;
                    struct_speed.data = 0;
                    struct_speed.counter = 0;                               
                    
                 %% 3.UKF 现在先只做一条车道线的跟踪（左车道线）
                    % 1) 量测
                    % 首先寻找初始化时候的左/右车道线
                    % 数据内容[横轴上截距， 1/斜率]
                    %  | Y
                    %  | 
                    %  |------> X
                    for i = 1: lane_coeff_struct.NUM
                        lane_offset = lane_coeff_struct.lane_coeff(i,1);
                        if(abs(lane_offset) < 4)
                            if(lane_offset <= 0)
                                lane_left_measure = lane_coeff_struct.lane_coeff(i,:);
                            else
                                lane_right_measure = lane_coeff_struct.lane_coeff(i,:);
                            end
                        end
                    end
                    
                    % 判断是否是新的车道线
                    d_offset_left = lane_left_measure(1,1) - lane_offset_left_pre;
                    d_offset_right = lane_right_measure(1,1) - lane_offset_right_pre;
                    if abs(d_offset_left) > 2 || abs(d_offset_right) > 2
                        is_new_lane = 1;
                    end
                    lane_offset_left_pre = lane_left_measure(1,1);
                    lane_offset_right_pre = lane_right_measure(1,1);
                    
                    % 构造新的量测 x=ay+b
                    a = lane_left_measure(2);
                    b = lane_left_measure(1);
                    new_y = g_head_offset;
                    new_x = a*new_y + b;
                    fai = atan(a);
                    lane_left_measure_new = [new_x, new_y, fai]';
                    
                    a = lane_right_measure(2);
                    b = lane_right_measure(1);
                    new_y = g_head_offset;
                    new_x = a*new_y + b;
                    fai = atan(a);
                    lane_right_measure_new = [new_x, new_y, fai]';
                    
                    % 变量初始化
                    if((is_first_run_UKF || is_new_lane == 1) )                        
                        X0_l = lane_left_measure_new;
                        xEst_l = X0_l;
                        Xk_predict_l = X0_l;
                        z_pre_l = lane_left_measure_new;
                        Pk_l = P0;
                        Q = Q0;
                        R = R0;
                        
                        X0_r = lane_right_measure_new;
                        xEst_r = X0_r;
                        z_pre_r = lane_right_measure_new;
                        Xk_predict_r = X0_r;
                        Pk_r = P0;
                        Q = Q0;
                        R = R0;
                                               
                        is_first_run_UKF = 0;
                        is_new_lane = 0;
                    else       
                        % 滤波
                        u = [speed_average*dt_iamge, gyro_d_average(3)*dt_iamge]';
                        % left
                        z_l = lane_left_measure_new;
                        [xEst_l, Pk_l, Xk_predict_l, lamuda_l] = fun_KF_residual(xEst_l, Pk_l, u, z_l, Q, R);
                        
                        % right
                        z_r = lane_right_measure_new;
                        [xEst_r, Pk_r, Xk_predict_r, lamuda_r] = fun_KF_residual(xEst_r, Pk_r, u, z_r, Q, R);
                        
                        save_index = save_index + 1;
                        save_xk_l(:,save_index) = [image_timestamp - time_start; xEst_l];
                        save_xk_predict_l(:,save_index) = [image_timestamp - time_start; Xk_predict_l];
                        save_z_l(:,save_index) = [image_timestamp - time_start; z_l];
                        
                        save_xk_r(:,save_index) = [image_timestamp - time_start; xEst_r];
                        save_xk_predict_r(:,save_index) = [image_timestamp - time_start; Xk_predict_r];
                        save_z_r(:,save_index) = [image_timestamp - time_start; z_r];
                    end
                                        
                     
                  %% 图片 IPM      
                    image_name = sprintf('/%s_%08d.jpg',ipm_image_file_name, ipm_index);
                    image_addr = [source_addr, image_file_name, image_name];
                    I_rgb = imread(image_addr);  
                                        
                    % IPM变换
                    if SHOW_IPM
                        CC_rgb = fun_IPM( I_rgb, camera_parameter );
                        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]); %  中值滤波
                        CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);
                        CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);
                     %% 画线
                        %  ------ left ------
                        % 蓝： 上一帧检测出的车道线
                        rgb_value_t = [10, 10, 200];
                        lane_coeff = [z_pre_l(1), tan(z_pre_l(3)) 0 0]; % lane_left_measure
                        [ CC_rgb ] = fun_plot_lane( lane_coeff, 13, CC_rgb, rgb_value_t, camera_parameter);
                        z_pre_l = lane_left_measure_new;

                        % 绿：预测当前帧的车道线 
                        rgb_value_t = [10, 200, 10];
                        lane_coeff = [Xk_predict_l(1), tan(Xk_predict_l(3)) 0 0]; % lane_left_measure
                        [ CC_rgb ] = fun_plot_lane( lane_coeff, 13, CC_rgb, rgb_value_t, camera_parameter);

                        % 红：当前跟踪车道线的量测点X0_left 
                        rgb_value_t = [200, 10, 10];
                        [ CC_rgb ] = fun_plot_lane( lane_left_measure, 13, CC_rgb, rgb_value_t, camera_parameter);   
                        
                        %  ------ right ------
                        % 蓝： 上一帧检测出的车道线
                        rgb_value_t = [10, 10, 200];
                        lane_coeff = [z_pre_r(1), tan(z_pre_r(3)) 0 0]; % lane_left_measure
                        [ CC_rgb ] = fun_plot_lane( lane_coeff, 13, CC_rgb, rgb_value_t, camera_parameter);
                        z_pre_r = lane_right_measure_new;

                        % 绿：预测当前帧的车道线 
                        rgb_value_t = [10, 200, 10];
                        lane_coeff = [Xk_predict_r(1), tan(Xk_predict_r(3)) 0 0]; % lane_left_measure
                        [ CC_rgb ] = fun_plot_lane( lane_coeff, 13, CC_rgb, rgb_value_t, camera_parameter);

                        % 红：当前跟踪车道线的量测点
                        rgb_value_t = [200, 10, 10];
                        [ CC_rgb ] = fun_plot_lane( lane_right_measure, 13, CC_rgb, rgb_value_t, camera_parameter);   

                        figure(1);
                        imshow(CC_rgb); 
                        str_name = sprintf('frame %d 俯视图', ipm_index);
                        %text(3,4,str_name,'horiz','left','color','r')
                        title(str_name);                  
                    end

                    ipm_index = ipm_index + ipm_step;
                    is_Lane_Camera_matched = 1;
                end
            end
        end
    end

end

figure()
ax1 = subplot(2,1,1);
hold on ; grid on;
plot(save_xk_l(1, :), save_xk_l(2, :), '.'); % offset-est
plot(save_xk_predict_l(1, :), save_xk_predict_l(2, :), '.'); % offset-predict
plot(save_z_l(1, :), save_z_l(2, :), '.'); % z
legend('offset-est', 'offset-predict', 'offset-z');
% t_new = strrep(raw_log_addr,'_','\_'); 
title('offset');

ax2 = subplot(2,1,2);
hold on ; grid on;
plot(save_xk_l(1, :), save_xk_l(4, :)*57.3, '.'); % angle-est
plot(save_xk_predict_l(1, :), save_xk_predict_l(4, :)*57.3, '.'); % angle-predict
plot(save_z_l(1, :), save_z_l(4, :)*57.3, '.'); % angle-z
legend('angle-est/°', 'angle-predict/°', 'angle-z/°');
% t_new = strrep(raw_log_addr,'_','\_'); 
title('angle');

linkaxes([ax1, ax2], 'x'); % 同步子图的坐标轴
    
fclose(fid_log);
fclose(fid_lan_coeff);
