%% 完成读取图片，IPM变化，并在IPM图上画车道线
clc 
clear all
close all

SHOW_IPM = 1; % 显示IPM图
%% 数据导入
source_addr = 'F:/数据/lane_KF/20170227_data';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

log_addr = [source_addr, '/log-4225.txt'];
% log_addr = [source_addr, '/log.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];
fid_log = fopen(log_addr,'r');
fid_lan_coeff = fopen(lane_coeff_addr,'r');

% 需要进行ipm显示的image文件夹名
ipm_image_file_name = 'rec_20161009_092201';
% 4225 开始空旷路段的变道
ipm_index = 4225; %1305; % 从哪一帧图片开始ipm
ipm_step = 2; % 步长
% imu数据
w_drift = [0, 0, 0]';

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
camera_parameter.M1 = [ fx  0 cx;  0  fy cy;  0  0  1 ];
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

%% 循环计算
gyro_fliter = [0 0 0]';
k_camera = 0;
k_imu = 0;
line_index_t = 0; % 用来调试，看当前读到第几行

% 对coeff相关结构体进行初始化
lane_coeff_struct.index = 0;
lane_coeff_struct.timestamp = 0;
lane_coeff_struct.NUM = 0;
figure(1);
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

 %%      
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
            gyro_fliter = funLowpassFilterVector3f( gyro_fliter, gyro_new, 0.01, 1 );

        % speed
        elseif strcmp(str_line_data_flag, 'brake_signal')
            speed_cur = str2num(str_line_raw{1, 24})/3.6;        
        % camera
        elseif strcmp(str_line_data_flag, 'cam_frame')                              
            % 获取数据
            t_s = str_line_raw{1, 1};
            t_us = str_line_raw{1, 2};
            mp4_file_name_log = str_line_raw{1, 4}; % mp4文件路径
            length_tmp = length(mp4_file_name_log);
            mp4_file_name_log = mp4_file_name_log(length_tmp-22 : length_tmp-4); 
            
            image_index_str = str_line_raw{1, 5};
            image_index_num = str2num(image_index_str) + 1; % log中图像index编号是从0开始

             % 比对当前图像的时间戳
            if SHOW_IPM
                if strcmp(mp4_file_name_log, ipm_image_file_name) && ipm_index == image_index_num                    
                    %% 图片 IPM      
                    image_name = sprintf('/%s_%08d.jpg',ipm_image_file_name, ipm_index);
                    image_addr = [source_addr, image_file_name, image_name];
                    I_rgb = imread(image_addr);          
                    % IPM变换
                    if(1)
                        [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
                        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
                        CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% 中值滤波
                        CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% 中值滤波
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
                            [ CC_rgb ] = fun_plot_lane( lane_coeff_struct.lane_coeff(i,:), CC_rgb, rgb_value_t, camera_parameter);   
                        end
                    end
                    
                    str_name = sprintf('frame%d俯视图', ipm_index);
                    title(str_name); 
                    imshow(CC_rgb); 

                    ipm_index = ipm_index + ipm_step;
                    is_Lane_Camera_matched = 1;
                end
            end
        end
    end

end
fclose(log_addr);
fclose(lane_coeff_addr);
