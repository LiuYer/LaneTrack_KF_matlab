%% 2017.03.02�� ��ʼ��дUKF����ģ��
clc 
clear all
close all

SHOW_IPM = 1; % ��ʾIPMͼ
%% ���ݵ���
source_addr = 'F:/����/lane_KF/20170227_data';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

% log_addr = [source_addr, '/log-4225.txt'];
log_addr = [source_addr, '/log-1950.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];

gsensor_addr = [source_addr, '/log-gsensor.ini'];
gsensor_data = load(gsensor_addr)';
data_gensor_raw = [gsensor_data(2, :); gsensor_data(3:8, :)]; 
% imu����
w_drift = [ 0.0095873, -0.02130, 0.015978]';
data_imu = fun_imu_data_trans( data_gensor_raw );
data_gyro = data_imu(5:7, :) - w_drift;

fid_log = fopen(log_addr,'r');
fid_lan_coeff = fopen(lane_coeff_addr,'r');

% ��Ҫ����ipm��ʾ��image�ļ�����
ipm_image_file_name = 'rec_20161009_092201';
% 4240 ��ʼ�տ�·�εı��
ipm_index = 1960; %4240; %1305; % ����һ֡ͼƬ��ʼipm
ipm_step = 3; % ����


%% ��ʼ������
camera_parameter.n = 1280; % u (width)
camera_parameter.m= 720; % v (height)
camera_parameter.yaw = -1.74688*pi/180; % (�Ҷ������NE,yaw��Ϊ���������ı߶�����Ϊ��)
camera_parameter.pitch = -0.824437*pi/180; % �궨�����ĽǶȸ���ʹ�õ����෴��
camera_parameter.roll = 0; % ˮƽ���
camera_parameter.h = 1.22; %1.2; % Distance camera was above the ground (meters)
camera_parameter.dl = 0; % ����ƫ�� ����Ϊ��
camera_parameter.d = 0;
camera_parameter.Pc =  [camera_parameter.dl 0 -camera_parameter.h]'; % ��������ϵ��������������  ;
fx = 1423.51504;
fy = 1424.32153;
cx = 660.36155;
cy = 375.73916;
camera_parameter.M1 = [ fx  0 cx;  
                        0  fy cy;  
                        0  0  1 ];
% ����ͼ ����
camera_parameter.x_min = 1; % ����ͷpitch���ϣ����½����뿴������
camera_parameter.x_max = 70; % ����
camera_parameter.y_min = -5;
camera_parameter.y_max = 5; % ����
camera_parameter.H1 = 400;
camera_parameter.W1 = 350;  %��Ҫ��ʾͼ��ĸߺͿ�
camera_parameter.zoom = 50;
% ��������
car_parameter.L = 2.637;
car_parameter.K_s2w = 0.0752;% ������ת��->ǰ��ת��

%% KF ���� ��ʼ��
% X:  [pxi, pyi]*12
% G = [ T^2/2  0;
%       T      0;
%       0    T^2/2;
%       0      T;];
% Ŀǰ��Զ����ķ����һ�¶Դ�������ʵ��Ӧ�ÿ��ǽ��ķ���С��Զ�ķ���� 
Q_var = 0.4;
Q0 = diag([Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,Q_var,... 
        Q_var,Q_var]);  
% Q0 = diag([Q_var,Q_var,Q_var,Q_var,Q_var]);  
% Q = G*q*G';

R_var = 0.2;         % R����۲�����
R0 = diag([R_var,R_var,R_var,R_var,R_var,R_var,R_var,R_var,R_var,R_var,... 
        R_var,R_var]); 
% R0 = diag([R_var,R_var,R_var,R_var,R_var]); 
    
P0_var = 1;
P0 = diag([P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,P0_var,... 
        P0_var,P0_var]);  
% P0 = diag([P0_var,P0_var,P0_var,P0_var,P0_var]);  
    
X0 = zeros(12,1);      % ״̬������ֵ(����õ���ʽ���ѭ�����ٸ�ֵ)  

alpha = 0.1; % sigma����x��ֵ�����ķֲ��̶� [0.0001, 1]
belta = 2; % x��̬�ֲ�ʱ������beta = 2
kalpha = 0;

is_first_run_UKF = 1; % 

%% ѭ������
gyro_fliter = [0 0 0]';
k_camera = 0;
k_imu = 0;
line_index_t = 0; % �������ԣ�����ǰ�����ڼ���

% ��coeff��ؽṹ����г�ʼ��
lane_coeff_struct.index = 0;
lane_coeff_struct.timestamp = 0;
lane_coeff_struct.NUM = 0;
% KF������ر�����ʼ��
struct_gyro_d.data = [0, 0, 0]'; % ��¼��֮֡���������imu���ݣ�������ƽ��
struct_gyro_d.counter = 0;
struct_speed.data = 0; % ��¼��֮֡����������ٶ����ݣ�������ƽ��
struct_speed.counter = 0; % ����
iamge_timestamp_pre = 0; % ��һ֡ͼ������ʱ��
is_first_step_KF = 1; % �Ƿ��ǵ�һ�ν���KF

while ~feof(fid_lan_coeff)
    is_lane_coeff_index_ok = 0;
    while(~is_lane_coeff_index_ok)
        % ��ȡlane coeff����
        lane_coeff_data = fgetl(fid_lan_coeff);
        str_line_raw = regexp(lane_coeff_data,' ','split'); %�Կո�Ϊ�����ָ��ַ���
        % lane_coeff_struct�ṹ��
        lane_coeff_struct.index = str2num(str_line_raw{1,1});
        if(lane_coeff_struct.index == ipm_index)
            is_lane_coeff_index_ok = 1;
        end
        
        lane_coeff_struct.timestamp = str2num(str_line_raw{1,2})/1000; % s
        lane_coeff_struct.NUM = str2num(str_line_raw{1,3});
        % �����û�г��������⣬��ôϵͳֻ���л���IMU��predict
        if lane_coeff_struct.NUM >0
            for i = 1:lane_coeff_struct.NUM
                for j = 1:4
                    % ʵ��ʹ��coeff��Ϊ�۲������ʱ����������match,�������ıߵĳ�����
                    % ��ֻ�����Գ�����Ϊ����������߳����ߣ����ʱ��Ҫ�������������������ۣ�
                    lane_coeff_struct.lane_coeff(i,j) = str2double(str_line_raw{1, 3+4*(i-1)+j});
                end
            end       
        end
    end

 %%  ��ȡlog����    
    is_Lane_Camera_matched = 0; % ��Ƶ���ļ�����log�е�frame����Ӧ��������log�в�ͬ����Ƶ��
    while ~is_Lane_Camera_matched && ~feof(fid_log)
        line_index_t = line_index_t+1;
        % ���ļ�ĩβ���˳�
        if feof(fid_log)
           break;
        end

        lineData = fgetl(fid_log);
        str_line_raw = regexp(lineData,' ','split'); %�Կո�Ϊ�����ָ��ַ���
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
            % ��¼��֡�����ͼƬ֮�����������
            struct_gyro_d.data = struct_gyro_d.data + gyro_fliter;
            struct_gyro_d.counter =  struct_gyro_d.counter + 1;
        % speed
        elseif strcmp(str_line_data_flag, 'brake_signal')
            speed_cur = str2num(str_line_raw{1, 24})/3.6; 
            struct_speed.data = struct_speed.data + speed_cur;
            struct_speed.counter =  struct_speed.counter + 1;
        % camera
        elseif strcmp(str_line_data_flag, 'cam_frame')                              
            % ��ȡ����
            t_s = str2num(str_line_raw{1, 1});
            t_us = str2num(str_line_raw{1, 2});
            image_timestamp = t_s + t_us*1e-6;
            mp4_file_name_log = str_line_raw{1, 4}; % mp4�ļ�·��
            length_tmp = length(mp4_file_name_log);
            mp4_file_name_log = mp4_file_name_log(length_tmp-22 : length_tmp-4); 
            
            image_index_str = str_line_raw{1, 5};
            image_index_num = str2num(image_index_str) + 1; % log��ͼ��index����Ǵ�0��ʼ

             % �ȶԵ�ǰͼ���ʱ���
            if SHOW_IPM
                if strcmp(mp4_file_name_log, ipm_image_file_name) && ipm_index == image_index_num                       
              %%  ִ��KF����
                %% 1. ���������� gyro_d_average, speed_average, dt_iamge
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
                    
                    % ������������
                    struct_gyro_d.data = 0;
                    struct_gyro_d.counter = 0;
                    struct_speed.data = 0;
                    struct_speed.counter = 0;
                    
                 %% 2. �����߲��� 
                    % ������dL = 5m Ŀǰ��Ч�۲��������Ϊ55m,����ÿ�������ߴ˲�������[xi; yi]*12
                    % ��Ϊһ��������12*1(��ΪX�ǹ̶�����������x��״̬�����Ͳ���Ҫ�����������)
                    dL = 5;
                    lane_length = 55; % �����߳���
                    if lane_coeff_struct.NUM > 0
                        for i = 1:lane_coeff_struct.NUM
                           lane_sample_points.lane(i) = fun_lane_sample( lane_coeff_struct.lane_coeff(i,:), dL, lane_length );                           
                        end
                    end
                    
                    NUM = round(lane_length/dL);
                    for i = 1:NUM+1
                        X_vector_step(i,1) = dL*(i-1);
                    end
                    
                 %% 3.UKF ������ֻ��һ�������ߵĸ��٣��󳵵��ߣ�
                    % 1) ����
                    % ����Ѱ�ҳ�ʼ��ʱ����󳵵���
                    for i = 1:lane_coeff_struct.NUM
                        lane_offset(i) = lane_sample_points.lane(i).point(1, 1); % ȡ������lane��offset
                    end

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
                    
                    % ������ʼ��
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
                        % �˲�
                        u = [speed_average*dt_iamge, gyro_d_average(3)*dt_iamge]';
                        z = X0_left;
                        [xEst, Pk, Xk_predict] = Fukf(xEst, Pk, z, u, Q, R, alpha, belta, kalpha, dt_iamge,'ffun','hfun');
                        % ��ӡ����
                        d_pitch = gyro_d_average(2)*dt_iamge*180/pi
                        d_yaw = gyro_d_average(3)*dt_iamge*180/pi
                    end
                    
                    XY_est = [X_vector_step';xEst'];  % [x ;y]*12
                    
                     
                  %% ͼƬ IPM      
                    image_name = sprintf('/%s_%08d.jpg',ipm_image_file_name, ipm_index);
                    image_addr = [source_addr, image_file_name, image_name];
                    I_rgb = imread(image_addr);          
                    % IPM�任
                    if(1)
                        % ��Ϊǰ����֡��dpitch  ���Իָ���ǰ����һ֡�ĽǶ�
                        camera_parameter_tmp = camera_parameter;
                        camera_parameter_tmp.pitch = camera_parameter.pitch + d_pitch/180*pi;
                        [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter_tmp );   
                        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% ��ֵ�˲�
                        CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% ��ֵ�˲�
                        CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% ��ֵ�˲�
                    else
                        % gray
                        I_g = rgb2gray(I_rgb);
                        [ CC_rgb ] = fun_IPM( I_g, camera_parameter );   
                        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% ��ֵ�˲�
                    end
                 %% ����
                    if (lane_coeff_struct.NUM >0)
                        rgb_value_t = [200, 10, 10]; % ��
                        for i=1:lane_coeff_struct.NUM
%                             [ CC_rgb ] = fun_plot_lane( lane_coeff_struct.lane_coeff(i,:), CC_rgb, rgb_value_t, camera_parameter);   
                        end
                    end
                    % ��KF���Ƶ�point��
                    rgb_value_t = [10, 10, 200];
                    XY_z_pre = [X_vector_step';z_pre'];  % ��һ֡�������                    
                    [ CC_rgb ] = fun_IPM_mark_points( XY_z_pre, CC_rgb, rgb_value_t, camera_parameter); 
                    z_pre = X0_left;
                    
                    % Ԥ��ĵ�
                    rgb_value_t = [10, 200, 10];
                    XY_predict = [X_vector_step';Xk_predict'];  % [x ;y]*12
                    [ CC_rgb ] = fun_IPM_mark_points( XY_predict, CC_rgb, rgb_value_t, camera_parameter); 
                    
                    % ��ǰ���ٳ����ߵ������X0_left
                    rgb_value_t = [200, 10, 10];
                    XY_z = [X_vector_step';X0_left'];  % [x ;y]*12
                    
                    [ CC_rgb ] = fun_IPM_mark_points( XY_z, CC_rgb, rgb_value_t, camera_parameter); 
                    
                    
                    figure(1);
                    imshow(CC_rgb); 
                    str_name = sprintf('frame %d ����ͼ', ipm_index);
                    %text(3,4,str_name,'horiz','left','color','r')
                    title(str_name);                  

                    ipm_index = ipm_index + ipm_step;
                    is_Lane_Camera_matched = 1;
                end
            end
        end
    end

end
fclose(log_addr);
fclose(lane_coeff_addr);
