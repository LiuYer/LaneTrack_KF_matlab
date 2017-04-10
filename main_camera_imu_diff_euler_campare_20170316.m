%% 2017.03.16: �Ƚ�SVO����ĺ�IMU���ֵ���֮֡��ı仯
clc 
clear all
close all

SHOW_IPM = 1; % ��ʾIPMͼ
%% ���ݵ���
source_addr = 'F:/����/Camera+IMU time';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

% svo att
svo_att_addr = [source_addr, '/svo_att.txt'];

% log_addr = [source_addr, '/log-4225.txt'];
log_addr = [source_addr, '/log.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];

% imu����
w_drift = [ 0.0095873, -0.02130, 0.015978]';

fid_log = fopen(log_addr,'r');
fid_svo_att = fopen(svo_att_addr,'r');

% ��Ҫ����ipm��ʾ��image�ļ�����
ipm_image_file_name = 'rec_20161117_095159';
% 4240 ��ʼ�տ�·�εı��
ipm_index = 1; %4240; %1305; % ����һ֡ͼƬ��ʼipm
ipm_step = 1; % ����


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
% ����svo��̬
svo_att_index = 0;
Rbn_pre = diag([1 1 1]);

save_index = 0;

while (1)
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
            
           %% ��һ��svo����
            while svo_att_index <= image_index_num && ~feof(fid_svo_att)
                lineData = fgetl(fid_svo_att);
                str_line_raw = regexp(lineData,' ','split'); %�Կո�Ϊ�����ָ��ַ���
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
             % �ȶԵ�ǰͼ���ʱ���
            if SHOW_IPM
                if strcmp(mp4_file_name_log, ipm_image_file_name) && ipm_index == image_index_num                       
                %% 1. ���������� gyro_d_average, speed_average, dt_iamge
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
                    
                    % ����IMU diff_euler
                    diff_euler_imu = gyro_d_average*dt_iamge*180/pi;
                    
                    % ������������
                    struct_gyro_d.data = 0;
                    struct_gyro_d.counter = 0;
                    struct_speed.data = 0;
                    struct_speed.counter = 0;
                    
                  %% ����svo�����̬
                    diff_R = Rbn_pre'*Rbn;                    
                    Rbn_pre = Rbn;% ��������
                    diff_euler = funRnb2Euler( diff_R );
                    
                  %% ��������
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
