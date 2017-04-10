%% ��ɶ�ȡͼƬ��IPM�仯������IPMͼ�ϻ�������
clc 
clear all
close all

SHOW_IPM = 1; % ��ʾIPMͼ
%% ���ݵ���
source_addr = 'F:/����/lane_KF/20170227_data';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

log_addr = [source_addr, '/log-4225.txt'];
% log_addr = [source_addr, '/log.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];
fid_log = fopen(log_addr,'r');
fid_lan_coeff = fopen(lane_coeff_addr,'r');

% ��Ҫ����ipm��ʾ��image�ļ�����
ipm_image_file_name = 'rec_20161009_092201';
% 4225 ��ʼ�տ�·�εı��
ipm_index = 4225; %1305; % ����һ֡ͼƬ��ʼipm
ipm_step = 2; % ����
% imu����
w_drift = [0, 0, 0]';

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
camera_parameter.M1 = [ fx  0 cx;  0  fy cy;  0  0  1 ];
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

%% ѭ������
gyro_fliter = [0 0 0]';
k_camera = 0;
k_imu = 0;
line_index_t = 0; % �������ԣ�����ǰ�����ڼ���

% ��coeff��ؽṹ����г�ʼ��
lane_coeff_struct.index = 0;
lane_coeff_struct.timestamp = 0;
lane_coeff_struct.NUM = 0;
figure(1);
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

 %%      
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
            gyro_fliter = funLowpassFilterVector3f( gyro_fliter, gyro_new, 0.01, 1 );

        % speed
        elseif strcmp(str_line_data_flag, 'brake_signal')
            speed_cur = str2num(str_line_raw{1, 24})/3.6;        
        % camera
        elseif strcmp(str_line_data_flag, 'cam_frame')                              
            % ��ȡ����
            t_s = str_line_raw{1, 1};
            t_us = str_line_raw{1, 2};
            mp4_file_name_log = str_line_raw{1, 4}; % mp4�ļ�·��
            length_tmp = length(mp4_file_name_log);
            mp4_file_name_log = mp4_file_name_log(length_tmp-22 : length_tmp-4); 
            
            image_index_str = str_line_raw{1, 5};
            image_index_num = str2num(image_index_str) + 1; % log��ͼ��index����Ǵ�0��ʼ

             % �ȶԵ�ǰͼ���ʱ���
            if SHOW_IPM
                if strcmp(mp4_file_name_log, ipm_image_file_name) && ipm_index == image_index_num                    
                    %% ͼƬ IPM      
                    image_name = sprintf('/%s_%08d.jpg',ipm_image_file_name, ipm_index);
                    image_addr = [source_addr, image_file_name, image_name];
                    I_rgb = imread(image_addr);          
                    % IPM�任
                    if(1)
                        [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
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
                            [ CC_rgb ] = fun_plot_lane( lane_coeff_struct.lane_coeff(i,:), CC_rgb, rgb_value_t, camera_parameter);   
                        end
                    end
                    
                    str_name = sprintf('frame%d����ͼ', ipm_index);
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
