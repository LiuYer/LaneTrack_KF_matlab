%% 2017.03.02�� ��ʼ��дUKF����ģ��
%% 2017.03.30 �����ó����߼���pitch
clc 
clear all
close all

SHOW_IPM = 1; % ��ʾIPMͼ
%% ���ݵ���
source_addr = 'F:/����/lane_KF/20170227_data';
image_file_name = '/rec_20161009_092201';
image_addr = [source_addr , image_file_name];

% log_addr = [source_addr, '/log-4225.txt'];
log_addr = [source_addr, '/log-1647-line-lane.txt'];
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
% 1647 ������ֱ��
ipm_index = 1647; %4240; %1305; % ����һ֡ͼƬ��ʼipm
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
camera_parameter.Pc =  [camera_parameter.dl 0 camera_parameter.h]'; % NED����ϵ  ������ͷ����ƽ��Ϊ0ƽ��
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

%% �����߲�����
dL = 2;
lane_length = 40; % �����߳���
lane_sapmle_points_num = round(lane_length/dL);

%% KF ���� ��ʼ��
% X:  [pxi, pyi]*12
% G = [ T^2/2  0;
%       T      0;
%       0    T^2/2;
%       0      T;];
% Ŀǰ��Զ����ķ����һ�¶Դ�������ʵ��Ӧ�ÿ��ǽ��ķ���С��Զ�ķ���� 
Q_var = 0.4;
Q_vector = ones(1, lane_sapmle_points_num)*Q_var;
Q0 = diag(Q_vector);  
% Q0 = diag([Q_var,Q_var,Q_var,Q_var,Q_var]);  
% Q = G*q*G';

R_var = 0.2;         % R����۲�����
R_vector = ones(1, lane_sapmle_points_num)*R_var;
R0 = diag(R_vector); 
% R0 = diag([R_var,R_var,R_var,R_var,R_var]); 
    
P0_var = 1;
P_vector = ones(1, lane_sapmle_points_num)*P0_var;
P0 = diag(P_vector);  
% P0 = diag([P0_var,P0_var,P0_var,P0_var,P0_var]);  
    
X0 = zeros(lane_sapmle_points_num,1);      % ״̬������ֵ(����õ���ʽ���ѭ�����ٸ�ֵ)  

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

save_i_index = 0;
pitch_imu = 0;
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
                    % ������dL Ŀǰ��Ч�۲��������Ϊ55m,����ÿ�������ߴ˲�������[xi; yi]
                    % ��Ϊһ��������(��ΪX�ǹ̶�����������x��״̬�����Ͳ���Ҫ�����������)

                    if lane_coeff_struct.NUM > 0
                        for i = 1:lane_coeff_struct.NUM
                           lane_sample_points.lane(i) = fun_lane_sample( lane_coeff_struct.lane_coeff(i,:), dL, lane_length );                           
                        end
                    end
                    
                    NUM = round(lane_length/dL);
                    for i = 1:NUM
                        X_vector_step(i,1) = dL*i;
                    end      
                    
                 %% 3.UKF ������ֻ��һ�������ߵĸ��٣��󳵵��ߣ�
                    % 1) ����
                    % ����Ѱ�ҳ�ʼ��ʱ����󳵵���
                    for i = 1:lane_coeff_struct.NUM
                        lane_offset(i) = lane_sample_points.lane(i).point(1, 1); % ȡ������lane��offset
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
                        d_pitch = gyro_d_average(2)*dt_iamge*180/pi;
                        d_yaw = gyro_d_average(3)*dt_iamge*180/pi;
                    end                   
                    XY_est = [X_vector_step';xEst'];  % [x ;y]*N             
                     
                  %% ͼƬ IPM      
                    image_name = sprintf('/%s_%08d.jpg',ipm_image_file_name, ipm_index);
                    image_addr = [source_addr, image_file_name, image_name];
                    I_rgb = imread(image_addr);
                    
%% �Ѽ������ĳ�����ͶӰ��ͼ������ ����pitch
                    I_rgb_t = I_rgb;

                    uv_points_NUM = length(X0_left);
                    i_left_index = 0;
                    i_right_index = 0;
                    for i = 1 : uv_points_NUM
                       % �󳵵���
                       xy_point = [X_vector_step(i), X0_left(i)]';
                       uv_point = fun_point_projection( xy_point, camera_parameter );
                       if uv_point(1) > 1 && uv_point(2)>1
                            i_left_index = i_left_index + 1;
                            save_uv_left(:, i_left_index) = uv_point;
                            % ����
                            rgb_value_t = [200, 10, 10]; % ��
                            I_rgb_t(uv_point(2), uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2), uv_point(1)+1, :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1)+1, :) = rgb_value_t;
                       end

                        % �ҳ�����
                        xy_point = [X_vector_step(i), X0_right(i)]';
                        uv_point = fun_point_projection( xy_point, camera_parameter );
                        if uv_point(1) > 1 && uv_point(2)>1
                            i_right_index = i_right_index + 1;
                            save_uv_right(:, i_right_index) = uv_point;
                            % ����
                            rgb_value_t = [200, 10, 10]; % ��
                            I_rgb_t(uv_point(2), uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1), :) = rgb_value_t;
                            I_rgb_t(uv_point(2), uv_point(1)+1, :) = rgb_value_t;
                            I_rgb_t(uv_point(2)+1, uv_point(1)+1, :) = rgb_value_t;
                        end
                    end

                    % ͼ������ϵ�³��������
                    ployfit_uv_point_left = [save_uv_left(1, :); -(save_uv_left(2, :)-camera_parameter.m)];
                    ployfit_uv_point_right = [save_uv_right(1, :); -(save_uv_right(2, :)-camera_parameter.m)];
                    uv_lane_coeff_left = polyfit(save_uv_left(1, :), -(save_uv_left(2, :)-camera_parameter.m), 2);
                    uv_lane_coeff_right = polyfit(save_uv_right(1, :), -(save_uv_right(2, :)-camera_parameter.m), 2);

                    % ��pitch 
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
%%%% --end--  ����pitch %% 
%%
                    % IPM�任
                    if(1)
%                         % ��Ϊǰ����֡��dpitch  ���Իָ���ǰ����һ֡�ĽǶ�
%                         camera_parameter_tmp = camera_parameter;
%                         camera_parameter_tmp.pitch = camera_parameter.pitch + d_pitch/180*pi;
                        [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
                        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% ��ֵ�˲�
                        CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% ��ֵ�˲�
                        CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% ��ֵ�˲�
                        
                        [ CC_rgb_new ] = fun_IPM( I_rgb, camera_parameter_tmp );   
                        CC_rgb_new(:,:, 1) = medfilt2(CC_rgb_new(:,:, 1),[2,2]);% ��ֵ�˲�
                        CC_rgb_new(:,:, 2) = medfilt2(CC_rgb_new(:,:, 2),[2,2]);% ��ֵ�˲�
                        CC_rgb_new(:,:, 3) = medfilt2(CC_rgb_new(:,:, 3),[2,2]);% ��ֵ�˲�
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
                    XY_predict = [X_vector_step';Xk_predict'];  % [x ;y]*N
                    [ CC_rgb ] = fun_IPM_mark_points( XY_predict, CC_rgb, rgb_value_t, camera_parameter); 
                    
                    % ��ǰ���ٳ����ߵ������X0_left
                    rgb_value_t = [200, 10, 10];
                    XY_z = [X_vector_step';X0_left'];  % [x ;y]*N
               
                    [ CC_rgb ] = fun_IPM_mark_points( XY_z, CC_rgb, rgb_value_t, camera_parameter);                
                    
                    figure(1);
                    imshow(CC_rgb); 
                    str_name = sprintf('frame %d ����ͼ', ipm_index);
                    %text(3,4,str_name,'horiz','left','color','r')
                    title(str_name);     
                    
                    figure(2);
                    imshow(CC_rgb_new); 
                    str_name = sprintf('��pitch frame %d ����ͼ', ipm_index);
                    %text(3,4,str_name,'horiz','left','color','r')
                    title(str_name); 

                    ipm_index = ipm_index + ipm_step;
                    is_Lane_Camera_matched = 1;
                    
                  %% ��������
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
