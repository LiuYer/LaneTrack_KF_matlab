%% 完成读取图片，IPM变化，并在IPM图上画车道线
clc 
clear all
close all

SHOW_IPM = 1; % 显示IPM图
%% 数据导入
source_addr = '/home/yj/bak/data/fcw/FCW测试0317';
image_file_addr = [source_addr, '/image_0317'];
% 需要进行ipm显示的image文件夹名
ipm_image_file_name = 'rec_20170317_033540';

log_addr = [source_addr, '/log-4225.txt'];
% log_addr = [source_addr, '/log.txt'];
lane_coeff_addr = [source_addr, '/imu_kf_test_data.txt'];
fid_log = fopen(log_addr,'r');
fid_lan_coeff = fopen(lane_coeff_addr,'r');


% 4225 开始空旷路段的变道
ipm_index = 4931; %1305; % 从哪一帧图片开始ipm
ipm_step = 2; % 步长
% imu数据
w_drift = [0, 0, 0]';

%% 初始化参数
camera_parameter.n = 1280; % u (width)
camera_parameter.m= 720; % v (height)
camera_parameter.pitch = 2.0*pi/180; % 标定出来的角度跟这使用的是相反的
camera_parameter.yaw = 2.2*pi/180; % (我定义的是NE,yaw右为正，车道哪边定义左为正)
camera_parameter.roll = 0; % 水平倾角
camera_parameter.h = 1.25; %1.2; % Distance camera was above the ground (meters)
camera_parameter.dl = 0; % 横向偏移 向右为正
camera_parameter.d = 0;
camera_parameter.Pc =  [camera_parameter.dl 0 camera_parameter.h]'; % 地理坐标系下相机中心坐标点  ;
fx = 1419.81510;
fy = 1418.26413;
cx = 639.25784;
cy = 368.90589;
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
while 1            
    %% 图片 IPM      
    image_name = sprintf('/%08d.jpg', ipm_index);
    image_addr = [image_file_addr, image_name];
    I_rgb = imread(image_addr);          
    % IPM变换
%     if(1)
%         [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
%         CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
%         CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% 中值滤波
%         CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% 中值滤波
%     else
%         % gray
%         I_g = rgb2gray(I_rgb);
%         [ CC_rgb ] = fun_IPM( I_g, camera_parameter );   
%         CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
%     end
    [ CC_rgb ] = fun_mark_radius( I_rgb, camera_parameter );

    str_name = sprintf('frame%d俯视图', ipm_index);
    title(str_name); 
    imshow(CC_rgb); 

    ipm_index = ipm_index + ipm_step;
      

end
fclose(log_addr);
fclose(lane_coeff_addr);
