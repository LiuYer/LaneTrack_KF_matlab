%% 2017.03.16 从旋转矩阵中求取姿态
clc 
clear all
close all

%% 数据导入
source_addr = 'F:/数据/Camera+IMU time';
att_file_name = '/att.txt';
image_addr = [source_addr , att_file_name];

att_R_data = load(image_addr)'; % 9*N

%% 姿态变换
R_pre = diag([1 1 1]);
NUM = length(att_R_data);
for i = 1:NUM
    Rnb = reshape(att_R_data(:, i), [3,3]);
    diff_R = R_pre'*Rnb;
 
    euler = funRnb2Euler( diff_R );
    save_diff_euler(:, i) = euler*180/pi;    
    
    % 更新数据
    R_pre = Rnb;
end

% 保存计算结果txt
svo_log_name = './svo_att.txt';
fp = fopen(svo_log_name, 'wt');
for i = 1:NUM
    fprintf(fp, '%d %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n', i-1, att_R_data(1, i), att_R_data(2, i), att_R_data(3, i), att_R_data(4, i),att_R_data(5, i), ...
                    att_R_data(6, i), att_R_data(7, i), att_R_data(8, i), att_R_data(9, i));
end
fclose(fp);