clc
clear all

camera_data_addr = 'F:\����\Camera+IMU time\log-camera.ini';
fid_log = fopen(camera_data_addr,'r');
i_index = 0;
while ~feof(fid_log)
    lineData = fgetl(fid_log);
    str_line_raw = regexp(lineData,' ','split'); %�Կո�Ϊ�����ָ��ַ���
    time_s = str2num(str_line_raw{1,1});
    time_us = str2num(str_line_raw{1,2});
    time = time_s + time_us *1e-6;
    i_index = i_index + 1;
    save_camera_time(:, i_index) = time;
end

NUM = length(save_camera_time);
error_dt_counter = 0;
for i = 2:NUM
    dt = save_camera_time(1, i) - save_camera_time(1, i-1);
    if dt > 0.1
        dt = 0.033317;
        error_dt_counter = error_dt_counter + 1;
    end
    save_dt(:, i-1) = dt;
    
end
mean_dt = mean(save_dt) %  0.033
std_dt = std(save_dt)  % �ֲ��� ��0.005   ���15%�������Կ��Կ����ù̶�ֵdt

figure()
plot(save_dt);