%% imu-camera q
velo_to_cam_r = [7.027555e-03, -9.999753e-01, 2.599616e-05;
                 -2.254837e-03, -4.184312e-05, -9.999975e-01;
                 9.999728e-01, 7.027479e-03, -2.255075e-03];
             
imu_to_velo_r = [9.999976e-01, 7.553071e-04, -2.035826e-03;
                 -7.854027e-04, 9.998898e-01, -1.482298e-02;
                 2.024406e-03, 1.482454e-02, 9.998881e-01];
             
r_cam_to_imu = (imu_to_velo_r*velo_to_cam_r)';
[ Q ] = funCb2n2Q( r_cam_to_imu)


%% T
velo_to_cam_t = [-7.137748e-03, -7.482656e-02, -3.336324e-01]';
m_p_velo_in_cam = velo_to_cam_t;

imu_to_velo_t = [-8.086759e-01, 3.195559e-01, -7.997231e-01]';
m_p_imu_in_velo = imu_to_velo_t;

p_cam_in_imu = -r_cam_to_imu*(m_p_velo_in_cam + velo_to_cam_r*m_p_imu_in_velo) 