%% **************************************************************
%名称：Opitiaml Based Alignment (GPS velocity aided) version 2.0
%功能：
%________________________________________________________________________
%
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月22日 
% ************************************************************************
%%
close all
clear all

% 全局变量
gvar_earth;

% 下载数据
% 包含avp_simData.(att, pos, vn) 和imu_simData.(acc, gry)
load('D:\data\simulate_data\trajectory_simulator_data.mat');

% 数据步长
ts_imu = 0.01;
% 采用多子样算法时使用的子样数
num_of_subsample = 2;
nts = num_of_subsample*ts_imu;

% 用msr=ref+err模拟imu输出
imuerr = imuerrorset('selfdefine');
[imu_msr.vb, imu_msr.tb] = imuadderr(imu_simData.vb, imu_simData.tb, imuerr, ts_imu);

% 时间变量
init_time = 0;
k_init = fix(init_time/ts_imu) + 1;
total_time = 100;

% 状态变量初值
init_pos_ref = avp_simData.pos(k_init, :)';
init_vn_ref = avp_simData.vn(k_init, :)';
init_att_ref = avp_simData.att(k_init, :)';
Cbn0_ref = a2mat(init_att_ref);

% 变量在前一时刻值,第二项是未来可能会设置的初值误差
prev_pos = init_pos_ref + [0, 0, 0]';
prev_vn = init_vn_ref + [0, 0, 0]';
init_vn = init_vn_ref + [0, 0, 0]';
prev_att = init_att_ref + [5, 5, 100]'*arcdeg;
prev_eth = earth(prev_pos, prev_vn);
prev_K = zeros(4, 4);

% 时变姿态矩阵初值
prev_Cntn0 = eye(3);
prev_Cbtb0 = eye(3);
alpha_sigma_term = [0, 0, 0]';
prev_beta = [0, 0, 0]';
beta = [0, 0, 0]';
qbtb0 = [1, 0, 0, 0]';
% 为动态变量分配内存
calc_att_stg = zeros(length(avp_simData.att), 3);
calc_init_att_stg = zeros(length(avp_simData.att), 3);
att_err_stg = zeros(length(avp_simData.att), 3);

% 设置循环中一些必要的计数变量
% 获得imu量测信息的次数
Lp_msr = 0;
% 进行更新的次数
Lp_update = 0;
% 当前时刻(自从导航起始时刻而非数据起始时刻)
crt = ts_imu;
%%
% 设计思路是这样的：导航中，每过ts_imu时间imu就会输出一组角增量和速度增量
% 信息。在采用多子样算法(以二子样为例)时，每获得2组imu量测值，导航计算机进行
% 一次惯导更新。Lp的含义就是imu输出信息的次数。
while (crt + init_time) < total_time
    % 进入循环，计数器+1
    Lp_msr = Lp_msr+1;
    
    % 当前时刻数据在avp_simData和imu_simData中对应的编号 
    k = round(crt/ts_imu) + k_init;
    
    % 每获得足够子样数的量测就进行一次惯导更新,否则什么也不做
    if mod(Lp_msr, num_of_subsample) == 0
        % 进行惯导更新的次数
        Lp_update = Lp_update+1;
        
        % 读入imu量测信息
        % 实际情况下导航计算机是每检测到一次imu输出就会保存一次。但是为
        % 编程方便，这里改为了每隔nts导航计算计就一下子读出num_of_subsample
        % 次采样，而不是一次读入并保存一个数据
        wm = imu_msr.tb(k-num_of_subsample+1 : k, :);
        vm = imu_msr.vb(k-num_of_subsample+1 : k, :);
        
        % 补偿圆锥/划桨误差
        [phim, dvbm] = cnscl(wm, vm);
        
        % 目前除了计算姿态之外该程序暂不进行惯导更新，pos和vn都用GPS数据
        % 模拟GPS数据(暂时不加误差)
        gps.pos = avp_simData.pos(k, :)';
        gps.vn = avp_simData.vn(k, :)';
        pos = gps.pos;
        vn = gps.vn; 
        
        % 更新Cbtb0和Cntn0
        Cbtb0 = prev_Cbtb0*rv2m(phim);
        Cntn0 = prev_Cntn0*rv2m(prev_eth.winn*nts);
        
        % *** 计算alpha(n0)和beta(b0) ***
        % alpha(n0)
        % 1. 目前用vn0_ref计算alpha, 暂时不考虑滑动窗口。
        % 2. 用gps输出的位置计算wien和gn
        alpha_sigma_term = alpha_sigma_term + ...
            prev_Cntn0*(cross(prev_eth.wien, prev_vn) - prev_eth.gn)*nts;
        alpha = Cntn0*vn - init_vn + alpha_sigma_term;
        % beta(b0)
        beta = prev_beta + prev_Cbtb0*dvbm;
        
        % QUEST法求qbn0
        [ qbn0, prev_K ] = QUEST( beta, alpha, prev_K );
        
        % 
        Cbn0 = q2mat(qbn0); 
        Cbn = Cntn0'*Cbn0*Cbtb0;
        calc_att_stg(Lp_update, :) = m2att(Cbn)';
        att_err_stg(Lp_update, :) = attnorml(m2att(Cbn)' ...
                                        - avp_simData.att(k, :));
        calc_init_att_stg(Lp_update, :) = q2att(qbn0)';
        
        % 计算一些必要角速度
        eth = earth(pos, vn);
        
        % 设置下次循环的初值
        prev_Cbtb0 = Cbtb0;
        prev_Cntn0 = Cntn0;
        prev_beta = beta;
        prev_eth = eth;
        prev_pos = pos;
        prev_vn = vn;
    else
        % do noting
    end
        
    % ** 下次进入循环的时间 **
    crt = crt + ts_imu;
end

% 删除空余空间
calc_att_stg(Lp_update+1 : end, :) = [];
calc_init_att_stg(Lp_update+1 : end, :) = [];
att_err_stg(Lp_update+1 : end, :) = [];
%% 绘图
Time_axis_of_update = (1:1:Lp_update)*nts;
length_of_imu = (1:1:length(imu_simData.vb))*ts_imu;

% 计算姿态(Cbn0)
msplot(311, Time_axis_of_update, calc_init_att_stg(:, 1)*deg, '时间 /s', 'pitch /rad');
title('Cbn0');
msplot(312, Time_axis_of_update, calc_init_att_stg(:, 2)*deg, '时间 /s', 'roll /rad');
msplot(313, Time_axis_of_update, calc_init_att_stg(:, 3)*deg, '时间 /s', 'yaw /rad');

% 计算姿态(Cbn)
msplot(311, Time_axis_of_update, calc_att_stg(:, 1)*deg, '时间 /s', 'pitch /rad');
title('change of Cbn');
msplot(312, Time_axis_of_update, calc_att_stg(:, 2)*deg, '时间 /s', 'roll /rad');
msplot(313, Time_axis_of_update, calc_att_stg(:, 3)*deg, '时间 /s', 'yaw /rad');

% 姿态误差
msplot(311, Time_axis_of_update, att_err_stg(:, 1)*deg, '时间 /s', 'pitch /rad');
title('attitude error');
msplot(312, Time_axis_of_update, att_err_stg(:, 2)*deg, '时间 /s', 'roll /rad');
msplot(313, Time_axis_of_update, att_err_stg(:, 3)*deg, '时间 /s', 'yaw /rad');

% imu输出模拟值
msplot(311, length_of_imu, imu_simData.vb(:, 1), '时间 /s', 'fb(1) m/s^2');
title('accelerometer outpute');
msplot(312, length_of_imu, imu_simData.vb(:, 2), '时间 /s', 'fb(2) m/s^2');
msplot(313, length_of_imu, imu_simData.vb(:, 3), '时间 /s', 'fb(3) m/s^2');

% imu输出模拟值
msplot(311, length_of_imu, imu_simData.tb(:, 1), '时间 /s', 'wb(1) rad/s');
title('gyroscope outpute');
msplot(312, length_of_imu, imu_simData.tb(:, 2), '时间 /s', 'wb(2) rad/s');
msplot(313, length_of_imu, imu_simData.tb(:, 3), '时间 /s', 'wb(3) rad/s');
