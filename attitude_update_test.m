%% **************************************************************
%名称：attitude update test
%功能：姿态更新算法测试程序
%________________________________________________________________________
% 
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月19日
% ************************************************************************
%%
clear all;close all;

% 读入数据
load('D:\data\simulate_data\angle_motion_data');

% 计算和保存一些全局变量
gvar_earth;
pos = dataInfo.pos;
eth = earth(pos, [0, 0, 0]');
% 数据长度
[roll_data, column_data] = size(att);
% 总时间
total_time_of_data = att(end, 4);
% 多子样算法采用的子样数
% 注意！调试时发现：若想"强调"一个数为整形，最好调用round(四舍五入)而不是
% fix(朝零进发)，以免出现时间戳与数据对应异常的问题。
num_sample = round(2);
% imu输出对应步长
ts_imu = dataInfo.ts;
% 设置imu误差
imuerr = imuerrorset('zero');
[ imu_msr.fb, imu_msr.wb ] = imuadderr( imu_ref.fb, imu_ref.wb, imuerr);

gyrerr = imu_msr.fb - imu_ref.fb;
% 初始时刻,e.g.设置init_time=20s,采用二子样算法输出的首个姿态将是t=20.02s
% 的姿态
init_time = abs(0);
% init_time时刻的数据在att中的index
k_init = round(init_time/ts_imu) + 1;

% 初始姿态
Cbn0_ref = a2mat(att(k_init, 1:3));
q_RK4 = a2qua(att(k_init, 1:3));
Cbn0 = Cbn0_ref;
Cntn0 = eye(3);
Cbtb0 = eye(3);

% *** 为动态变量分别存储空间 ***
% 多子样算法
att_subsample_stg = zeros(roll_data, 3);
% 龙格库塔法
att_RK4_stg = zeros(roll_data, 3);
att_ref_stg = zeros(roll_data, 3);
% 多子样算法误差
att_err_subsample_stg = zeros(roll_data, 3);
% 龙格库塔法误差
att_err_RK4_stg = zeros(roll_data, 3);
idx_stg = zeros(roll_data, 3);
% 循环次数
Lp_msr = 0;
Lp_subsample = 0;
Lp_RK4 = 0;

% 当前时刻
crt = ts_imu;
k = 0;
%% 
while round((crt + init_time)/ts_imu) <= round(120/ts_imu)
    % 第Lp次循环
    Lp_msr = Lp_msr+1;
    
    % crt时刻数据所对应的编号
    k = round(crt/ts_imu) + k_init;
    idx_stg(Lp_msr, :) = [crt + init_time, k, round((crt + init_time)/ts_imu) - k];
    
    % *** 用不同方法更新姿态 1.二子样法更新四元数 2.四阶龙格库塔法 ***
    % ** 二子样 **
    if mod(round(Lp_msr), num_sample) == 0  
        % i是num_sample的整数倍，获得了足够的采样，进行更新
        Lp_subsample = Lp_subsample+1;
        % imu输出用上下限时刻速率取平均来计算imu增量
        vm = 0.5*ts_imu.*(imu_msr.fb(k-num_sample+1 : k, 1:3)...
            + imu_msr.fb(k-num_sample : k-1, 1:3));
        
        wm = 0.5*ts_imu.*(imu_msr.wb(k-num_sample+1 : k, 1:3)...
            + imu_msr.wb(k-num_sample : k-1, 1:3));
        
        % 补偿圆锥/划桨误差
        [ phim, dvbm ] = cnscl(wm, vm);
        
        % 计算姿态变化量
        Cntn0 = Cntn0*rv2m(eth.winn*num_sample*ts_imu);
        Cbtb0 = Cbtb0*rv2m(phim);
        Cbn = Cntn0'*Cbn0*Cbtb0;
        
        att_subsample_stg(Lp_subsample, :) = m2att(Cbn)*deg;
        att_ref_stg(Lp_subsample, :) = att(k, 1:3)*deg;
        att_err_subsample_stg(Lp_subsample, :) = ...
           att_subsample_stg(Lp_subsample, :) - att_ref_stg(Lp_subsample, :);
            
    else
        % do nothing
    end
    
    % ** 四阶龙格库塔法 **
    % 四阶龙格库塔法在更新区间(tn, tn+1]上，需获得三次采样：tn,tn+0.5,
    % tn+1。紧接着的下一次更新区间为(tn+1, tn+2]，需要三次采样tn+1,tn+1.5,
    % tn+2。相当于每获得两组新的量测就进行一次更新。
    % 具体更新时刻是idx=3,5,7,9...
    if mod(round(Lp_msr+1), 2) == 1 && Lp_msr+1 ~= 1
        % RK4更新次数
        Lp_RK4 = Lp_RK4+1;
        % wibb
        wm = imu_msr.wb(k-2:k, 1:3);
        % wnbb = wibb - winb
        Cbn_RK4 = q2mat(q_RK4);
        wm = wm - repmat((Cbn_RK4*eth.winn)', 3, 1);
        % 进行RK4更新
        [ q_RK4 ] = Runge_Kutta_att_update( q_RK4, wm, ts_imu*2 );
        att_RK4_stg(Lp_RK4, :) = q2att(q_RK4)*deg;
        att_err_RK4_stg(Lp_RK4, :) = ...
                            att_RK4_stg(Lp_RK4, :) - att(k, 1:3)*deg;
    end
    
    % 断点位置
    hereStop = 1;
    
    % 下次更新的时刻
    crt = crt + ts_imu;
    
end
% crt返回最后一次更新的时刻
crt = crt - ts_imu;

% 清除未使用空间
idx_stg(Lp_msr+1:end, :) = [];
att_subsample_stg(Lp_subsample+1:end, :) = [];
att_RK4_stg(Lp_RK4+1:end, :) = [];
att_ref_stg(Lp_subsample+1:end, :) = [];
att_err_subsample_stg(Lp_subsample+1:end, :) = [];
att_err_RK4_stg(Lp_RK4+1:end, :) = [];
%% 绘图
Time_axis_subsample = (1:1:Lp_subsample)*num_sample*ts_imu;
Time_axis_RK4 = (1:1:Lp_RK4)*2*ts_imu;
Time_axis_data = (0: length(imu_ref.fb)-1)*ts_imu;

% 姿态计算值
figure;
subplot(311)
plot(Time_axis_subsample, att_subsample_stg(:, 1));
xlabel('时间 /s'); ylabel('pitch /deg'); hold on; 
plot(Time_axis_subsample, att_ref_stg(:, 1));
legend('计算值','参考值');
subplot(312)
plot(Time_axis_subsample, att_subsample_stg(:, 2));
xlabel('时间 /s'); ylabel('pitch /deg'); hold on; 
plot(Time_axis_subsample, att_ref_stg(:, 2));
subplot(313)
plot(Time_axis_subsample, att_subsample_stg(:, 3));
xlabel('时间 /s'); ylabel('pitch /deg'); hold on; 
plot(Time_axis_subsample, att_ref_stg(:, 3));


% 二子样 姿态误差
msplot(311, Time_axis_subsample, att_err_subsample_stg(:, 1), '时间 /s', 'pitch err');
title('Subsample algorithm');
msplot(312, Time_axis_subsample, att_err_subsample_stg(:, 2), '时间 /s', 'roll err');
msplot(313, Time_axis_subsample, att_err_subsample_stg(:, 3), '时间 /s', 'yaw err');

% RK4 姿态误差
msplot(311, Time_axis_RK4, att_err_RK4_stg(:, 1), '时间 /s', 'pitch err');
title('RK4 algorithm');
msplot(312, Time_axis_RK4, att_err_RK4_stg(:, 2), '时间 /s', 'roll err');
msplot(313, Time_axis_RK4, att_err_RK4_stg(:, 3), '时间 /s', 'yaw err');

% 传感器输出
figure
subplot(321)
plot(Time_axis_data, imu_ref.fb(:, 1));xlabel('时间 /s');ylabel('fbx / m/s^2');hold on;
title('加计')
plot(Time_axis_data, imu_msr.fb(:, 1));
subplot(323)
plot(Time_axis_data, imu_ref.fb(:, 2));xlabel('时间 /s');ylabel('fby / m/s^2');hold on;
plot(Time_axis_data, imu_msr.fb(:, 2));
subplot(325)
plot(Time_axis_data, imu_ref.fb(:, 3));xlabel('时间 /s');ylabel('fbz / m/s^2');hold on;
plot(Time_axis_data, imu_msr.fb(:, 3));
subplot(322)
plot(Time_axis_data, imu_ref.wb(:, 1));xlabel('时间 /s');ylabel('wx / rad/s');hold on;
title('陀螺');
plot(Time_axis_data, imu_msr.wb(:, 1));
subplot(324)
plot(Time_axis_data, imu_ref.wb(:, 2));xlabel('时间 /s');ylabel('wy / rad/s');hold on;
plot(Time_axis_data, imu_msr.wb(:, 2));
subplot(326)
plot(Time_axis_data, imu_ref.wb(:, 3));xlabel('时间 /s');ylabel('wz / rad/s');hold on;
plot(Time_axis_data, imu_msr.wb(:, 3));
