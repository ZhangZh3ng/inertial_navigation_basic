%% **************************************************************
%名称：Example: Static SINS 
%纯惯导程序
%_________________________________________________________________________
%作者：哈尔滨工程大学 智能科学与工程学院 张峥
%日期：2020年11月29日
% ************************************************************************
%% 初始化
close all;
clear all;

% 全局变量
gvar_earth;

% *** 读入数据 ***
data_folder = 'D:\data\simulate_data\2020年11月30日';
data_name = 'simData_2020年11月30日_1';
data_path = strcat(data_folder,'\', data_name, '.mat');
simulate_data_1 = load(data_path);

% 采用的数据
data = simulate_data_1;
disp(data.dataInfo.description);
% imu输出间隔
ts_imu = data.dataInfo.ts;
% 更新算法子样数
subsampleNum = 2;
% ins更新间隔
nts = subsampleNum*ts_imu;

% *** 选择程序的开始时间和截至时间 ***
% 开始时间
start_time = 0;
% 截至时间
end_time = 80000;
% 开始时刻的数据在所有数据中的脚标
k_init = floor(start_time/ts_imu) + 1;

% *** 设置导航参数初值 ***
position_init = data.avp.pos(k_init, :)';
velocity_init = data.avp.vn(k_init, :)';
attitude_init = data.avp.att(k_init, :)';

pos = position_init;
vn = velocity_init;
qbn = a2qua(attitude_init);
phi = [0.1, 0.2, 3]'*arcmin;
qbn = qaddphi(qbn, phi);

% *** 为存储变量预分配内存 ***
storage = zeros( ceil(end_time/ts_imu), 3);
position_calc_stg = storage;    position_ref_stg = storage;     position_err_stg = storage;
velocity_calc_stg = storage;    velocity_ref_stg = storage;     velocity_err_stg = storage;
attitude_calc_stg = storage;    attitude_ref_stg = storage;

%% 循环解算

Lp_update = 0;
crt = 0;
while crt < end_time
    
    % 惯导更新循环
    Lp_update = Lp_update+1;
    % 时间增加nts
    crt = crt + nts;
    
    % *** 提取imu输出 ***
    % 进行惯导解算的时刻的数据在所有数据中的脚标
    k = round(Lp_update*nts/ts_imu) + k_init;
    angle_increment = data.imu.tb( k-subsampleNum+1 : k, :);
    velocity_increment = data.imu.vb( k-subsampleNum+1 : k, :);
    
    %惯导更新
    [ qbn, vn, pos, eth ] = ...
        insupdate( qbn, vn, pos, angle_increment, velocity_increment, ts_imu );
    
    % 高度阻尼
%     pos(3) = position_init(3);
%     vn(3) = 0;
    
    % *** 保存计算结果 ***
    % 储存计算导航信息
    position_calc_stg(Lp_update, :) = pos';
    velocity_calc_stg(Lp_update, :) = vn';
    attitude_calc_stg(Lp_update, :) = q2att(qbn)';
    % 参考信息
    position_ref_stg(Lp_update, :) = data.avp.pos(k, :)';
    velocity_ref_stg(Lp_update, :) = data.avp.vn(k, :)';
    attitude_ref_stg(Lp_update, :) = data.avp.att(k, :)';
    % 误差信息
    position_err_stg(Lp_update, :) = diag(pos' - data.avp.pos(k, :)');
    velocity_err_stg(Lp_update, :) = diag(vn' - data.avp.vn(k, :)');
    
    % 显示运行进度
    progressTip(crt, nts, 5000);
end
% 清除未使用的空间
position_calc_stg(Lp_update+1 : end, :) = [];
velocity_calc_stg(Lp_update+1 : end, :) = [];
attitude_calc_stg(Lp_update+1 : end, :) = [];
position_ref_stg(Lp_update+1 : end, :) = [];
velocity_ref_stg(Lp_update+1 : end, :) = [];
attitude_ref_stg(Lp_update+1 : end, :) = [];
position_err_stg(Lp_update+1 : end, :) = [];
velocity_err_stg(Lp_update+1 : end, :) = [];

%% 输出和绘图
timeAxis = (1:1:Lp_update)*nts;
figureNum = 1;

% % 位置
% figure(figureNum)
% figureNum = figureNum+1;
% subplot(311)
% plot(timeAxis, deg*position_calc_stg(:, 1)); hold on;
% plot(timeAxis, deg*position_ref_stg(:, 1)); xlabel('时间 /s'); ylabel('纬度 /deg');
% title('位置')
% legend('计算值', '真实值');
% subplot(312)
% plot(timeAxis, deg*position_calc_stg(:, 2)); hold on;
% plot(timeAxis, deg*position_ref_stg(:, 2));xlabel('时间 /s'); ylabel('经度 /deg');
% subplot(313)
% plot(timeAxis, position_calc_stg(:, 3)); hold on;
% plot(timeAxis, position_ref_stg(:, 3));xlabel('时间 /s'); ylabel('高度 /deg');
% 
% % 速度
% figure(figureNum)
% figureNum = figureNum+1;
% subplot(311)
% plot(timeAxis, velocity_calc_stg(:, 1)); hold on;
% plot(timeAxis, velocity_ref_stg(:, 1)); xlabel('时间 /s'); ylabel('vE / m/s');
% title('速度');
% legend('计算值', '真实值');
% subplot(312)
% plot(timeAxis, velocity_calc_stg(:, 2)); hold on;
% plot(timeAxis, velocity_ref_stg(:, 2)); xlabel('时间 /s'); ylabel('vN / m/s');
% subplot(313)
% plot(timeAxis, velocity_calc_stg(:, 3)); hold on;
% plot(timeAxis, velocity_ref_stg(:, 3)); xlabel('时间 /s'); ylabel('vU / m/s');
% 
% % 姿态
% figure(figureNum)
% figureNum = figureNum+1;
% subplot(311)
% plot(timeAxis, deg*attitude_calc_stg(:, 1)); hold on;
% plot(timeAxis, deg*attitude_ref_stg(:, 1)); xlabel('时间 /s'); ylabel('俯仰角 / deg');
% title('姿态');
% legend('计算值', '真实值');
% subplot(312)
% plot(timeAxis, deg*attitude_calc_stg(:, 2)); hold on;
% plot(timeAxis, deg*attitude_ref_stg(:, 2)); xlabel('时间 /s'); ylabel('横滚角 / deg');
% subplot(313)
% plot(timeAxis, deg*attitude_calc_stg(:, 3)); hold on;
% plot(timeAxis, deg*attitude_ref_stg(:, 3)); xlabel('时间 /s'); ylabel('航向角 / deg');

% 位置误差
figure(figureNum)
figureNum = figureNum+1;
plot(timeAxis, Re*position_err_stg(:, 1)); hold on; xlabel('时间 /s'); ylabel('误差量 /m');
plot(timeAxis, Re*position_err_stg(:, 2)); hold on;
plot(timeAxis, position_err_stg(:, 3)); 
title('位置误差')
legend('\DeltaL', '\Delta\lambda', '\Deltah');

% 速度误差
figure(figureNum)
figureNum = figureNum+1;
plot(timeAxis, velocity_err_stg(:, 1)); hold on; xlabel('时间 /s'); ylabel('速度误差 / m/s');
plot(timeAxis, velocity_err_stg(:, 2)); hold on;
plot(timeAxis, velocity_err_stg(:, 3));
title('速度误差');
legend('\DeltavE', '\DeltavN', '\DeltavU');

% 姿态角
figure(figureNum)
figureNum = figureNum+1;
subplot(121)
plot(timeAxis, deg*attitude_calc_stg(:, 1)); hold on; xlabel('时间 /s'); ylabel('角度 / deg');
plot(timeAxis, deg*attitude_calc_stg(:, 2));
title('俯仰角和横滚角');
legend('俯仰角', '横滚角');
subplot(122)
plot(timeAxis, deg*attitude_calc_stg(:, 3)); xlabel('时间 /s'); ylabel('航向角 /deg');
title('航向角计算值');
