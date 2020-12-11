% ************************************************************************
% SINS
% Function: A demonstration of SINS
% 
% Descrition: 
%
% Version: 1.0	
% Created on 2020/12/4
%
% Copyright(c) 2020，Zhang Zheng 
% College of Intelligent Systems Science and Engineering, Harbin
% Engineering University, Harbin, China.
% ************************************************************************
%% initialization
% Whether from the perspective of program accuracy or readability, it is
% worthwhile to add the corresponding units in calculation (at least when
% defining orassigning variables). However, considering that a considerable
% number of variable units are the corresponding international units, in
% consideration of the simplicity of the program, the international units
% are omitted without causing ambiguity.
close all;
clear all;
global Gl
setGl_all;

% load data
data_folder = 'D:\data\simulate_data\2020年12月';
data_name = 'simulation_data_2020年12月9日_1';
data_fullPath = strcat(data_folder,'\', data_name, '.mat');
load(data_fullPath);

% subsample number
subsampleNum = 2;
% time
length_imu = length(imu.tb);
ts_imu = dataInfo.ts;
f_imu = 1/ts_imu;
nts = subsampleNum*ts_imu;
start_time = 0; % s
stop_time = 100; % s
k_init = floor(start_time/ts_imu) + 1;

% *** add imu error ***
imuError = selectImuError('selfdefine', 'increment'); % 
[imu_msr, gyrError, accError] = addImuError(imu, imuError, ts_imu);

% set initial navigation information (without initial error)
pos_init = avp.pos(k_init, :)';
vn_init = avp.vn(k_init, :)';
att_init = avp.att(k_init, :)';
% initial navigation information that we use
pos_prev = pos_init + [0*Gl.sec, 0*Gl.sec, 0*Gl.m]';
vn_prev = vn_init + 0*rand(3, 1);
att_prev = att_init + [0*Gl.deg, 0*Gl.deg, 0*Gl.deg]';
qbn_prev = a2qua(att_prev);

% *** preallocating memory ***
room = zeros(round(length_imu/subsampleNum)+10, 3);
position_calc_stg = room;   position_ref_stg = room;    position_err_stg = room;
velocity_calc_stg = room;   velocity_ref_stg = room;    velocity_err_stg = room;
attitude_calc_stg = room;   attitude_ref_stg = room;

%% computing
current = start_time; % s
% How much times the program enter the coresponding loop.
Lp_msr = 0; % times
Lp_update = 0; 
while current <= stop_time 
    Lp_msr = Lp_msr+1;
    % Data index at this moment. Perhaps we don't need this parameter in 
    % every measurement loop, but we still update it.
    k = floor(current/ts_imu) + k_init;
    
    % SINS update loop
    flag_update = false;
    % When Lp = subsampleNum*i + 1 (i=1,2,3...), we update.
    if (Lp_msr > subsampleNum) && (mod(round(Lp_msr-1), subsampleNum) == 0 )
        flag_update = true;
        Lp_update = Lp_update+1;      
        % read raw imu measurement
        gyrOutput = imu_msr.tb( k-subsampleNum+1 : k, :);
        accOutput = imu_msr.vb( k-subsampleNum+1 : k, :);
        
        % The following content can be simplified by function 'insupdate' 
        % compensate coning and sculling motion error.
        [phim, dvbm] = cnscl(gyrOutput, accOutput);  
        % caculate some important earth rotation velocity.
        eth = developing_earth(pos_prev, vn_prev);
        % velocity update 
        vn = vn_prev + rv2m(-eth.winn*nts/2)*qmulv(qbn_prev, dvbm) + eth.gcc*nts;
        vn_avg = (vn_prev + vn)/2;  
        % position update 
        pos = pos_prev + [vn_avg(2)/eth.RMh, vn_avg(1)/eth.clRNh, vn_avg(3)]'*nts;
        % 天向阻尼
%         vn(3) = 0; 
%         pos(3) = pos_init(3);
        % attitude updete 
        qbn = qmul(rv2q(-eth.winn*nts), qmul(qbn_prev, rv2q(phim)));
        qbn = qnormlz(qbn);  
    end
    
    % save result
    if flag_update
    position_calc_stg(Lp_update, :) = pos';	
    velocity_calc_stg(Lp_update, :) = vn';
    attitude_calc_stg(Lp_update, :) = q2att(qbn)';
    % reference
    position_ref_stg(Lp_update, :) = avp.pos(k, :)';
    velocity_ref_stg(Lp_update, :) = avp.vn(k, :)';
    attitude_ref_stg(Lp_update, :) = avp.att(k, :)';
    % error
    position_err_stg(Lp_update, :) = diag(pos' - avp.pos(k, :)');
    velocity_err_stg(Lp_update, :) = diag(vn' - avp.vn(k, :)');
    end
    
    % Time of next measurement Loop.
    current = current + ts_imu;
    if flag_update
        pos_prev = pos;
        vn_prev = vn;
        qbn_prev = qbn;
    end
    progressTip(current, ts_imu, 500);
end
% clear the unused room
position_calc_stg(Lp_update+1 : end, :) = [];   position_ref_stg(Lp_update+1 : end, :) = [];
velocity_calc_stg(Lp_update+1 : end, :) = [];   velocity_ref_stg(Lp_update+1 : end, :) = [];
attitude_calc_stg(Lp_update+1 : end, :) = [];   attitude_ref_stg(Lp_update+1 : end, :) = [];
position_err_stg(Lp_update+1 : end, :) = [];
velocity_err_stg(Lp_update+1 : end, :) = [];
%% output result and plot
timeAxis = (1:1:length_imu)*ts_imu;
timeAxis_update = (1:1:Lp_update)*nts;
figureNum = 1;
gyrErrorUnit = Gl.rps*ts_imu;
accErrorUnit = Gl.mps2*ts_imu;

figure(figureNum)
figureNum = figureNum+1;
subplot(311)
plot(timeAxis, gyrError(:, 1)/gyrErrorUnit); xlabel('时间 /s'); ylabel('x轴陀螺误差 / rad/s')
mean(gyrError(:, 1)/gyrErrorUnit); title('陀螺误差');
subplot(312)
plot(timeAxis, gyrError(:, 2)/gyrErrorUnit); xlabel('时间 /s'); ylabel('y轴陀螺误差 / rad/s')
subplot(313)
plot(timeAxis, gyrError(:, 3)/gyrErrorUnit); xlabel('时间 /s'); ylabel('z轴陀螺误差 / rad/s')

figure(figureNum)
figureNum = figureNum+1;
subplot(311)
plot(timeAxis, accError(:, 1)/accErrorUnit); xlabel('时间 /s'); ylabel('x轴加计误差 / m/s^2')
mean(accError(:, 1)/accErrorUnit); title('加计误差');
subplot(312)
plot(timeAxis, accError(:, 2)/accErrorUnit); xlabel('时间 /s'); ylabel('y轴加计误差 / m/s^2')
subplot(313)
plot(timeAxis, accError(:, 3)/accErrorUnit); xlabel('时间 /s'); ylabel('z轴加计误差 / m/s^2')
 
%%
% figure(figureNum)
% figureNum = figureNum+1;
% subplot(311)
% plot(timeAxis, imu.tb(:, 1)/gyrUnit); hold on; mean(imu.tb(:, 1)/gyrUnit);
% plot(timeAxis, imu_msr.tb(:, 1)/gyrUnit);
% subplot(312)
% plot(timeAxis, imu.tb(:, 2)/gyrUnit); hold on;
% plot(timeAxis, imu_msr.tb(:, 2)/gyrUnit);
% subplot(313)
% plot(timeAxis, imu.tb(:, 3)/gyrUnit); hold on;
% plot(timeAxis, imu_msr.tb(:, 3)/gyrUnit);
% 
% figure(figureNum)
% figureNum = figureNum+1;
% subplot(311)
% plot(timeAxis, imu.vb(:, 1)/accUnit); hold on; mean(imu.vb(:, 1)/accUnit);
% plot(timeAxis, imu_msr.vb(:, 1)/accUnit);
% subplot(312)
% plot(timeAxis, imu.vb(:, 2)/accUnit); hold on;
% plot(timeAxis, imu_msr.vb(:, 2)/accUnit);
% subplot(313)
% plot(timeAxis, imu.vb(:, 3)/accUnit); hold on
% plot(timeAxis, imu_msr.vb(:, 3)/accUnit);
%%
% 位置误差
figure(figureNum)
figureNum = figureNum+1;
plot(timeAxis_update, Gl.Re*position_err_stg(:, 1)); hold on; xlabel('时间 /s'); ylabel('误差量 /m');
plot(timeAxis_update, Gl.Re*position_err_stg(:, 2)); hold on;
plot(timeAxis_update, position_err_stg(:, 3)); 
title('位置误差')
legend('\DeltaL', '\Delta\lambda', '\Deltah');

% 速度误差
figure(figureNum)
figureNum = figureNum+1;
plot(timeAxis_update, velocity_err_stg(:, 1)); hold on; xlabel('时间 /s'); ylabel('速度误差 / m/s');
plot(timeAxis_update, velocity_err_stg(:, 2)); hold on;
plot(timeAxis_update, velocity_err_stg(:, 3));
title('速度误差');
legend('\DeltavE', '\DeltavN', '\DeltavU');