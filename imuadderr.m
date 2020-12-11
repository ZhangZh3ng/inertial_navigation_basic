function [ vm, wm ] = imuadderr( vm, wm, imuerr, ts )
%% **************************************************************
%名称：imu add error
%功能：给真实imu输出添加误差，模拟实际中带有量测噪声的imu输出。
%程序@ 捷联惯导算法与组合导航原理 P235
%________________________________________________________________________
% 输入：
%       wm: n×3 矩阵，每一行对应一个角增量：[theta_x, theta_y, theta_z] rad
%       vm: n×3 矩阵，每一行对应一个速度增量 m/s
%       注意！wm和vm如果由四列，最后一列是对应的时间
%       eb: 陀螺仪常值漂移，即陀螺零偏
%       web:角度随机游走误差
%       db: 加速度计常值偏置
%       wdb: 速度随机游走误差
%       ts: 步长 s
% 输出：
%       wm: 添加了误差的角增量
%       vm: 添加了误差的速度增量
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月7日
% ************************************************************************
%%
% 数据组数
[m, n] = size(wm);

% 如果输入信息包含ts则认为imu信息是增量信息，否则认为是速度信息
if exist('ts', 'var')
    condition = 'increment';
else
    condition = 'velocity';
end

switch condition
    % ** imu输出为增量形式 **
    case 'increment'
        % 步长开平方
        sts = sqrt(ts);
        
        wm(:, 1:3) = wm(:, 1:3) + ...
            [ts*imuerr.eb(1) + sts*imuerr.web(1)*randn(m, 1), ...
             ts*imuerr.eb(2) + sts*imuerr.web(2)*randn(m, 1), ...
             ts*imuerr.eb(3) + sts*imuerr.web(3)*randn(m, 1)];
        
        vm(:, 1:3) = vm(:, 1:3) + ...
            [ts*imuerr.db(1) + sts*imuerr.wdb(1)*randn(m, 1), ...
             ts*imuerr.db(2) + sts*imuerr.wdb(2)*randn(m, 1), ...
             ts*imuerr.db(3) + sts*imuerr.wdb(3)*randn(m, 1)];
       
    % ** imu输出为比力和角速度 **    
    case 'velocity'
        wm(:, 1:3) = wm(:, 1:3) + ...
            [imuerr.eb(1) + imuerr.web(1)*randn(m, 1), ...
             imuerr.eb(2) + imuerr.web(2)*randn(m, 1), ...
             imuerr.eb(3) + imuerr.web(3)*randn(m, 1)];
        
        vm(:, 1:3) = vm(:, 1:3) + ...
            [imuerr.db(1) + imuerr.wdb(1)*randn(m, 1), ...
             imuerr.db(2) + imuerr.wdb(2)*randn(m, 1), ...
             imuerr.db(3) + imuerr.wdb(3)*randn(m, 1)];
end
end
