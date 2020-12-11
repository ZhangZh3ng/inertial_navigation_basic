function [] = setGl_unit()
%% **************************************************************
%名称：Set Global Variable unit
%功能：设置全局变量 单位
%_________________________________________________________________________
%作者：哈尔滨工程大学 智能科学与工程学院 张峥
%日期：2020年12月2日
% ************************************************************************
%%
% 我想将角Alpha的大小设置为30deg，但表征角度大小的国际单位是rad，所以我需要
% 给30乘以一个系数以将其转化为国际单位下相应的大小，这个系数就是Gl.deg.
% 例子2，我需要wie(15deg/h)在国际单位(rad/s)下的大小，令wie=15*Gl.dph即可.
% 反过来讲，我想知道4000m等于多少海里，令4000/Gl.nm.
global Gl
Gl.Call_unit = true;

% 时间 s
Gl.s = 1; % 时间国际单位
Gl.ms = 1e-3; % 1ms = 0.001 (s)
Gl.h = 3600; % 1hour = 3600 (s)
Gl.day =  86400; % 1day =  86400 (s)
% 长度 m
Gl.m = 1; % 长度国际单位
Gl.nm = 1853;  % 1海里 = 1853 (m)
% 角度 rad
Gl.rad = 1; % 角的大小 国际单位
Gl.deg = pi/180; % 1deg = pi/180 (rad)
Gl.min = pi/180/60; % 1min = pi/180/60 (rad)
Gl.sec = pi/180/3600; % 1sed = pi/180/3600 (rad)
Gl.rad2deg = 180/pi; % 1rad = 180/pi (deg)
Gl.rad2min = 60*180/pi; % 1rad = 60*180/pi (min)
Gl.rad2sec = 3600*180/pi; % 1rad = 3600*180/pi (sec)
Gl.meru = 7.2921151467e-8; % 毫地转角速率
% 频率 Hz
Gl.Hz = 1; % 1Hz = 1*(1/s)
Gl.ppm = 1e-6; % 百万分之一
% 速度 m/s
Gl.mps = 1; % 速度
Gl.kmph = 1/3.6; % 1km/h = 1000/3600 (m/s)
Gl.knot = 1853/3600; % 1节 = 1853/3600 (m/s)
% 加速度 m/s^2
Gl.mps2 = 1; % 加速度国际单位
% Gl.g = 9.780325333434361; % 赤道重力加速度
Gl.mg = 9.780325333434361*1e-3; % 千分之一赤道重力加速度
Gl.ug = 9.780325333434361*1e-6; % 百万分之一赤道重力加速度
% 角速度 rad/s
Gl.rps = 1; % 角速度国际单位
Gl.dps = pi/180/1; % 1deg/s = pi/180/1 (rad/s)
Gl.dpmin = pi/180/60; % 1deg/min = pi/180/60 (rad/s)
Gl.dph = pi/180/3600; % 1deg/h = pi/180/3600 (rad/s)

%% 随机游走系数
% @ 捷联惯导算法与组合导航原理 P131,132
% @ 惯性仪器测试与数据分析 P145,P157

% 连续时间卡尔曼滤波(Continuous-Time Kalman Filter, CTKF)中,对应状态xi(U)的
% 状态噪声为wi。因为CTKF的状态方程是微分方程，所以wi的单位为U/s。
% CTKF状态噪声的协方差矩阵 E(wi*wi') = q*delta, sqrt(q)称为激励噪声密度或随机
% 游走系数。关于如何使用随机游走系数，在惯性仪器测试与数据分析P157有非常好的例子。

% CTKF的状态方程是微分方程，wi的单位是U/s,通过对微分方程获得熟悉的离散时间
% 卡尔曼滤波，状态噪声变为Wi(U).  E(Wk*Wj') = Qk*delta(k,j) = q*ts*delta(k,j)
% 在惯导误差连续状态方程的离散化中，Qk表示平台失准角的角度激励噪声方差。描述
% 角速率输出型陀螺噪声时，常用角速率功率谱密度和等效带宽参数,Qk = Sb*fb*ts^2；
% 描述角增量型输出陀螺噪声时，常用角度随机游走系数,Qk = N^2*ts

Gl.sqs = 1; % sqrt(1s) = 1(1/s^1/2) = sqrt(Hz)
Gl.sqh = 60; % sqrt(3600s) = 60 (1/s^1/2)
Gl.sqhz = 1; % sqrt(Hz) 
% 角度随机游走系数 rad/s^1/2    对应驱动失准角的白噪声
Gl.dpss = Gl.deg/Gl.sqs; % deg / sqrt(s)
Gl.dpsh = Gl.deg/Gl.sqh; % deg / sqrt(h)
% 速度随机游走系数 m/s/s^1/2        对应驱动速度误差的白噪声
Gl.mpspss = 1/Gl.sqs; % m/s / sqrt(s)
Gl.mpspsh = 1/Gl.sqh; % m/s / sqrt(h) 
% 位置随机游走系数 m/s^1/2      对应驱动位置误差的白噪声
Gl.mpss = 1/Gl.sqs; % m / sqrt(s)
Gl.mpsh = 1/Gl.sqh; % m / sqrt(h)
% 角速度随机游走系数 rad/s^3/2       对应驱动陀螺漂移（零偏）的白噪声
Gl.dphpsh = Gl.dph/Gl.sqh; % deg/h / sqrt(h)

% 零偏不稳定性 rad/s
% Gl.dph = Gl.dph; % deg/h 

% 加速度随机游走 m/s^2 / sqrt(s)     对应驱动加计零偏的白噪声
Gl.ugpsh = Gl.ug/Gl.sqh; % ug / sqrt(h)
% 刻度系数误差随机游走 ppm/sqrt(s)    对应驱动刻度系数误差的白噪声
Gl.ppmpsh = Gl.ppm/Gl.sqh; % ppm / sqrt(h)
% 安装误差随机游走系数 sec / sqrt(s)
Gl.secpsh = Gl.sec/Gl.sqh; % sec / sqrt(h)

Gl.dphpsHz = Gl.dph/Gl.sqhz;   % deg/h / sqrt(Hz)
Gl.ugpsHz = Gl.ug/Gl.sqhz;  % ug / sqrt(Hz)

% clear Gl.sqs Gl.sqh Gl.sqhz
end