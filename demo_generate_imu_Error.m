%% **************************************************************
%名称：Example generate imu error 
%功能：示例程序: 生成imu误差
%_________________________________________________________________________
%作者：哈尔滨工程大学 智能科学与工程学院 张峥
%日期：2020年12月4日
% ************************************************************************
%%
close all;
clear all;
global Gl;
setGl_all;

% 采样频率、imu相邻两次输出之间时间间隔
fs = 10; % Hz
ts = 1/fs;
% 模拟时间 s
total_time = 600;
length = floor(total_time/ts);

%   eb=0.01dph, db=50ug, web=0.001dpsh, wdb=10ugpsHz
%   scale factor error=10ppm, askew installation error=10arcsec
%   sqrtR0G=0.001dph, taug=1000s, sqrtR0A=10ug, taug=1000s
imuError.Gbias = 0.1*randn(1, 1); % deg/h
% imuError.GN = 0.01; % deg/sqrt(Hz)
imuError.GN2 = 0.0001; % (deg/h)^2/Hz
imuError.GfB = 400; % Hz
imuError.Gtau = 50; % s
imuError.GR0 = 0.01; % (deg/h)^2

imuError.Abias = 50; % ug
% imuError.AN = 10; % ug/sqrt(HZ)
imuError.AN2 = 0.01; % (ug)^2/Hz
imuError.AfB = 400; % Hz

imuError.Atau = 50; % s
imuError.AR0 = 100; % ug^2

% add error
[gyrError, Geb, Gwg, Ger] = generateGyrError(length, ts, imuError);
[accError, Aeb, Awg, Aer] = generateAccError(length, ts, imuError);
%% 输出和绘图
timeAxis = (1:1:length)*ts;
figureNum = 1;

gyrUnit = Gl.dph;

figure(figureNum)
figureNum = figureNum + 1;
subplot(121)
plot(timeAxis, gyrError/gyrUnit); hold on;
plot(timeAxis, (Geb+Ger)/gyrUnit);
legend('含白噪声', '不含白噪声');
xlabel('\itt \rm/s'); ylabel('\it\epsilon \rm/(\circ)/h');

% p1 = pwelch((Ger + Gwg)/gyrUnit, 1024);
% p2 = pwelch(Ger/gyrUnit, 1024);
% subplot(122)
% % 功率谱
% semilogy([0 : 512]*fs/1024, [p1/fs, p2/fs]);
% xlabel('\itf\rm /Hz'); ylabel('\itS\epsilon \rm/ (\circ)/h^2/Hz');

figure(figureNum)
figureNum = figureNum+1;
subplot(211)
plot(timeAxis, Ger/gyrUnit); xlabel('\itt \rm/s'); ylabel('\it\epsilon \rm/(\circ)/h');
title('陀螺一阶马尔可夫噪声');
subplot(212)
plot(timeAxis, Gwg/gyrUnit);xlabel('\itt \rm/s'); ylabel('\it\epsilon \rm/(\circ)/h');
title('陀螺白噪声');

std(Gwg/gyrUnit)*3600;

% 加计
accUnit = Gl.ug;
figure(figureNum)
figureNum = figureNum+1;
plot(timeAxis, accError/accUnit); xlabel('时间 /s'); ylabel('ug'); hold on;
plot(timeAxis, (Aeb + Aer)/accUnit);
legend('含白噪声', '不含白噪声');

figure(figureNum)
figureNum = figureNum+1;
subplot(211)
plot(timeAxis, Aer/accUnit); xlabel('时间 /s'); ylabel('ug');
title('加计一阶马尔可夫噪声');
subplot(212)
plot(timeAxis, Awg/accUnit); xlabel('时间 /s'); ylabel('ug');
title('加计白噪声');
std(Awg/accUnit)