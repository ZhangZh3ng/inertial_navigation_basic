%% **************************************************************
%名称： Gyroscope random drift 
%功能：模拟陀螺随机误差，专门针对于角度率输出的陀螺！
%@惯性仪器测试于数据分析 P245 D.4
%_________________________________________________________________________
%作者：哈尔滨工程大学 智能科学与工程学院 张峥
%日期：2020年12月3日
% ************************************************************************
%%
close all;
clear all;
global Gl
setGl_all;

% 随机常值漂移 
eb = 0.1*Gl.dph*randn(1, 1);
% @ @惯性仪器测试于数据分析 P155 (9.2-2)(9.2-4), P151
% 一阶马尔可夫过程的相关时间与方差
tauG = 50; beta = 1/tauG;
R0 = 0.01*Gl.dph^2;
% 角速率随机游走系数
q = 2*beta*R0;
% 观测噪声的功率谱和带宽
N2 = 0.0001*Gl.dph^2/Gl.Hz;
fB = 400*Gl.Hz;
% 采样频率，周期
fs = 10;
ts = 1/fs;
% 仿真时间长度
total_time = 600;
len = floor(total_time/ts);

er = zeros(len, 1);
er1 = sqrt(R0)*randn(1, 1);
% 一阶马尔可夫过程离散化
Phi = 1 - beta*ts;
sQkr = sqrt(q*ts);

for k =2:len
    er(k) = Phi*er(k - 1) + sQkr*randn(1, 1);
end

% 观测噪声均方差
sQkg = sqrt(N2*fB);
wg = sQkg*randn(len, 1);

%%
timeAxis = (1:1:len)*ts;
subplot(121)
plot(timeAxis, [eb + er + wg, eb + er]/Gl.dph); 
xlabel('\itt \rm/s'); ylabel('\it\epsilon \rm/(\circ)/h');

p1 = pwelch((er + wg)/Gl.dph, 1024);
p2 = pwelch(er/Gl.dph, 1024);
subplot(122)
% 功率谱
semilogy([0 : 512]*fs/1024, [p1/fs, p2/fs]);
xlabel('\itf\rm /Hz'); ylabel('\itS\epsilon \rm/ (\circ)/h^2/Hz');
