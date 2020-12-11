%% **************************************************************
%名称：angle motion and imu output simulation (version 1.0)
%功能：用户预设姿态，该程序模拟出姿态曲线所对应imu输出
%________________________________________________________________________
% 输入：
%       ang_motion: 用户设置的角运动
%  ang_motion.p(r,y)的每一行保存一个正弦运动对应的参数
%  theta = A*sin( w*t + phi) + k;
%  ang_motion.p = [A1(幅值), w1(角速度), phi1(初始相位), k1(中心值)
%                 A2(幅值), w2(角速度), phi2(初始相位), k2(中心值)]
%       Time: [total_time, ts, nn] 
% 输出：
%       att : 与设置的角运动所对应的姿态
%       imu : 包含 imu.fb_ref, imu.wb_ref, imu.fb_msr, imu.wb_msr
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月19日
% ************************************************************************
%%
close all;
clear all;

% 设置存储位置及目标文件名
saved_path = 'D:\data\simulate_data';
data_name = 'angle_motion_data';

% 全局变量设置
gvar_earth;
pos = [46*arcdeg, 126*arcdeg, 100]';
eth = earth(pos, [0, 0, 0]');

total_time = 3000;
ts = 0.01;

% 设置角运动1
ang_motion.p = [2.5*arcdeg, 10*arcdeg, 0*arcdeg, 0*arcdeg];
ang_motion.r = [1.5*arcdeg, 10*arcdeg, 0*arcdeg, 0*arcdeg];
ang_motion.y = [10*arcdeg, 12*arcdeg, 0*arcdeg, 0*arcdeg
                6*arcdeg, 10*arcdeg, 0*arcdeg, 0*arcdeg
                8*arcdeg, 5*arcdeg, 0*arcdeg, 0*arcdeg];

% % 设置角运动2
% ang_motion.p = [0*arcdeg, 0*arcdeg, 0*arcdeg, 0*arcdeg];
% ang_motion.r = [0*arcdeg, 0*arcdeg, 0*arcdeg, 0*arcdeg];
% ang_motion.y = [10*arcdeg, 5*arcdeg, 0*arcdeg, 0*arcdeg];

% 设置传感器误差
imuerr = imuerrorset('selfdefine');

% 为动态变量分配存储空间
att = zeros(fix(total_time/ts) + 10, 4);
imu_ref.fb = zeros(fix(total_time/ts) + 10, 4);
imu_ref.wb = zeros(fix(total_time/ts) + 10, 4);
%% simulation
% 当前时刻,注意crt从0开始计时,att中首元素为t=0时刻的载体姿态
crt = 0;
% 循环次数
Lp = 0;
while crt < total_time
    
    % 循环次数+1
    Lp = Lp+1;
    
    % 分别提取三个姿态角运动信息
    % 计算pitch(crt), roll(crt), yaw(crt)
    [pitch, wnb_p] = calculate_theta( ang_motion.p, crt );
    [roll, wnb_r] = calculate_theta( ang_motion.r, crt );
    [yaw, wnb_y] = calculate_theta( ang_motion.y, crt );
    
    % 姿态
    Cbn = a2mat([pitch, roll, yaw]');
    
    % wnbnx, 注意该角速度并不完全投影于n系，这里只是为了方便才这样表示
    wnbnx = [wnb_p, wnb_r, wnb_y]';
    
    % 计算wibb, Cnxb中的nx表示(n1, n2, b)这三个坐标系
    Cnxb = [ cos(roll), 0, -cos(pitch)*sin(roll)
                    0,  1,            sin(pitch)
            sin(roll),  0,  cos(pitch)*cos(roll)];
    wibb_ref = Cnxb*wnbnx + Cbn'*eth.winn;
    
    % 载体无线运动，加计输出应为b系下重力加速度
    fb_ref = Cbn'*eth.gn;
    
    % 保存结果
    att(Lp, :) = [pitch, roll, yaw, crt];
    imu_ref.fb(Lp, :) = [fb_ref', crt];
    imu_ref.wb(Lp, :) = [wibb_ref', crt];
    
    % 增加一个步长的时间，为下次计算做准备
    crt = crt + ts;
%     progressTip(crt);
end
% 删除存储变量中的空位
att(Lp+1:end, :) = [];
imu_ref.fb(Lp+1:end, :) = [];
imu_ref.wb(Lp+1:end, :) = [];

% imu_ref添加误差获得imu_msr
[ imu_msr.fb, imu_msr.wb ] = imuadderr(imu_ref.fb, imu_ref.wb, imuerr, ts);
%% 绘图
% 时间轴
Time_axis = (1 : 1 : Lp)*ts;

% 姿态角
msplot(311, Time_axis, att(:, 1)*deg, '时间 /s', 'pitch /\circ');
title('姿态角变化');
msplot(312, Time_axis, att(:, 2)*deg, '时间 /s', 'roll /\circ');
msplot(313, Time_axis, att(:, 3)*deg, '时间 /s', 'yaw /\circ');

% imu.fb_ref
msplot(321, Time_axis, imu_ref.fb(:, 1), '时间 /s', 'fbx m/s^2');
title('加计标准输出');
msplot(323, Time_axis, imu_ref.fb(:, 2), '时间 /s', 'fby m/s^2');
msplot(325, Time_axis, imu_ref.fb(:, 3), '时间 /s', 'fbz m/s^2');
% imu.wb_ref
msplot(322, Time_axis, imu_ref.wb(:, 1)*deg, '时间 /s', 'wibb x rad/s');
title('陀螺标准输出');
msplot(324, Time_axis, imu_ref.wb(:, 2)*deg, '时间 /s', 'wibb y rad/s');
msplot(326, Time_axis, imu_ref.wb(:, 3)*deg, '时间 /s', 'wibb z rad/s');

% imu fb_msr
msplot(321, Time_axis, imu_msr.fb(:, 1), '时间 /s', 'fbx m/s^2');
title('加计测量值');
msplot(323, Time_axis, imu_msr.fb(:, 2), '时间 /s', 'fby m/s^2');
msplot(325, Time_axis, imu_msr.fb(:, 3), '时间 /s', 'fbz m/s^2');
% imu.wb_msr
msplot(322, Time_axis, imu_msr.wb(:, 1)*deg, '时间 /s', 'wibb x rad/s');
title('陀螺测量值');
msplot(324, Time_axis, imu_msr.wb(:, 2)*deg, '时间 /s', 'wibb y rad/s');
msplot(326, Time_axis, imu_msr.wb(:, 3)*deg, '时间 /s', 'wibb z rad/s');

%% 保存数据
% 相关信息
dataInfo.descreption = ...
    'This attitude trajectory combination of sine angular montion';
dataInfo.name = data_name;
dataInfo.date = datestr(now);
dataInfo.type = 'simulation data';
dataInfo.ts = ts;
dataInfo.pos = pos;
dataInfo.imuerr = imuerr;
dataInfo.length = length(att);

% 为便于管理，将轨迹生成器产生的数据保存到同一个文件夹
cd(saved_path);
save(data_name, 'att', 'imu_ref', 'imu_msr', 'dataInfo');
cd('D:\inertial_navigation_basic');
% 清除不需要的变量
clearvars -except att imu_ref imu_msr dataInfo
%% 专用函数
function [ theta, omega ] = calculate_theta( ang_motion, crt )
%% **************************************************************
%名称：Calculate theta(angule) form the seted angule motion 
%功能：给定预设角运动形式，计算current时刻的角度,专用子函数
%注意：目前还没有添加规范化姿态角的功能，因此尽量不要把摇摆幅值设的过大
%________________________________________________________________________
% 输入：
%       ang_motion: 预设角运动，每一行对应着一个形如：
%                   theta = A*sin( w*t + phi) + k的运动，载体角运动由若干个
%                   正弦运动叠加而成
%       crt: 当前时间
% 输出：
%       theta: 跟预设角运动所对应的姿态 (rad)
%       omega: 与预设角运动所对应的当前时刻角速度 (rad/s)
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月19日
% ************************************************************************
%%
% i表示 ang_motion 中的第i行
i = 1;

% 如果ang_motion中存在第i行
while i <= size(ang_motion, 1)
    
    if i == 1
        theta = 0;  % rad
        omega = 0;  % rad/s
    end
    
    % ang_motion.p = [A1(幅值), w1(角速度), phi1(初始相位), k1(中心值)
    %                 A2(幅值), w2(角速度), phi2(初始相位), k2(中心值)
    %                                    ...                         ]
    % 注意: ang_motion中变量的单位为rad或rad/s
    A = ang_motion(i, 1);
    w = ang_motion(i, 2);
    phi = ang_motion(i, 3);
    k = ang_motion(i, 4);
    
    % 姿态角
    theta = A*sin( w*crt + phi) + k + theta;
    
    % wnbnx    nx分别是(n n1), (n1 n2), (n1 b)
    omega = w*A*cos(w*crt + phi) + omega;
    
    % i+1, 准备检查ang_motion是否存在下一行
    i = i+1;
    
end

end
