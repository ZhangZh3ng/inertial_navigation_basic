%% **************************************************************
%名称：trajectory simulator
%功能：轨迹模拟生成器
%程序@ 捷联惯导系统与组合导航原理 P229
%________________________________________________________________________
% 最后计算获得并保存的结果是
% avp_SD.(pos, vn, att): 位置、速度、姿态参考值
% imu_SD.(wb, fb): 角增量和速度增量
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月7日
% ************************************************************************
%%  轨迹生成
% 加载全局变量
gvar_earth;
% 步长
ts = 0.01;

% 初始姿态、速度、位置
att0 = [0, 0, 90]'*arcdeg;
vn0 = [0, 0, 0]';
pos0 = [34*arcdeg, 108*arcdeg, 100]';

% 预设轨迹
%   w_pitch   w_roll   w_yaw   vb_y    time
wat = [0,       0,      0,      0,      10      %静止
       0,       0,      0,      1,      10      %加速
       0,       0,      0,      0,      10      %匀速
       5,       0,      0,      0,      4       %抬头
       0,       0,      0,      0,      10      %匀速
       -5,      0,      0,      0,      4       %低头
       0,       0,      0,      0,      10      %匀速
       0,       10,     0,      0,      1       %横滚
       0,       0,      9,      0,      10      %转弯
       0,       -10,    0,      0,      1       %横滚
       0,       0,      0,      0,      10      %匀速
       0,       0,      0,      -1,     10      %减速
       0,       0,      0,      0,      10    ];%静止

% 把用deg/s表示的角速度转化成rad/s
wat(:, 1:3) = wat(:, 1:3)*pi/180;

[avp_SD.att, avp_SD.vn, avp_SD.pos] = trjprofile(att0, vn0, pos0, wat, ts);
% 这里用的avp2imu即为书上的av2imu函数 @P229
[imu_SD.wb, imu_SD.fb] = avp2imu(avp_SD.att, avp_SD.vn, avp_SD.pos, ts);

% 在imu数据中"插入" t=0时的imu输出,即(-ts,0]这段时间内的角增量和速度增量。
% 这样做使得imu_SD与avp_SD拥有相同的长度，便于数据的调用。
imu_SD.wb = [[0, 0, 0]; imu_SD.wb];
imu_SD.fb = [[0, 0, 0]; imu_SD.fb];

% 保存数据
save('trajectory_simulator_data.mat', 'avp_SD', 'imu_SD');
%%  作图
tt = (0 : length(avp_SD.att) - 1 )'*ts;

msplot(221, tt, avp_SD.att/arcdeg, 'Att/(\circ)');
legend('\it\theta', '\it\gamma', '\it\psi')

msplot(222, tt, avp_SD.vn, 'Vel /m.s^{-1}');
legend('\itv\rm_E', '\itv\rm_N', 'itv\rm_U')

msplot(223, tt, deltapos(avp_SD.pos), '\DeltaPos /m');
legend('\Delta\itL', '\Delta\it\lambda', '\Delta\ith')

% 横轴是经度，纵轴是纬度
msplot(224, avp_SD.pos(:, 2)/arcdeg, avp_SD.pos(:, 1)/arcdeg, ...
    '\it\lambda\rm /(\circ)', '\itL\rm /(\circ)');    hold on
plot(avp_SD.pos(1,2)/arcdeg, avp_SD.pos(1, 1)/arcdeg, 'ro');

% imu输出信息作图
msplot(121, tt, imu_SD.wb/ts/arcdeg, ...
    '\it\omega^b_{ib}\rm /(circ.s^{-1})');
legend('\it\omega^b_{ibx}', '\it\omega^b_{iby}', '\it\omega^b_{ibz}');

msplot(122, tt, imu_SD.fb/ts, '\itf^b\rm_{sf}/(m.s^{-2})');
legend('\itf^b\rm_{sf\itx}', '\itf^b\rm_{sf\ity}', '\itf^b\rm_{sf\itz}');
