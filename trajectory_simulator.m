%% **************************************************************
%名称：trajectory simulator
%功能：模拟轨迹生成器
%程序@ 捷联惯导系统与组合导航原理 P229
%________________________________________________________________________
%
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月7日
% ************************************************************************
%%  轨迹生成
% 加载全局变量
gvar_earth;

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

[att, vn, pos] = trjprofile(att0, vn0, pos0, wat, ts);
% 这里用的avp2imu即为书上的av2imu函数 @P229
[wm, vm] = avp2imu(att, vn, pos, ts);

%%  作图
tt = (0 : length(att) - 1 )'*ts;

msplot(221, tt, att/arcdeg, 'Att/(\circ)');
legend('\it\theta', '\it\gamma', '\it\psi')

msplot(222, tt, vn, 'Vel /m.s^{-1}');
legend('\itv\rm_E', '\itv\rm_N', 'itv\rm_U')

msplot(223, tt, deltapos(pos), '\DeltaPos /m');
legend('\Delta\itL', '\Delta\it\lambda', '\Delta\ith')

% 横轴是经度，纵轴是纬度
msplot(224, pos(:, 2)/arcdeg, pos(:, 1)/arcdeg, '\it\lambda\rm /(\circ)', ...
      '\itL\rm /(\circ)');    hold on
plot(pos(1,2)/arcdeg, pos(1, 1)/arcdeg, 'ro');

% imu输出信息作图
msplot(121, tt(2 : end), wm/ts/arcdeg, '\it\omega^b_{ib}\rm /(circ.s^{-1})');
legend('\it\omega^b_{ibx}', '\it\omega^b_{iby}', '\it\omega^b_{ibz}');

msplot(122, tt(2 : end), vm/ts, '\itf^b\rm_{sf}/(m.s^{-2})');
legend('\itf^b\rm_{sf\itx}', '\itf^b\rm_{sf\ity}', '\itf^b\rm_{sf\itz}');
