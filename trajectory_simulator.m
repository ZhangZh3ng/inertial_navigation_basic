%% **************************************************************
%名称：trajectory simulator
%功能：轨迹模拟生成器
%程序@ 捷联惯导系统与组合导航原理 P229
%________________________________________________________________________
% 最后计算获得并保存的结果是
% avp.(pos, vn, att): 位置、速度、姿态参考值
% imu.(wb, fb): 角增量和速度增量
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月7日
% ************************************************************************
%%  轨迹生成
% 设置保存数据时的文件名
data_folder = 'D:\data\simulate_data\2020年12月';
data_name = 'simulation_data_2020年12月10日_1';

% 加载全局变量
gvar_earth;
% imu输出两次之间的时间间隔
ts = 0.01;

% 初始姿态、速度、位置
att0 = [1, 1, 30]'*arcdeg;
vn0 = [0, 0, 0]';
pos0 = [34*arcdeg, 108*arcdeg, 100]';

% 预设轨迹
wat = developing_wat_select( '20201210' );

% 把用deg/s表示的角速度转化成rad/s
wat(:, 1:3) = wat(:, 1:3)*pi/180;

[avp.att, avp.vn, avp.pos] = trjprofile(att0, vn0, pos0, wat, ts);
% 这里用的avp2imu即为书上的av2imu函数 @P229
[imu.tb, imu.vb] = avp2imu(avp.att, avp.vn, avp.pos, ts);

% 在imu数据中"插入" t=0时的imu输出,即(-ts,0]这段时间内的角增量和速度增量。
% 这样做使得imu与avp拥有相同的长度，便于数据的调用。
imu.tb = [[0, 0, 0]; imu.tb];
imu.vb = [[0, 0, 0]; imu.vb];

%%  作图
tt = (0 : length(avp.att)-1)'*ts;

msplot(221, tt, avp.att/arcdeg, 'Att/(\circ)');
legend('\it\theta', '\it\gamma', '\it\psi')

msplot(222, tt, avp.vn, 'Vel /m.s^{-1}');
legend('\itv\rm_E', '\itv\rm_N', 'itv\rm_U')

msplot(223, tt, deltapos(avp.pos), '\DeltaPos /m');
legend('\Delta\itL', '\Delta\it\lambda', '\Delta\ith')

% 横轴是经度，纵轴是纬度
msplot(224, avp.pos(:, 2)/arcdeg, avp.pos(:, 1)/arcdeg, ...
    '\it\lambda\rm /(\circ)', '\itL\rm /(\circ)');    hold on
plot(avp.pos(1,2)/arcdeg, avp.pos(1, 1)/arcdeg, 'ro');

% imu输出信息作图
msplot(121, tt, imu.tb/ts/arcdeg, ...
    '\it\omega^b_{ib}\rm /(circ.s^{-1})');
legend('\it\omega^b_{ibx}', '\it\omega^b_{iby}', '\it\omega^b_{ibz}');

msplot(122, tt, imu.vb/ts, '\itf^b\rm_{sf}/(m.s^{-2})');
legend('\itf^b\rm_{sf\itx}', '\itf^b\rm_{sf\ity}', '\itf^b\rm_{sf\itz}');

%% 保存数据
dataInfo.name = data_name;
dataInfo.description = 'Long time random trajectory';
dataInfo.date = datestr(now);
dataInfo.type = 'simulation data';
dataInfo.ts = ts;
dataInfo.length = length(avp.pos);
dataInfo.imuerr = 'Only reference data';

cd(data_folder);
save(data_name, 'avp', 'imu', 'dataInfo');
cd('D:\inertial_navigation_basic');

clearvars -except avp imu dataInfo