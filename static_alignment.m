%% **************************************************************
% static alignment
% 静态对准
%_________________________________________________________________________
%作者：哈尔滨工程大学 智能科学与工程学院 张峥
%日期：2020年11月2日
% ************************************************************************
%%
close all
clear all

% 全局变量
gvar_earth;

% imu采样频率
ts_imu = 5*ms;
% 每次姿态更新使用的子样数
num_of_subsample = 2;
% 姿态更新步长
nts = num_of_subsample*ts_imu;

% *** 读入imu及转台数据 ***
% 第1列：  惯导上电时间，每5ms递增1；
% 第2列：  惯导状态，0：待机， 1~2 对准，3导航；
% 第3-5列：XYZ陀螺角速率，单位rad/s，坐标系为顺序为“前上右”
% 第6-8列：XYZ加表比力，单位m/s2，坐标系为顺序为“前上右”
% 第9-15列：XYZ陀螺温度，XYZ加表温度，电路温度，单位℃
% 第16-17列：转台内环位置，转台外环位置，单位°
data_folder = 'D:\data\N_201102\数据1_\';
data_dir_info = dir(data_folder);

~isempty(strfind(data_dir_info(3).name, 'imu'));
data_name = '083815_testb_imuBoard';
data_path = strcat(data_folder, data_name, '.txt');
DT = load(data_path);

% 分离数据中的成分
% 注意！原始数据中imu数据顺序为“前上右”，这里改为了习惯使用的“右前上”
imu_msr.wb = [DT(:, 5), DT(:, 3:4)];
imu_msr.fb = [DT(:, 8), DT(:, 6:7)];
% 比力和角速度转化为速度增量和速度增量
imu_msr.tb = 0.5*ts_imu * (imu_msr.wb(1:end-1, :) + imu_msr.wb(2:end, :));
imu_msr.vb = 0.5*ts_imu * (imu_msr.fb(1:end-1, :) + imu_msr.fb(2:end, :));

for k = 1:length(imu_msr.fb)
    norm_of_msr_gn_stg(k) = norm(imu_msr.fb(k, :), 2);
end

if size(DT, 2) == 17   
    platform_ref = DT(:, 16:17);
end

% 数据总长度
data_time = length(DT)*ts_imu;

% 实验地位置信息
pos_ref = [39.8122*arcdeg, 116.1515*arcdeg, 44]';
pos = pos_ref;
vn_ref = [0, 0, 0]';
vn = vn_ref;

att_ref = [-0.0034, 0.0040, -1.5694]';
Cbn_ref = a2mat(att_ref);
qbn_ref = a2qua(att_ref);
% 基本导航解算
eth = earth(pos, vn);

% 导航和滤波初值
qbn = [1, 0, 0, 0]';
% kf.Qk = diag();
% kf.Rk = diag();

% 初值
prev_alpha = [0, 0, 0]';
prev_beta = [0, 0, 0]';
prev_K = zeros(4, 4);
prev_Cbtb0 = eye(3);
prev_Cntn0 = eye(3);

% 为动态变量预分配内存
calc_att_stg = zeros(length(DT) + 10, 3);
calc_init_att_stg = zeros(length(DT) + 10, 3);
calc_pos_stg = zeros(length(DT) + 10, 2);
calc_vn_stg = zeros(length(DT) + 10, 3);
%% 惯导更新和对准
% 循环次数计数变量
Lp_msr = 1;
Lp_update = 0;
crt = 0;
while crt < 305   
    Lp_msr = Lp_msr+1;
    k = Lp_msr; 
    
    % 每量测量测进行一次更新
    if mod(Lp_msr, 2) == 1 && Lp_msr ~= 1
        Lp_update = Lp_update+1;
        
        % 获取量测信息
        wm = imu_msr.tb(k-num_of_subsample:k-1, :);
        vm = imu_msr.vb(k-num_of_subsample:k-1, :);
        %补偿圆锥运动和划桨运动
        [phim, dvbm] = cnscl(wm, vm);
        
        % 更新Cbtb0和Cntn0
        Cbtb0 = prev_Cbtb0*rv2m(phim);
        Cntn0 = prev_Cntn0*rv2m(eth.winn*nts);       
        
        % *** 惯导更新 ***
        vn1 = vn + rv2m(-eth.winn*nts/2)*qmulv(qbn_ref, dvbm) + eth.gcc*nts;
        % 更新位置时采用[tm-1, tm]这段时间内的平均速度
        vn = (vn + vn1)/2;
        
        % 位置更新
        pos = pos + [vn(2)/eth.RMh, vn(1)/eth.clRNh, vn(3)]'*nts;
        % tm-1 时刻的速度
        vn = vn1;

        % 粗对准+精对准
        if round(Lp_msr*ts_imu) <= 100000
            % *** 粗对准 ***
            % alpha and beta
            alpha = prev_alpha - prev_Cntn0*nts*eth.gn;
            beta = prev_beta + prev_Cbtb0*dvbm;
            % QUEST法求qbn0
            [ qbn0, prev_K ] = QUEST( beta, alpha, prev_K );
            % 下次更新初值
            prev_alpha = alpha;
            prev_beta = beta;
            
            % OBA法解算姿态
            Cbn0 = q2mat(qbn0); 
            Cbn = Cntn0'*Cbn0*Cbtb0;            
        else
            % *** 精对准 ***
            [kf.Phikk_1, kf.Gammak, kf.Hk] = ...
                        model_static_fine_aligment(eth, Cbn, 12, nts);            
            kf = kfupdate( kf, vn ); 
        end
      
        % 保存计算结果
        calc_init_att_stg(Lp_update, :) = q2att(qbn0)';
        calc_att_stg(Lp_update, :) = m2att(Cbn)';
        calc_pos_stg(Lp_update, :) = pos(1:2)'*deg;
        calc_vn_stg(Lp_update, :) = vn';
        
        % 下次初值
        prev_Cbtb0 = Cbtb0;
        prev_Cntn0 = Cntn0;    
    end   
    
    if myequal(crt, 300)
        result_stg = m2att(Cbn)';
    end 
    
    progressTip(crt, ts_imu);
    % 下一个获得imu量测信息的时刻
    crt = crt + ts_imu;
end
calc_att_stg(Lp_update+1:end, :) = [];
calc_init_att_stg(Lp_update+1:end, :) = [];
calc_pos_stg(Lp_update+1:end, :) = [];
calc_vn_stg(Lp_update+1:end, :) = [];
%% 绘图
timeAxis_msr = (0:1:length(DT)-1)*ts_imu;
timeAxis_update = (0:1:Lp_update-1)*nts;

% 计算姿态(Cbn)
msplot(311, timeAxis_update, calc_att_stg(:, 1)*deg, '时间 /s', 'pitch /deg');
title(strcat(data_name, '   esitmated attitude curve'), 'Interpreter', 'none');
msplot(312, timeAxis_update, calc_att_stg(:, 2)*deg, '时间 /s', 'roll /deg');
msplot(313, timeAxis_update, calc_att_stg(:, 3)*deg, '时间 /s', 'yaw /deg');

% 计算姿态init
msplot(311, timeAxis_update, calc_init_att_stg(:, 1)*deg, '时间 /s', 'pitch /deg');
title(strcat(data_name,'    esitmated initial attitude '), 'Interpreter', 'none');
msplot(312, timeAxis_update, calc_init_att_stg(:, 2)*deg, '时间 /s', 'roll /deg');
msplot(313, timeAxis_update, calc_init_att_stg(:, 3)*deg, '时间 /s', 'yaw /deg');

msplot(111, timeAxis_update, calc_init_att_stg(:, 3)*deg, '时间 /s', 'yaw /deg');
% % 转台姿态
% if size(DT, 2) == 17
% msplot(211, timeAxis_msr, attnorml(DT(:, 16), 'deg'), '时间/s', '内环位置 /deg');
% title('转台姿态');
% msplot(212, timeAxis_msr, attnorml(DT(:, 17), 'deg'), '时间/s', '外环位置 /deg');
% end
% 
% % 计算位置及姿态
% msplot(211, timeAxis_update, calc_pos_stg(:, 1), '时间 /s ', '纬度 /deg');
% title('惯导解算所得位置');
% msplot(212, timeAxis_update, calc_pos_stg(:, 2), '时间 /s ', '经度 /deg');
% 
msplot(311, timeAxis_update, calc_vn_stg(:, 1), '时间 /s', 've /m/s');
title('纯惯导解算速度');
msplot(312, timeAxis_update, calc_vn_stg(:, 2), '时间 /s', 'vn /m/s');
msplot(313, timeAxis_update, calc_vn_stg(:, 3), '时间 /s', 'vu /m/s');

% imu输出
figure
subplot(321)
plot(timeAxis_msr, imu_msr.fb(:, 1));xlabel('时间 /s');ylabel('fbx / m/s^2');
title('加计输出')
subplot(323)
plot(timeAxis_msr, imu_msr.fb(:, 2));xlabel('时间 /s');ylabel('fby / m/s^2');
subplot(325)
plot(timeAxis_msr, imu_msr.fb(:, 3));xlabel('时间 /s');ylabel('fbz / m/s^2');
subplot(322)
plot(timeAxis_msr, imu_msr.wb(:, 1));xlabel('时间 /s');ylabel('wx / rad/s');
title('陀螺输出');
subplot(324)
plot(timeAxis_msr, imu_msr.wb(:, 2));xlabel('时间 /s');ylabel('wy / rad/s');
subplot(326)
plot(timeAxis_msr, imu_msr.wb(:, 3));xlabel('时间 /s');ylabel('wz / rad/s');

%%
% 理论航向精度
gyro_accuracy = 1e-3;   % deg/h
theoretical_azimuth_accuracy = gyro_accuracy/(15*cos(pos(1)))*deg;
%% 保存结果
% cd('D:\result\N_201102\数据2_粗对准加精对准');
% saveas(1, strcat(data_name, '   esitmated attitude curve'), 'fig');
% saveas(2, strcat(data_name, '   esitmated initial attitude'), 'fig');
% cd('D:\inertial_navigation_basic');
