function [ gyrError, eb, wg, er ] = generateGyrError(length, ts, imuError)
%% **************************************************************
%名称：Generate Gyroscope error
%功能：生成一组陀螺误差
%原理@ 惯性仪器测试与数据分析
%________________________________________________________________________
% 输入：
% 	length: 数据长度
% 	ts: imu输出频率
%	imuError: imu误差，当然我们只会使用其中的陀螺误差部分，请注意将输入参数的
%   单位调整为与以下一致。
%       Gbias: 随机常值漂移 （deg/h）
%       GN ：随机游走系数 (deg/sqrt(h))    
%       GN2: 白噪声功率谱密度 (deg/h)^2/Hz 
%       GfB: 白噪声带宽 (HZ) 
%       Gtau: 一阶马尔可夫过程的相关时间 (s)
%       GR0: 一阶马尔可夫过程方差 (deg/h)^2
%       *Gwg: 白噪声标准差 (deg/h)，考虑到有时我们希望直接指定白噪声标准差，故
%             特地添加了这个参数，若指定了Gwg，则会忽略GN;GN2,GfB的影响。
% 输出：
%       gyrError: 陀螺随机误差
%       eb = 常值漂移
%       wg = 白噪声
%       er = 一阶马尔可夫过程，不存在imuError.Gtau 时输出为0
%_________________________________________________________________________
%作者：哈尔滨工程大学 智能科学与工程学院 张峥
%日期：2020年12月3日
% ************************************************************************
%%
% @ P155,陀螺随机漂移分为 1.随机常值漂移，2.白噪声，3.一阶马尔可夫过程。前两种
% 噪声肯必定会添加，第三种不一定。因为陀螺分为角速率输出型和角增量输出型两种，
% 因此分开考虑。角增量输出型陀螺使用随机游走系数GN；角速率输出型陀螺需已知功率
% 谱密度GN2和带宽GfB
global Gl
setGl_unit;
% 陀螺常值零偏
if isfield(imuError, 'Gbias')
    Gbias = imuError.Gbias * Gl.dph; %（deg/h）
else
    Gbias = 0;
    disp('注意，缺少陀螺零偏参数，已默认为0！');
end
% 判断陀螺输出类型
if isfield(imuError, 'GN')
    % 角增量输出型
    condition = 'increment';
    N = imuError.GN*Gl.dpsh; % (deg/sqrt(h))
else
    if isfield(imuError, 'GN2') && isfield(imuError, 'GfB')
        % 角速度输出型
        condition = 'velocity';
        N2 = imuError.GN2*Gl.dph^2/Gl.Hz; %(deg/h)^2/Hz
        fB = imuError.GfB*Gl.Hz; %(HZ)
    else
        condition = 'noWhiteNoise';
        disp('注意，缺少陀螺白噪声参数，已默认不添加白噪声！');
    end
end

% 判断是否添加一阶马尔可夫过程噪声
if isfield(imuError, 'Gtau') && isfield(imuError, 'GR0')
    ismarkov = 1;
    tau = imuError.Gtau*1; % s
    R0 = imuError.GR0*Gl.dph^2; % (deg/h)^2
else
    disp('陀螺噪声不含一阶马尔可夫过程');
    ismarkov = 0;
end

% 预分配内存
er = zeros(length, 1);

switch condition
    % *** 增量输出型 ***
    case 'increment' % deg
        disp('陀螺误差为角增量型');
        % 随机常值漂移
        eb = Gbias*ts*ones(length, 1);
        % 白噪声
        std_wg = N*sqrt(ts); % 随机游走系数为N，输出频率为ts的情况下，白噪声的标准差
        wg = std_wg*randn(length, 1);
        % 一阶马尔可夫噪声
        if ismarkov == 1
            beta = 1/tau;
            Phi = 1 - beta*ts;
            qr = 2*beta*R0;
            std_wr = sqrt(qr*ts);
            for k = 2:length
                er(k) = Phi*er(k-1) + std_wr*randn(1, 1);
            end
            er = er*ts;
        else
            er = zeros(length, 1);
        end
        
    % *** 速度输出型 ***    
    case 'velocity' % deg/h
        disp('陀螺误差为角速度型');
        % 随机常值漂移
        eb = Gbias*ones(length, 1);
        % 白噪声 参照@P245。一定要注意：P157中的Qk失准角的角度激励噪声方差
        % 单位为deg，而角速率输出型陀螺的误差单位为(deg/h) 
        std_wg = sqrt(N2*fB);
        wg = std_wg*randn(length, 1);
        % 一阶马尔可夫噪声
        if ismarkov == 1
            beta = 1/tau;
            Phi = 1 - beta*ts;
            qr = 2*beta*R0;
            std_wr = sqrt(qr*ts);
            for k = 2:length
                er(k) = Phi*er(k-1) + std_wr*randn(1, 1);
            end   
        else
            er = zeros(length, 1);
        end
    otherwise
        %未指定白噪声的情况下，默认为速度输出型
        disp('因未指定陀螺白噪声，已默认陀螺输出为角速度型！');
        eb = Gbias*ones(length, 1);
        wg = zeros(length, 1);
        % 一阶马尔可夫噪声
        if ismarkov == 1
            beta = 1/tau;
            Phi = 1 - beta*ts;
            qr = 2*beta*R0;
            std_wr = sqrt(qr*ts);
            for k = 2:length
                er(k) = Phi*er(k-1) + std_wr*randn(1, 1);
            end   
        else
            er = zeros(length, 1);
        end
end

gyrError = eb + er + wg;
end
