function [ accError, eb, wg, er ] = generateAccError(length, ts, imuError)
%% **************************************************************
%名称：Generate Accelerometer error
%功能：生成一组加计误差
%本程序为类比生成陀螺误差那个程序写的，也认为加计有两种输出模式：加速度输出型和
%速度增量输出型。
%________________________________________________________________________
% 输入：
% 	length: 数据长度
% 	ts: imu输出频率
%	imuError: imu误差，当然我们只会使用其中的加计误差部分，请注意将输入参数的
%   单位调整为与以下一致。
%       Abias: 随机常值漂移 （ug）
%       AN ：随机游走系数 (deg/sqrt(h)) (ug)*s/sqrt(h)
%       AN2: 白噪声功率谱密度 (deg/h)^2/Hz  (ug^2)/Hz
%       AfB: 白噪声带宽 (HZ) (Hz)
%       Atau: 一阶马尔可夫过程的相关时间 (s) (s)
%       AR0: 一阶马尔可夫过程方差 (deg/h)^2 (ug^2)
% 输出：
%       gyrError: 陀螺随机误差
%       eb = 加计常值零偏
%       wg = 白噪声
%       er = 一阶马尔可夫过程，不存在imuError.Atau 时输出为0
%_________________________________________________________________________
%作者：哈尔滨工程大学 智能科学与工程学院 张峥
%日期：2020年12月3日
% ************************************************************************
%%
global Gl
setGl_unit;
if isfield(imuError, 'Abias')
    Abias = imuError.Abias * Gl.ug; %（ug）
else
    Abias = 0;
    disp('注意，缺少加计零偏参数，已默认为0！');
end
% 判断加计输出类型
if isfield(imuError, 'AN')
    % 速度增量输出型
    condition = 'increment';    
    N = imuError.AN*Gl.ug/sqrt(Gl.Hz); % (ug/sqrt(Hz))
else
    if isfield(imuError, 'AN2') && isfield(imuError, 'AfB')
        % 加速度输出型
        condition = 'velocity';        
        N2 = imuError.AN2*Gl.ug^2/Gl.Hz; %(ug)^2/Hz
        fB = imuError.AfB*Gl.Hz; %(HZ)
    else
        condition = 'noWhiteNoise';
        disp('注意，缺少加计白噪声参数，已默认不添加白噪声！');
    end
end

% 判断是否添加一阶马尔可夫过程噪声
if isfield(imuError, 'Atau') && isfield(imuError, 'AR0')
    existMarkov = 1;
    tau = imuError.Atau*1; % s
    R0 = imuError.AR0*Gl.ug^2; % (ug)^2
else
    existMarkov = 0;
    disp('加计噪声不含一阶马尔可夫过程 ');
end

% 预分配内存
er = zeros(length, 1);

switch condition
    % *** 增量输出型 ***
    case 'increment' % deg
        disp('加计噪声为速度增量型');
        % 随机常值漂移
        eb = Abias*ts*ones(length, 1);
        % 白噪声
        std_wg = N*sqrt(ts); % 随机游走系数为N，输出频率为ts的情况下，白噪声的标准差
        wg = std_wg*randn(length, 1);
        % 一阶马尔可夫噪声
        if existMarkov == 1
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
        disp('加计噪声为加速度型');
        % 随机常值漂移
        eb = Abias*ones(length, 1);
        % 白噪声 参照@P245。一定要注意：P157中的Qk失准角的角度激励噪声方差
        % 单位为deg，而角速率输出型陀螺的误差单位为(deg/h) 
        std_wg = sqrt(N2*fB);
        wg = std_wg*randn(length, 1);
        % 一阶马尔可夫噪声
        if existMarkov == 1
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
        disp('因未指定白噪声，已默认加计输出为加速度型！');
        eb = Abias*ones(length, 1);
        wg = zeros(length, 1);
        if existMarkov == 1
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

accError = eb + er + wg;

end