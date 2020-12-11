function [ imuError ] = selectImuError( imuType, outputType )
% ************************************************************************
% Prototype: 
% Function: Set up the value of imu error.
%
% Input：
%   imuType: A string that indicating the type of IMU device.
%   outputType: The type of device output, the value must be increment or
%   velocity.
%
% Output：
%   imuError: Using capital letter 'G' or 'A' to present gyroscope and 
%   accelerometer resepctively.
%       .bias: Device constant bias.
%       .tau: Correlation time of Markov process.
%       .R0: Correlation variance of Markov process.
%       .N2: Power spectral density of excited white noise.
%       .fB: Equivalent bandwidth.
%       .N: Random walk coefficient.
%
% NOTE: We assumpt error consists the following three parts:
% 1. CONSTANT BIAS: characterlized by 'bias'
% 2. FIRST ORDER MARKOV PORCESS: characterlized by 'tau' and 'R0'.
% 3. WHITE NOISE: characterlized by 'N' or {'N2', 'fB'} in which the former 
% is for INCREMENT OUTPUT device while the latter is for VELOCITY OUTPUT 
% device. 
% We will always model the constant biase and white noise in practice.
% For detail: @惯性仪器测试与数据分析 P151-152,P155-157,P245
%
% Version: 1.0
% Created on 2020/12/5
%
% Copyright(c) 2020，Zhang Zheng 
% College of Intelligent Systems Science and Engineering, Harbin
% Engineering University, Harbin, China.
% ************************************************************************
%%
% If user does not specify a imuType, the function will automatically
% assign value 'default' to imuType, and excutes OTHERWISE.
if ~exist('imuType', 'var')
    imuType = 'default';
end

% If user doesn't specify outputType, the function will automatically
% excute INCREMENT case.
if ~exist('outputType', 'var')
    outputType = 'increment';
end

switch imuType
    case 'selfdefine'
        % user define
        imuError.Gbias = 100*randn(1, 1); % deg/h
        imuError.Gtau = 50; % s
        imuError.GR0 = 0; % (deg/h)^2  
        imuError.Abias = 500*randn(1, 1); % ug
        imuError.Atau = 50; % s
        imuError.AR0 = 0; % ug^2
        switch outputType
            case 'velocity'
                imuError.GN2 = 0.0001; % (deg/h)^2/Hz
                imuError.GfB = 400; % Hz
                imuError.AN2 = 0.01; % (ug)^2/Hz
                imuError.AfB = 400; % Hz
            otherwise
                imuError.GN = 10; % deg/sqrt(Hz)
                imuError.AN = 100; % ug/sqrt(HZ)
        end
    % ---------------------------------------------------------------------
    case 'zero'
        % no imu error
        imuError.Gbias = 0*randn(1, 1); % deg/h
        imuError.Gtau = 50; % s
        imuError.GR0 = 0; % (deg/h)^2       
        imuError.Abias = 0*randn(1, 1); % ug
        imuError.Atau = 50; % s
        imuError.AR0 = 0; % ug^2
        switch outputType
            case 'velocity'
                imuError.GN2 = 0; % (deg/h)^2/Hz
                imuError.GfB = 400; % Hz
                imuError.AN2 = 0; % (ug)^2/Hz
                imuError.AfB = 400; % Hz
            otherwise
                imuError.GN = 0; % deg/sqrt(Hz)
                imuError.AN = 0; % ug/sqrt(HZ)
        end
    % ---------------------------------------------------------------------
    case 'mid'
        imuError.Gbias = 0.1*randn(1, 1); % deg/h
        % tau: Correlation time of Markov process
        imuError.Gtau = 50; % s
        % R0: Correlation variance of Markov process
        imuError.GR0 = 0.01; % (deg/h)^2        
        imuError.Abias = 50*randn(1, 1); % ug
        imuError.Atau = 50; % s
        imuError.AR0 = 100; % ug^2 
        switch outputType
            case 'velocity'
                % N2: Power spectral density of excited white noise
                imuError.GN2 = 0.0001; % (deg/h)^2/Hz
                % fB: Equivalent bandwidth
                imuError.GfB = 400; % Hz
                imuError.AN2 = 0.01; % (ug)^2/Hz
                imuError.AfB = 400; % Hz
            otherwise
                % N: random walk coefficient
                imuError.GN = 0.01; % deg/sqrt(Hz)
                imuError.AN = 10; % ug/sqrt(HZ)
        end
    % ---------------------------------------------------------------------
    case 'low'
        % low 
        imuError.Gbias = 100*randn(1, 1); % deg/h
        imuError.Gtau = 50; % s
        imuError.GR0 = 10; % (deg/h)^2  
        imuError.Abias = 500*randn(1, 1); % ug
        imuError.Atau = 50; % s
        imuError.AR0 = 10000; % ug^2
        switch outputType
            case 'velocity'
                imuError.GN2 = 0.0001; % (deg/h)^2/Hz
                imuError.GfB = 400; % Hz
                imuError.AN2 = 0.01; % (ug)^2/Hz
                imuError.AfB = 400; % Hz
            otherwise
                imuError.GN = 10; % deg/sqrt(Hz)
                imuError.AN = 1000; % ug/sqrt(HZ)
        end
    % ---------------------------------------------------------------------
    otherwise
        % defualt condition
        imuError.Gbias = 0.1*randn(1, 1); % deg/h
        imuError.Gtau = 50; % s
        imuError.GR0 = 0.0001; % (deg/h)^2  
        imuError.Abias = 50*randn(1, 1); % ug
        imuError.Atau = 50; % s
        imuError.AR0 = 100; % ug^2
        switch outputType
            case 'velocity'
                imuError.GN2 = 0.0001; % (deg/h)^2/Hz
                imuError.GfB = 400; % Hz
                imuError.AN2 = 100; % (ug)^2/Hz
                imuError.AfB = 400; % Hz
            otherwise
                imuError.GN = 0.01; % deg/sqrt(Hz)
                imuError.AN = 10; % ug/sqrt(HZ)
        end
end

end