function [ imu_msr, gyrError, accError ] = addImuError(imu, imuError, ts)
% ************************************************************************
% Add IMU Error 
% Function: Add noise to a piece of IMU measurement data
% 
% Input：
%   imu:
%   imuError:
%   ts:
%
% Output：
%   imu_msr:
%   gyrError:
%   accError:
%
% Version: 1.0
% Created on 2020/12/5 
%
% Copyright(c) 2020，Zhang Zheng 
% College of Intelligent Systems Science and Engineering, Harbin
% Engineering University, Harbin, China.
% ************************************************************************
%%
% data length
[length, imuNum] = size(imu.tb);

% pre-allocating memory
gyrError = zeros(length, imuNum);
accError = zeros(length, imuNum);

% generate error
for k = 1:1:imuNum
    gyrError(:, k) = generateGyrError(length, ts, imuError);
    accError(:, k) = generateAccError(length, ts, imuError);
end

% At present, we mostly consider the scenario where the measurement
% information is increment, so we default imu measurement is 'tb' and 'vb'.
imu_msr.tb = imu.tb + gyrError;
imu_msr.vb = imu.vb + accError;
end