function [ att ] = m2att( Cbn )
%% **************************************************************
%名称：direction cosine Matrix to ATTitude
%功能：姿态矩阵转换成姿态角
%________________________________________________________________________
% 输入：
%       Cbn: 从b系到n系的坐标转换矩阵
% 输出：
%       att: 有三个分量，依次为俯仰theta、横滚gamma和航向psi。 航向角北偏
%       西为正,取值为（-pi, pi]
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年8月14日
% ************************************************************************
%% 
% 原理@ 捷联惯导算法与组合导航原理 P245
% 在theta ≈ ±pi/2的时候，gamma和psi是无法分离的，仅知道：
%   gamma + psi = atan2(Cbn(1, 3), Cbn(1, 1));  (theta ≈ +pi/2)
%   gamma - psi = atan2(Cbn(1, 3), Cbn(1, 1));  (theta ≈ -pi/2)
% gamma和psi存在多值性，只有指定了其中一个的值才能确定另一个，这里令psi = 0

% 注意: 这种情况相当于俯仰角等于±90°正常情况下地面上的载体不可能有这
% 种运动轨迹。尽管这种运动不可能在物理上存在，但可能在进行导航解算时在计算
% 中产生。

if abs(Cbn(3,2)) <= 0.999999
    att = [asin(Cbn(3, 2)), -atan2(Cbn(3, 1), Cbn(3, 3)), -atan2(Cbn(1, 2), Cbn(2, 2))]';
else
    att = [asin(Cbn(3, 2)), atan2(Cbn(1, 3), Cbn(1, 1)), 0]';
end

end