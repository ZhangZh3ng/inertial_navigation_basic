function [ att_normalized ] = attnorml( att, condition )
%% **************************************************************
%名称：Attitude Normalization (version 1.0)
%功能：姿态角标准化 (目前仅修正航向角误差)
%________________________________________________________________________
% 输入：
%       att: 未标准化的姿态角误差
% 输出：
%       att_normalized: 标准化过后的姿态角误差
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月14日
% ************************************************************************
%%
threshold = pi;

% 如果输入数据单位是deg
if exist('condition', 'var') && strcmp(condition, 'deg')
    threshold = 180;
end


if length(att) ==3
    % 输入姿态有三个分量，认为它是普通的姿态角
    % 分别提取三个姿态角误差
    thresholdtch_err = att(1);
    roll_err = att(2);
    yaw_err = att(3);
    
    % thresholdtch 角度范围为: [-threshold/2, threshold/2]
    % roll 角度范围为: (-threshold, threshold]
    % yaw 角度范围为: [0, 2*threshold)
    
    % 直接计算出的航向角误差∈(-2*threshold, 2*threshold),但我们希望航向角
    % 误差∈(-threshold, threshold]。如果航向角误差绝对值大于threshold，
    % 就令其标准化
    if norm(yaw_err) > threshold
        % 如果航向角误差为正
        if sign(yaw_err) == 1
            yaw_err = yaw_err - 2*threshold;
        else
            % 航向角误差大于threshold，且符号为负
            yaw_err = yaw_err + 2*threshold;
        end
    end    
    att_normalized = [thresholdtch_err, roll_err, yaw_err]';
    
else
    % 认为输入变量是一串姿态数据
    k = 0;
    while k < length(att)
        % 进入循环，编号+1
        k = k+1;
        
        if norm(att(k)) > threshold
            % 如果航向角误差为正
            if sign(att(k)) == 1
                att(k) = att(k) - 2*threshold;
            else
                % 航向角误差大于threshold，且符号为负
                att(k) = att(k) + 2*threshold;
            end
        end
       
    end
    att_normalized = att;
end
end
