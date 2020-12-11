function [ imuerr ] = imuerrorset( condition )
%% **************************************************************
%名称：imu error
%功能：设置imu误差
%________________________________________________________________________
% 输入：
%       condition: 器件类型
% 输出：
%       imuerr: imu器件参数组成的struct,包含：
%            eb: 陀螺零偏
%            web: 角度随机游走
%            db: 加计零偏
%            wdb: 速度随机游走
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月15日
% ************************************************************************
%%
gvar_earth;

% 初始化，默认imuerr.case = 'zero'
imuerr.case = 'zero';
imuerr.eb = [0, 0, 0]'*dph;
imuerr.web = [0, 0, 0]'*dpsh;
imuerr.db = [0, 0, 0]'*ug;
imuerr.wdb = [0, 0, 0]'*ugpsHz;

% 根据用户设置的condition选择对应的imuerr配置方案
if exist('condition', 'var') 
    switch condition
        % *** selfdefine case ***
        case 'selfdefine'
            imuerr.case = 'selfdefine';
            imuerr.eb = [0.1, 0.1, 0.1]'*dph;
            imuerr.web = [0.1, 0.1, 0.1]'*dpsh;
            imuerr.db = [100, 100, 100]'*ug;
            imuerr.wdb = [1, 1, 1]'*ugpsHz;
            
        % *** specific case ***
        case '201029_1' 
            imuerr.case = '201029_1';
            imuerr.eb = [0.1, 0.1, 0.1]'*dph;
            imuerr.web = [0.1, 0.1, 0.1]'*dpsh;
            imuerr.db = [100, 100, 100]'*ug;
            imuerr.wdb = [1, 1, 1]'*ugpsHz;
            
        % *** big error
        case 'big'
            imuerr.case = 'big';
            imuerr.eb = [10, 10, 10]'*dph;
            imuerr.web = [1, 1, 1]'*dpsh;
            imuerr.db = [1000, 1000, 1000]'*ug;
            imuerr.wdb = [10, 10, 10]'*ugpsHz;
            
        % ** zero ***
        case 'zero'
            % do nothing
        
        % Yan @捷联惯导算法与组合导航原理 P239 组合导航程序中的器件误差
        otherwise
            imuerr.case = 'Yan';
            imuerr.eb = [0.01, 0.015, 0.02]'*dph;
            imuerr.web = [0.001, 0.001, 0.001]'*dpsh;
            imuerr.db = [80, 90, 100]'*ug;
            imuerr.wdb = [1, 1, 1]'*ugpsHz;
    end
 
end

end
