%% **************************************************************
%名称：euler angel matrix
%功能：计算欧拉角对应的过渡矩阵的形式解
%________________________________________________________________________
%       Xb - east - pitch : theta 
%       Yb - north - roll : phi
%       Zb - up - yaw : psi
%   n(定系)经过三次连续的旋转得到b(动系)
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年9月9日
% ************************************************************************
%%
% 姿态角：定义载体坐标系b与理想导航坐标系n之间的角度关系如下：
% 航向角psi: 载体纵轴by在水平面内投影与北向轴nN的夹角
% 纵摇角theta: 载体纵轴by与水平面的夹角
% 横摇角phi: 载体横轴bx与水平面的夹角
% 分别是 sin(x) sin(y) sin(z) cos(x) cos(y) cos(z)
syms sx sy sz cx cy cz real;

% 坐标系转换矩阵
Rx = [1,  0,   0
      0, cx, -sx
      0, sx,  cx];

Ry = [cy, 0, sy
       0, 1,  0
     -sy, 0, cy];

Rz = [cz, -sz, 0
      sz,  cz, 0
       0,   0, 1];

% *** 设置转动顺序 ***
condition = '321';

% euler angle 
switch condition
    case '123'
        Cbn = Rx*Ry*Rz; 
        Aw = (Ry*Rz)'*diag([1, 0, 0]) + Rz'*diag([0, 1, 0]) + diag([0, 0, 1]);
    case '132'
        Cbn = Rx*Rz*Ry;
        Aw = (Rz*Ry)'*diag([1, 0, 0]) + Ry'*diag([0, 0, 1]) + diag([0, 1, 0]);
    case '213'
        Cbn = Ry*Rx*Rz;
        Aw = (Rx*Rz)'*diag([0, 1, 0]) + Rz'*diag([1, 0, 0]) + diag([0, 0, 1]);
    case '231'
        Cbn = Ry*Rz*Rx;
        Aw = (Rz*Rx)'*diag([0, 1, 0]) + Rx'*diag([0, 0, 1]) + diag([1, 0, 0]);
    case '312'
        Cbn = Rz*Rx*Ry;
        Aw = (Rx*Ry)'*diag([0, 0, 1]) + Ry'*diag([1, 0, 0]) + diag([0, 1, 0]);
    case '321'
        Cbn = Rz*Ry*Rx;
        Aw = (Ry*Rx)'*diag([0, 0, 1]) + Rx'*diag([0, 1, 0]) + diag([1, 0, 0]);
    otherwise
        disp('请输入合法的转动顺序 ');
end

% 坐标转换矩阵
Cbn
% Cnb = Cbn'
Aw