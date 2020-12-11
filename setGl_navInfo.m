function setGl_navInfo()
% ************************************************************************
% Set Global Variable - navigation information
% Function: set some Global navigation information
%
% Version: v1.0   
% Created on 2020/12/4 and last updated on 2020/12/4.
%
% Copyright(c) 2020，Zhang Zheng 
% College of Intelligent Systems Science and Engineering, Harbin
% Engineering University, Harbin, China.
% ************************************************************************
%%
global Gl
Gl.Call_navInfo = true;

% deg2rad
deg = pi/180;

% @Harbin 
Gl.pos = [34*deg, 108*deg, 100]';

end