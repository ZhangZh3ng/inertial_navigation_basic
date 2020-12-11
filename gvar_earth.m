%% **************************************************************
%名称：Global variable of earth
% @ 捷联惯导算法与组合导航原理 P231
%________________________________________________________________________
% 全局变量
%_________________________________________________________________________
%作者：哈尔滨工程大学 自动化学院 张峥
%日期：2020年10月4日
% ************************************************************************
%%
global GM Re ff wie ge gp g0 ug arcdeg arcmin arcsec hour ms ...
    dph dpsh ugpsHz lsc deg min sec
%% WGS-84 model
GM = 3.986004415e14;
Re = 6.378136998405e6;
wie = 7.2921151467e-5;

% 椭球扁率 ff = (Re - Rp)/Re
ff = 1/298.257223563;
ee = sqrt(2*ff - ff^2);
e2 = ee^2;
Rp = (1 - ff)*Re;

ge = 9.780325333434361;
gp = 9.832184935381024;
g0 = ge; 
ug = g0*1e-6;
%% 角度
% 角度、角分和角秒,把deg转化为相应的rad:
% 1°= pi/180 rad, 1′= pi/180/60 rad, 1″= pi/180/60/60 rad
arcdeg = pi/180;
arcmin = arcdeg/60;
arcsec = arcmin/60;
% 把弧度转化成角度、角分和角秒
deg = 1/arcdeg;
min = 1/arcmin;
sec = 1/arcsec;
%% 时间和速率
hour = 3600;
ms = 1e-3;
dph = arcdeg/hour;
dpsh = arcdeg/sqrt(hour);
ugpsHz = ug/sqrt(1);
%% lsc: line shape and color
% 注意! lsc中的字符串长度必须相同，空格也计入长度
lsc = [' -k'; ' -b'; ' -r'; '-.m'; '--g'; ' :c'];
