function [ecefPos] = geodetic_to_ecef(geoPos)
% ************************************************************************
% Geodetic coordinates transports to ECEF coordinates
% Profile: This function converses a geodetic coordinates to a ECEF 
% coordinates.  
%
% Input
%   geoPos: [latitude(rad), longitude(rad), altitude(m)]', the geodetic
%   coordinates.
%
% Output
%   ecefPos: [x, y, z](m), the ECEF coordinates. 
%
% For detail: @捷联惯导算法与组合导航原理 P50
%
% Version: developing
% Created on 2020/12/8
%
% Copyright(c) 2020，Zhang Zheng 
% College of Intelligent Systems Science and Engineering, Harbin
% Engineering University, Harbin, China.
% ************************************************************************
%%
% Equatorial radius of earth
Re = 6.378136998405e6; % m
% Square eccentricity of earth
e2 = 0.006694379990141;
% Principal radii of curvature along the prime-vertical normal section
RN = Re/sqrt(1 - e2*sin(geoPos(1))^2);
% ECEF coordinate
ecefPos = zeros(3, 1);
ecefPos(1) = (RN + geoPos(3)) * cos(geoPos(1)) * cos(geoPos(2));
ecefPos(2) = (RN + geoPos(3)) * cos(geoPos(1)) * sin(geoPos(2));
ecefPos(3) = (RN*(1 - e2) + geoPos(3)) * sin(geoPos(1));

end