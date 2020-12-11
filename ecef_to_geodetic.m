function [geoPos] = ecef_to_geodetic(ecefPos)
% ************************************************************************
% Geodetic coordinates transports to ECEF coordinates
% Profile: This function converses a geodetic coordinates to a ECEF 
% coordinates.  
%
% Input
%   ecefPos: [x, y, z](m), the ECEF coordinates. 
%
% Output
%   geoPos: [latitude(rad), longitude(rad), altitude(m)]', the geodetic
%   coordinates.
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
% ECEF coordinates
x = ecefPos(1); y = ecefPos(2); z = ecefPos(3);

% Geodetic coordinates
geoPos = zeros(3, 1);
% Longitude
geoPos(2) = atan2(ecefPos(2), ecefPos(1));
% Denote t = tan(Latitude), we caculate t by iteration firstly, then
% determine the latitude by arctangent t.
t0 = 0;
Lp_iteration = 0;
while Lp_iteration < 8
    Lp_iteration = Lp_iteration+1;
    t = 1/sqrt(x^2 + y^2) * (z + Re*e2*t0/sqrt(1 + (1-e2)*t0^2));
    t0 = t;
end
geoPos(1) = atan(t);
% Principal radii of curvature along the prime-vertical normal section
RN = Re/sqrt(1 - e2*sin(geoPos(1))^2);
% Altitude
geoPos(3) = sqrt(x^2 + y^2)/cos(geoPos(1)) - RN;

end