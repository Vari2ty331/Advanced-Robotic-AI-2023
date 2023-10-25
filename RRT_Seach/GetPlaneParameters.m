function [plane_param] = GetPlaneParameters(pos1, pos2, pos3)
%GETPLANEPARAMETERS Summary of this function goes here
%   Detailed explanation goes here
vec1 = pos1-pos2;
vec2 = pos2-pos3;
n = cross(vec1, vec2);
d = -dot(n, pos1);
plane_param = [n; d]';
end

