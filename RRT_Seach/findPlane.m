function [ parameters ] = findPlane(node1, node2, node3)
%FINDPLANE Summary of this function goes here
%   Detailed explanation goes here

normal = cross(node1-node2, node2-node3);
d = -dot(normal, node1);

parameters = [normal;d];

end

