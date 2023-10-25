function [ fixed_index ] = get_fixed_index( fixed_nodes )
%GET_FIXED_INDEX Summary of this function goes here
%   Detailed explanation goes here
fixed_index = [];

for i = 1:numel(fixed_nodes)
    fixed_index = [fixed_index, 3*fixed_nodes(i)-2:3*fixed_nodes(i)];
end

end

