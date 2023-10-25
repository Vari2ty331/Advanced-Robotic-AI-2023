function [path, plot_path, LocalTree, IATT_concat] = multiple_truss_RRT(initial_truss, final_truss, fixed_nodes, maxIter, mixing_factor, smooth_path_length)
%TRUSS_RRT Summary of this function goes here
%   Detailed explanation goes here

global fixed_index pos_of_fixed_index
dim = 3*length(initial_truss.pos);
fixed_index = get_fixed_index(fixed_nodes);

pos_of_fixed_index = reshape([initial_truss.pos], [dim, 1]);
pos_of_fixed_index = pos_of_fixed_index(fixed_index');

initial_config = reshape(initial_truss.pos, [dim, 1]);
final_config = reshape(final_truss.pos, [dim, 1]);

RRT_structure.Start_Node = whole_pos_2_planning_pos(initial_config, fixed_index);
RRT_structure.Goal_Node = whole_pos_2_planning_pos(final_config, fixed_index);
RRT_structure.Dim = dim - length(fixed_index);
RRT_structure.stepsize = 0.02;
RRT_structure.Config_Limit = [-5*ones(RRT_structure.Dim, 1), 5*ones(RRT_structure.Dim, 1)];
% RRT_structure.Config_Limit = [-1E4*ones(RRT_structure.Dim, 1), 1E4*ones(RRT_structure.Dim, 1)];
RRT_structure.mixingFactor = mixing_factor; % 0 for all random
[path, LocalTree, IATT_concat] = MultipleRRT(RRT_structure, initial_truss, maxIter);
% startNode_in_constraints = startNode_in_constraints';
% goalNode_in_constraints = goalNode_in_constraints';
path = path.smooth_path;

numpath = size(path, 2);
plot_path = zeros(size(path,1), smooth_path_length);
px1 = linspace(0, 1, numpath);
px2 = linspace(0, 1, smooth_path_length);
for i=1:size(path, 1)
    pp = polyfit(px1, path(i,:), 2);
    plot_path(i,:) = polyval(pp, px2);
end



end

