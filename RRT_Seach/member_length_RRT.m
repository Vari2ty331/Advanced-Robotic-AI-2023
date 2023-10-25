function [path,StartTree, GoalTree] = member_length_RRT(initial_truss, final_truss, fixed_member, maxIter, mixing_factor)
%TRUSS_RRT Summary of this function goes here
%   Detailed explanation goes here

global fixed_index length_of_fixed_index;
dim = size(initial_truss.elist,1);
fixed_index = fixed_member;


max_length = 2.5;

initial_config = initial_truss.mem_length;
final_config = final_truss.mem_length;

length_of_fixed_index = reshape([initial_truss.mem_length], [dim, 1]);
length_of_fixed_index = length_of_fixed_index(fixed_index');

RRT_structure.Start_Node = whole_pos_2_planning_pos(initial_config, fixed_index);
RRT_structure.Goal_Node = whole_pos_2_planning_pos(final_config, fixed_index);
RRT_structure.Dim = dim - length(fixed_index);
RRT_structure.stepsize = 0.02;
RRT_structure.Config_Limit = [zeros(RRT_structure.Dim,1), max_length*ones(RRT_structure.Dim,1)];
RRT_structure.mixingFactor = mixing_factor; % 0 for all random

[path,StartTree, GoalTree] = member_length_BiRRT(RRT_structure, initial_truss, maxIter);

path = path.final_path;




end

