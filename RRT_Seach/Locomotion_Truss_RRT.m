function [path,StartTree,iter,dist_data] = Locomotion_Truss_RRT(initial_truss, final_com_pos, fixed_node, maxIter, stepsize)
%TRUSS_RRT Summary of this function goes here
%   Detailed explanation goes here
tic

M_3D = [ones(1,length(initial_truss.pos))  zeros(1,length(initial_truss.pos))  zeros(1,length(initial_truss.pos))
     zeros(1,length(initial_truss.pos)) ones(1,length(initial_truss.pos))  zeros(1,length(initial_truss.pos))
     zeros(1,length(initial_truss.pos)) zeros(1,length(initial_truss.pos))  ones(1,length(initial_truss.pos))]/length(initial_truss.pos);
 
M = [ones(1,length(initial_truss.pos))  zeros(1,length(initial_truss.pos))  zeros(1,length(initial_truss.pos))
     zeros(1,length(initial_truss.pos)) ones(1,length(initial_truss.pos))  zeros(1,length(initial_truss.pos))
     zeros(1,length(initial_truss.pos)) zeros(1,length(initial_truss.pos)) zeros(1,length(initial_truss.pos))]/length(initial_truss.pos);
 
dim = 3*length(initial_truss.pos'); % The number of nodes position coordinates
fixed_index = get_fixed_index(fixed_node); % Index of fixed nodes position coordinates

pos_of_fixed_index = reshape(initial_truss.pos', [dim, 1]);
pos_of_fixed_index = pos_of_fixed_index(fixed_index');

initial_config = reshape(initial_truss.pos', [dim, 1]);
% config = [x1 y1 z1 x2 y2 z2 ...].   cf) x = [x1 x2 x3 y1 y2 y3 ...]

% final_config = reshape(final_truss.pos, [dim, 1]);

temp_com = (M_3D * reshape(initial_truss.pos,[numel(initial_truss.pos),1]))'; % 1x3

RRT_structure.Start_Node = whole_pos_2_planning_pos(initial_config, fixed_index);
RRT_structure.Goal_ComPos = final_com_pos;
RRT_structure.Dim = dim - length(fixed_index);
RRT_structure.stepsize = stepsize;
origin_config = zeros(RRT_structure.Dim,1);
 
for i = 1:RRT_structure.Dim/3
    origin_config(3*(i-1)+1:3*i) = temp_com';
end

RRT_structure.Config_Limit = [-3*ones(RRT_structure.Dim, 1) + origin_config, 3*ones(RRT_structure.Dim, 1) + origin_config];
% RRT_structure.Config_Limit = [-1E4*ones(RRT_structure.Dim, 1), 1E4*ones(RRT_structure.Dim, 1)];
% RRT_structure.mixingFactor = mixing_factor; % 0 for all random

[path,StartTree,iter,dist_data] = Locomotion_retraction_RRT(RRT_structure, initial_truss, maxIter, fixed_index, pos_of_fixed_index, M);
% 
% path = temp_path.final_path;


toc