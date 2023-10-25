% refernce: Linear Actuator Robots: Diff erential Kinematics, Controllability, and Algorithms for Locomotion and Shape Morphing
%
clc
clear
% close all

n_initial = octa12_link_connection;
L_nom = 1.25;

[n, param, Ground] = foot_initialize(n_initial, L_nom);

[n, data] = data_initialization(n, param, Ground);

data = 0;

walls = wall_create_not_mesh(6,Ground);

% plot_ground(Ground,param,walls);

rng shuffle
s = rng;
% rng(541312981); %wtf why plots faster?

%% Initial position setting

x_ini = reshape(n.pos,[numel(n.pos),1]);
L_ini = strut_length_cal(n);

%% Load motion primitive

load('Motion_Primitive_Data.mat')

param.mixing_factor = 0;
%% Foot planning multiple times

graph_plot =1;
p = gcp('nocreate');
% if isempty(p)
% parpool(8)
% end
% parfor itr=1:8
for itr = 1:1
    close all
% [Final_Path_T,StartTree] = foot_planning(n, param, Ground, data, data_MotionPrimitive);
% [Final_Path_T,StartTree] = foot_planning_temp_rrt(n, param, Ground, data, data_MotionPrimitive,walls);
[Final_Path_T,StartTree,pathInfo] = foot_planning_polygon_rrt(n, param, Ground, data, data_MotionPrimitive,walls);
% [Final_Path_T,StartTree] = foot_planning_temp(n, param, Ground, data, data_MotionPrimitive,walls);
% 
% No_FinalNode_Start = length(Final_Path_T);

% if length(Final_Path_T) < 8
% 
%     plot_polygonplanning
% 
%     
% 
% end
% 
% path_tree_samples(itr) = length(Final_Path_T);
saveData(itr).Final_Path_T = Final_Path_T;
saveData(itr).StartTree = StartTree;
saveData(itr).pathInfo = pathInfo;
disp('itr finished')
disp(itr)
end

% delete(gcp('nocreate'))

%%
if graph_plot == 1
    plot_polygonplanning_poly
end