% refernce: Linear Actuator Robots: Differential Kinematics, Controllability, and Algorithms for Locomotion and Shape Morphing
 
clc
clear
close all

disp('Initialize...')

param.L_nom = 1.25;

n = octa12_link_connection;
n.pos = n.pos * param.L_nom;

stop_flag = 0;
max_iter = 300;
max_rolling_index = 300;

Ground_Point = [    -2, -2, 0;
                        -2, 2,  0;
                        2,  -2, 0;
                        2,  2,  0; ];

Ground = Ground_Generation(Ground_Point); 

[n, data] = data_initialization(n, param, Ground);


%% Foot planning

% Calculate and draw motion primitive positions
foot_MotionPrimitive

% Step2: Trajectory planning of center of mass
N_step = 100; % The number of steps for motion primitive

dx_cm_desire = [];
foot_desire = [];
for i = 1:length(Final_Path_T)-1
    com1 = sum(Final_Path_T(i).n) / 3;
    com2 = sum(Final_Path_T(i+1).n) / 3;
    vec = (com2 - com1)/N_step;
    dx_cm_desire = [dx_cm_desire ; vec(1)*ones(N_step,1) vec(2)*ones(N_step,1) vec(3)*ones(N_step,1)];
    foot_desire = [foot_desire ; Final_Path_T(i+1).n(3,:)];
end



data.foot_desire = foot_desire;
data.dx_cm_desire = dx_cm_desire;

%% Motion Primitive generation
tic

path_index = 1;
index = 2;
rolling = 0;
foot_index = 1;
Terminate = 0;

surface_normal_vector = data.normal_vector{1};
f_count = zeros(length(n.elist),1);
non_front_node = [];
M = [ones(1,length(n.pos))  zeros(1,length(n.pos))  zeros(1,length(n.pos))
     zeros(1,length(n.pos)) ones(1,length(n.pos))  zeros(1,length(n.pos))
     zeros(1,length(n.pos)) zeros(1,length(n.pos))  zeros(1,length(n.pos))]/length(n.pos);
 
dt = data.dt;

while Terminate == 0
    
    x = data.x(index-1,:)';
    n.pos = reshape(x,size(n.pos));
    fixed_node = data.fixed_node{index-1};
    
    %% Stability check
    sup_polygon = fixed_node( convhull(n.pos(fixed_node,1),n.pos(fixed_node,2)) );
    stability_check; % calculate stability and find rotate node, front node   
   
    if stability == 0
     %% Rolling phase and Returning phase 
        rolling_phase 
        returning_phase
        Terminate = 1;      % Terminate the main iteration.
        if stop_flag == 1
            break;
        end
    else % center of mass control (stability = 1)   
    %% Control phase
        control_phase;
        path_index = path_index + 1;
    end % stability if end
      
    if stop_flag == 1
        break;
    end
end % end while
toc

if stop_flag == 0
    disp('Motion Primitive generation succeed. Save the data')
%     data_MotionPrimitive = data;
%     clearvars -except data_MotionPrimitive
%     save(['Motion_Primitive_Data.mat'])
end