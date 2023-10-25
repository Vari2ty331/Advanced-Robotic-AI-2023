function data = Locomotion_Planning_MP(Final_Path_T, n, Ground, data, data_MotionPrimitive)

%% Transform and connect the motion primitive

Locomotion_index = 2;                   % Index for locomotion
L_nom = data.L_nom;

for polygon_index = 1 : length(Final_Path_T) - 1
    Locomotion_index_StepStart = Locomotion_index;          % Save the index that starts the k-th step
    
    [Pose_Start_ini, Pose_End_ini, MotionPrimitive_Transform, Rotation_Angle, Polygon_Center_1] = MPPose(Final_Path_T(polygon_index), Final_Path_T(polygon_index+1), n, data_MotionPrimitive);
    
    % Finding linear transformation component for current step
    x_length = length(data_MotionPrimitive.x(1,:));
    OptX_Start = Final_Path_T(polygon_index).OptPose;
    OptX_End = Final_Path_T(polygon_index + 1).OptPose;
    
    Lin_Transform_A = MPLinearTransform(Pose_Start_ini, Pose_End_ini, OptX_Start, OptX_End, x_length);

    x_Start_ini = reshape(Pose_Start_ini, [1, numel(Pose_Start_ini)]);          % Change Motion primitive start/end pose to x form
    x_End_ini = reshape(Pose_End_ini, [1, numel(Pose_End_ini)]);

    
%% Transforming motion primitive for each time step
  
    for trajectory_index = 1 : length(data_MotionPrimitive.x)        
        % Transforming the motion primitive
        data_temp_pos = reshape(data_MotionPrimitive.x(trajectory_index,:),size(n.pos));
        data_temp_pos = MotionPrimitive_Transform * data_temp_pos;                    % Matching ground node index
        for k = 1 : length(data_temp_pos)
            data_temp_pos(k,:) = (Rotating_Line((data_temp_pos(k,:)).', [0,0,0], [0,0,1], Rotation_Angle)).';
            data_temp_pos(k,:) = data_temp_pos(k,:) + Polygon_Center_1;
        end

        data_onestep.x(trajectory_index, :) = reshape(data_temp_pos,[1, numel(data_temp_pos)]);
        data_onestep.x(trajectory_index, :) = (data_onestep.x(trajectory_index, :) - x_Start_ini) * Lin_Transform_A.' + reshape(OptX_Start, [1, numel(OptX_Start)]);         % order swaped becuase data.x is row vector
        data.x(Locomotion_index, :) = data_onestep.x(trajectory_index, :);
        Locomotion_index = Locomotion_index + 1;
    end

    
%% Check constraints and fix transformed motion primitive
        
    Locomotion_ConsCheck_MP
    
    while constraint_flag == 1
        Locomotion_Revision_MP
        Locomotion_ConsCheck_MP
    end
    
end

