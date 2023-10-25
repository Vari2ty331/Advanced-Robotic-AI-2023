%% Revise transformed motion primitive if contraint flag is not zero

Transform_Ref_index = [];           % Reference point that moved out of constraint region. (The point that most severly violates the constraints)
Violation_Size_Prev = 0;            % Previous size of violation. (norm of negative constraint)

Revising_Coeff = 1e-3;              % Coefficient for how much move 

% Find the position that violates the constraints most
for k = ConsViolated_index
    Constraint_Temp = data.constraint{k};
    Constraint_Temp(find(data.constraint{k}>=0)) = 0;           % Make positive constraints (not violated) to zero
    if norm(Constraint_Temp) > Violation_Size_Prev
        Transform_Ref_index = k;                                % Set Transform_Ref_index as the one has largest constraint violation size
        Violation_Size_Prev = norm(Constraint_Temp);
    end
end

% Move the reference point out of constraint region.
x_Pre_ref = data.x(Transform_Ref_index,:);                          % Save previous reference point for linear transformation
x_Temp_pos = data.x(Transform_Ref_index,:);
constraint_flag_revise = 1;                                         % Inicator for if revised reference point get out of the constrain region (0 is not violating cons)       
Revision_Number = 1;                                                % Number of revision

[~, Transform_Ref_GradCon] = constraint_gen(data.x(Transform_Ref_index,:)',n,data,data.fixed_node{Transform_Ref_index},data.normal_vector{Transform_Ref_index},data.MaxStaticCoeff);   % Calculating gradient of constraints

RevisionMatrix = eye(length(x_Pre_ref));        % Matrix that prevents moving of fixed node position
for k = data.fixed_node{Transform_Ref_index}
    RevisionMatrix(k, k) = 0;
    RevisionMatrix(6 + k, 6 + k) = 0;
    RevisionMatrix(12 + k, 12 + k) = 0;
end

while constraint_flag_revise == 1
    [~, Transform_Ref_GradCon] = constraint_gen(x_Temp_pos',n,data,data.fixed_node{Transform_Ref_index},data.normal_vector{Transform_Ref_index},data.MaxStaticCoeff);
    Grad_Cons_Num = size(Transform_Ref_GradCon);      % Find number of constraint gradients
    Grad_Cons_Num = Grad_Cons_Num(1);
    for k = 1 : Grad_Cons_Num
        x_Temp_pos = x_Temp_pos + Revising_Coeff * Transform_Ref_GradCon(k,:) * RevisionMatrix;     % Move the point outside of constraint violation region without moving fixed nodes.
    end
    fprintf('Index %d was revised %d times\n',Transform_Ref_index, Revision_Number)
    constraint = constraint_gen(x_Temp_pos,n,data,data.fixed_node{Transform_Ref_index},data.normal_vector{Transform_Ref_index},data.MaxStaticCoeff);
    constraint_flag_revise = 0;
    
    for i = 1:length(constraint)
        if constraint(i) < 0
            constraint_flag_revise = 1;
        end
    end
    Revision_Number = Revision_Number + 1;
end

% data.x(Transform_Ref_index,:) = x_Temp_pos;

%% Linear transform w.r.t. reference point

% Finding linear transformation from x_ini to x_ref
x_Start_ini = data.x(Locomotion_index_StepStart,:);                         % Change Motion primitive start/end pose to x form
% x_End_ini = data.x(Transform_Ref_index,:);
Lin_Transform_A_Prev = MPLinearTransform(x_Start_ini, x_Pre_ref, x_Start_ini, x_Temp_pos, x_length);


for trajectory_index = Locomotion_index_StepStart : Transform_Ref_index
    data.x(trajectory_index, :) = (data.x(trajectory_index, :) - x_Start_ini) * Lin_Transform_A_Prev.' + x_Start_ini;         % order swaped becuase data.x is row vector
end

% Finding linear transformation from x_ref to x_fin
% x_Start_ini = data.x(Transform_Ref_index,:);                                % Change Motion primitive start/end pose to x form
x_End_ini = data.x(length(data.x),:);
Lin_Transform_A_Aft = MPLinearTransform(x_Pre_ref, x_End_ini, x_Temp_pos, x_End_ini, x_length);


for trajectory_index = Transform_Ref_index + 1 : length(data.x)
    data.x(trajectory_index, :) = (data.x(trajectory_index, :) - x_Pre_ref) * Lin_Transform_A_Aft.' + x_Temp_pos;         % order swaped becuase data.x is row vector
end
