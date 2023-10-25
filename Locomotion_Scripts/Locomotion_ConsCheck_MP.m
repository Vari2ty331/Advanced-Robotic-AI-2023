%% Contraint verification

constraint_flag = 0;        % Flag used for alert if constraint violation occurs
ConsViolated_index = [];    % Set of trajectory indexes that violates constraints


for trajectory_index = Locomotion_index_StepStart : length(data.x)
    constraint_flag_index = 0;          % Flag used for indicating if constraints were violated on current trajectory index 
    
    % Matching the index used on various codes
    path_index = trajectory_index;
    index = trajectory_index;
    
    % Variable initializing
    fixed_node = [];
    surface_normal_vector = [];
    x = data.x(trajectory_index,:)';
    n.pos = reshape(x,size(n.pos));

    %% Fixed node determination
    % find fixed node by calculating distance between ground and every node. 
    % If distance is smaller than Collision_Tol, the node is on the ground.
    
    Collision_Tol = 1e-3;
    for node_index = 1 : length(n.pos)
        [~, Ground_Distance] = Ground_Collision(Ground, n.pos(node_index, :));          % Calculating distance between the ground meshes and the node.
        if ~isempty(find(isnan(Ground_Distance),1))
            Ground_index_min = find(isnan(Ground_Distance),1);
            Ground_Distance_min = 0;
        else
            [Ground_Distance_min, Ground_index_min] = min(Ground_Distance);                             % Find the min distance and corresponding index of the ground mesh.
        end
        if (Ground_Distance_min <= Collision_Tol)
            fixed_node = [fixed_node, node_index];
            surface_normal_vector = [surface_normal_vector; NVector_Find(node_index, Ground, Ground_index_min);];
        end
    end
    data.fixed_node{trajectory_index} = fixed_node;
    data.normal_vector{trajectory_index} = surface_normal_vector;
    
    %% Stability check
    sup_polygon = fixed_node( convhull(n.pos(fixed_node,1),n.pos(fixed_node,2)) );
    stability_check; % calculate stability and find rotate node, front node   
    
    %% Constraint check
    constraint = constraint_gen(x,n,data,fixed_node,surface_normal_vector,data.MaxStaticCoeff);

    % Find the contraint value that less than zero (violation)
    
    data.Grad_Cons{trajectory_index,1} = [];
    
    for i = 1:length(constraint)
        if constraint(i) < 0
            constraint_flag = 1;
            constraint_flag_index = 1;
%             data.Grad_Cons{trajectory_index,1} = [data.Grad_Cons{trajectory_index,1};dfdx(i,:)];
        end
    end 
    
    if constraint_flag_index == 1
        ConsViolated_index = [ConsViolated_index, trajectory_index];
    end
    
    rolling = 0;

    
    %% Data write
    x_new = x;
    data_write
end

if constraint_flag ~= 0
    fprintf('Constraint violation occurred in step %d\n',polygon_index)
else
    fprintf('No constraint violation occurred in step %d\n',polygon_index)
end
