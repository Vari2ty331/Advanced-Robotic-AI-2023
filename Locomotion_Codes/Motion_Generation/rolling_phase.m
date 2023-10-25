
rolling = 1;

%% Rolling

pos_foot = data.foot_desire(foot_index,:);
rolling_index = 0;

while rolling == 1
    Ground_Contact_Flag = 0;    % Indicate contacting between front nodes and the ground
    D = [];
    feasible = false;
    x = data.x(index-1,:)';
    data.fixed_member{index,1} = [];
    iter = 0;
    
    R = R_cal(x,n);
    C = C_cal(fixed_node,n);
    
    while feasible == false
        constraint_flag = 0;
        [dx_temp,~,~,constraint_flag] = desired_motion_rolling(D,n,R,C,M,x,dt,rotate_node,front_node,back_node,pos_foot,data);
                                                          
        x_new = x + dx_temp;
        n.pos = reshape(x_new,size(n.pos));
        active = 0;

        [constraint,dfdx,P] = constraint_gen(x_new,n,data,fixed_node,surface_normal_vector,data.MaxStaticCoeff);
        [f_count,data,fix_flag] = f_count_cal(data,n,P,f_count);

        for i = 1:length(constraint)
            if constraint(i) < 0
                D = [D;dfdx(i,:)];
                active = active + 1;
%                 disp('aa')
            end
        end

        if active == 0 && fix_flag == 0
            feasible = true;
        end
        
        iter = iter + 1;
        
        % stop flag
        if iter > max_iter
            stop_flag = 1;
            fprintf('Planning failed: the iteration exceeded maximum number\n')
            break;
        end    
    end

    pos_new_temp = reshape(x_new,size(n.pos));
    for front_node_index = 1 : length(front_node)
        % Find a distance between a front node and the ground.
        [~, Ground_Distance_Temp] = Ground_Collision(Ground, pos_new_temp(front_node(front_node_index),:),1);
        % Calculate the distance, find the closest ground index
        % If collision happens, find that instead.
        if any(isnan(Ground_Distance_Temp))
            Ground_index_min = find(isnan(Ground_Distance_Temp),1);
            Ground_Distance(front_node_index) = 0;
        else
            [Ground_Distance(front_node_index), Ground_index_min] = min(Ground_Distance_Temp);
        end
    end
    if (sum(Ground_Distance(front_node_index)) < 0.005) || any(isnan(Ground_Distance_Temp))
        % Set Ground Flag as 1 if node is too close to the ground (1st condition) or collide with the ground (2nd condition)
        Ground_Contact_Flag = 1;
    end
    if Ground_Contact_Flag == 1 % condition when a rotating node contact to the ground
        rolling = 2;
        ground_node = [];
        for front_node_index = 1 : length(front_node)
            if (Ground_Distance(front_node_index) < 0.005) || isnan(Ground_Distance(front_node_index))
                ground_node = [ground_node front_node(front_node_index)];
                new_normal_vector = NVector_Find(front_node(front_node_index), Ground, Ground_index_min);
                surface_normal_vector = [surface_normal_vector; new_normal_vector;];
            end
        end
        fixed_node = [fixed_node ground_node];
        transient_node = [rotate_node ground_node]; % target supporting polygon in transient phase
        surface_normal_vector = NVector_Sort(surface_normal_vector, fixed_node);    % Sorting normal vector w.r.t. fixed node
        
%         pos_new_temp(fixed_node,3) = 0; % for preventing ground contact constraint error
    end

    x_new = reshape(pos_new_temp,[numel(n.pos) 1]);
    
    if stop_flag == 1
        break;
    end
    
    data_write;    
    fprintf(['path_index: %d  index: %d  iter: %d  D size: %d  rolling    ',datestr(now),'\n'],path_index,index,iter,size(D,1))
    index = index + 1;
    rolling_index = rolling_index + 1;    
    
    if rolling_index > max_rolling_index
        stop_flag = 1;
        fprintf('Planning failed: rolling_index exceeded maximum\n')
        break;
    end
end % while end

foot_index = foot_index + 1;

%% Transient phase

while rolling == 2
    
    % stability check at target supporting polygon
    sup_polygon = transient_node( convhull(n.pos(transient_node,1),n.pos(transient_node,2)) );
    stability_check

    if stability == 1
        disp('stable transient')
        rolling = 0;
        fixed_node = transient_node;
        surface_normal_vector = NVector_Sort(surface_normal_vector, fixed_node);    % Remove the surface normal vector that not contacting to the ground.
        break;
    end
    
    % Control phase part
    x = data.x(index-1,:)';
    n.pos = reshape(x,size(n.pos));
    
    R = R_cal(x,n);
    C = C_cal(fixed_node,n);
    D = [];
    data.fixed_member{index,1} = [];
    
    feasible = false;
    iter = 0;
           
    while feasible == false
        dx_cm = data.dx_cm_desire(path_index,:)';
        constraint_flag = 0;
        [dx_temp,~,~,constraint_flag] = desired_motion(D,R,C,dx_cm,data);
        x_new = x + dx_temp;    

        n.pos = reshape(x_new,size(n.pos));
        active = 0;

        [constraint,dfdx,P] = constraint_gen(x_new,n,data,fixed_node,surface_normal_vector,data.MaxStaticCoeff);
        [f_count,data,fix_flag] = f_count_cal(data,n,P,f_count);

        for i = 1:length(constraint)
            if constraint(i) < 0
                D = [D;dfdx(i,:)];
                active = active + 1;
            end
        end

        if active == 0 && fix_flag == 0
            feasible = true;
        end

        iter = iter + 1;    
        
        % stop flag
        if iter > max_iter
            stop_flag = 1;
            fprintf('Planning failed: the iteration exceeded maximum number\n')
            break;
        end    
    end
    
    % stability check at current support polygon
    sup_polygon = fixed_node( convhull(n.pos(fixed_node,1),n.pos(fixed_node,2)) );
    stability_check

    if stop_flag == 1
        break;
    end
    
    data_write;   
    fprintf(['path_index: %d  index: %d  iter: %d  D size: %d  transient   ',datestr(now),'\n'],path_index,index,iter,size(D,1))
    index = index + 1;    
    
    path_index = path_index + 1;
  
end
