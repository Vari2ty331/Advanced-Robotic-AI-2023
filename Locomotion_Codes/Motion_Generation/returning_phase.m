returning = 1;

%% Moving to x_fin configuration

returning_index = 0;

while returning == 1
    D = [];
    feasible = false;
    x = data.x(index-1,:)';
    data.fixed_member{index,1} = [];
    iter = 0;
    
    R = R_cal(x,n);
    C = C_cal(fixed_node,n);
    
    while feasible == false
        constraint_flag = 0;
        [dx_temp,~,~,constraint_flag] = desired_motion_returning(D,n,R,C,M,x,dt,rotate_node,front_node,back_node,pos_foot,data,x_fin);
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
    pos_new_temp = reshape(x_new,size(n.pos));    
    x_new = reshape(pos_new_temp,[numel(n.pos) 1]);
    
    if distance_from_x_fin(x_new,zeros(size(x_new)),x_fin) <= 1e-2  % Terminating criterion. If distance between x_new and x_fin is larger than 1e-2, terminate.
        returning = 2;
    end
    
    if stop_flag == 1
        break;
    end
    
    data_write;    
    fprintf(['path_index: %d  index: %d  iter: %d  D size: %d  returning    ',datestr(now),'\n'],path_index,index,iter,size(D,1))
    index = index + 1;
    returning_index = returning_index + 1;    
    
    if returning_index > max_rolling_index
        stop_flag = 1;
        fprintf('Planning failed: returning_index exceeded maximum\n')
        break;
    end

end