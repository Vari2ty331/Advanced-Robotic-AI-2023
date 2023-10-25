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

data_write; % update x and com_member
fprintf(['path_index: %d  index: %d  iter: %d  D size: %d   ',datestr(now),'\n'],path_index,index,iter,size(D,1))
index = index + 1;  