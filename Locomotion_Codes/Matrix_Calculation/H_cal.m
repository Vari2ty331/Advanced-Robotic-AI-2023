function H = H_cal(x,n,fixed_node)

C = C_cal(fixed_node,n);
R = R_cal(x,n);
H = [R;C];