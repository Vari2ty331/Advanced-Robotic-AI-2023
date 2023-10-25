function [c, ceq] = truss_force_cal_cons(X,surface_normal_vector_aligned,elist_length,fixed_node,MaxStaticCoeff)

% surface_normal_vector_aligned: Aligned w.r.t. fixed node number

ceq = [];

R_ori = X(6*elist_length+1:end);
Force_Margin = 1e-2;    % Margin to prevent float error
Normal_Force = zeros(length(fixed_node),3);
Req_Fric = zeros(length(fixed_node),3);
c = zeros(length(fixed_node));

for f_num = 1 : length(fixed_node)
    Normal_Force(f_num,:) = (dot(R_ori((3*f_num-2):(3*f_num)), surface_normal_vector_aligned(f_num,:)))* surface_normal_vector_aligned(f_num,:);
    Req_Fric(f_num,:) = R_ori((3*f_num-2):(3*f_num)).' - Normal_Force(f_num,:);
    c(f_num) = norm(Req_Fric(f_num,:)) - MaxStaticCoeff * norm(Normal_Force(f_num,:)) + Force_Margin;
end
% for f_num = 1 : length(fixed_node)
%     c(f_num) = - dot(R_ori((3*f_num-2):(3*f_num)), surface_normal_vector_aligned(f_num,:)) + norm(R_ori((3*f_num-2):(3*f_num))) * cos(atan(MaxStaticCoeff)) + Force_Margin;
% end