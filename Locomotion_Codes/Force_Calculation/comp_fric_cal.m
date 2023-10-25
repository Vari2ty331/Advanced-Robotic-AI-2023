function fric_cons = comp_fric_cal(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff)

% Calculate the difference between the max static force and required
% friction force

[~, ~, Req_Fric, Normal_Force] = truss_force_cal(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff);

Req_Fric = reshape(Req_Fric, 3, length(Req_Fric)/3);
Normal_Force = reshape(Normal_Force, 3, length(Normal_Force)/3);

for k = 1 : length(fixed_node)
    fric_cons(k) = MaxStaticCoeff * norm(Normal_Force(:,k)) - norm(Req_Fric(:,k));
end

end