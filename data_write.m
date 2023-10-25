n.pos = reshape(x_new,size(n.pos));

%% Angle calculation
angle = angle_cal(n);
link_angle = link_angle_cal(n);
dihedral_angle = dihedral_angle_cal(n.link_connection,n.elist,n.pos);

%% Truss force calculation
[P_c, P_t, Reaction, Req_Fric, Normal_Force] = calc_comp_tens_force(n.elist,n.pos,fixed_node,surface_normal_vector,data.MaxStaticCoeff);
[L, mass_sphere, mass_member, com_member] = com_member_cal(n.elist,n.pos); % Calculate L, mass and com of members and sphere

%% Data write
data.x(index,:) = x_new;
data.dx(index,:) = ( data.x(index,:) - data.x(index-1,:) );
data.L(index,:) = lengths(n.elist,n.pos);
data.L_min(index,:) = min(data.L(index,:));
data.L_max(index,:) = max(data.L(index,:));
data.dL(index,:) = ( data.L(index,:) - data.L(index-1,:) );
data.vel_max(index,1) = max(abs(data.dL(index,:)));
data.x_cm(index,:) = cm_cal(n.elist,data.x(index,:)');
data.constraint{index,1} = constraint';
data.constraint_flag(index,1) = constraint_flag;
data.stability(index,1) = stability;
data.rolling(index,1) = rolling;
data.angle_min(index,1) = min(angle);
data.angle_max(index,1) = max(link_angle);
data.dihedral_angle_min(index,1) = min(dihedral_angle);
data.d_min(index,1) = d_min_cal(constraint,data,n);
data.path_index(index,1) = path_index;
data.compressive_force(index,:) = P_c;
data.tensile_force(index,:) = P_t;
data.Reaction{index,1} = Reaction;
data.Reaction_Req_Fric{index,1} = Req_Fric;
data.Reaction_Normal_Force{index,1} = Normal_Force;
data.stb_margin = min(dist);


data.fixed_node{index,1} = fixed_node;
data.front_node{index,1} = front_node;
data.rotate_node(index,:) = rotate_node;
data.normal_vector{index,1} = surface_normal_vector;
