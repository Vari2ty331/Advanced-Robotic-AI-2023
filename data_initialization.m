function [n, data] = data_initialization(n, param, Ground)
    comp_strength = 150; % initial: 80
    tens_strength = 100; % initial: 75
    stb_margin_val = 0.1; % initial: 0.05
    vel_max_val = 0.005;

    comp_strength_diff = 0; % initial: 80
    tens_strength_diff = 0; % initial: 75
    stb_margin_val_diff = 0; % initial: 0.05
    vel_max_val_diff = -0.002;

    %% Mechanical Constraints (octa)

    non_front_node = [];
    fixed_node = [1 2 3];       % Initial Ground nodes
    n.elist

    L_nom = param.L_nom;
%     n.pos = n.pos*L_nom;

    % Actual length constraint (by Alex Dec.3): 1100 mm - 2450 mm
    stroke = 2; % original 1.25
    L_min_long = 1.11; % original 1.15
    L_max_long = L_min_long + stroke;          
    L_min_short = 1.11; % original 1.15
    L_max_short = L_min_short + stroke;

    L_min = L_min_long*ones(1,12);
    L_max = L_max_long*ones(1,12);

    angle_min = (25 + 0) /180*pi; % unit: rad
    angle_max = (155 - 0) /180*pi; % unit: rad

    dihedral_angle_min = (45 + 0) / 180*pi;

    dt = 5;
    vel_max = vel_max_val*dt; % for step_num 1008
    acc_max = 2*dt;
    % vel_max = 0.015/2; % for step_num 200

    manip_min = 0.2; % previous: 0.20
    stb_margin = stb_margin_val; 

    [L, mass_sphere, mass_member, com_member] = com_member_cal(n.elist,n.pos); % Calculate L, mass and com of members and 
    
    % variable for fixing the member length
    f_count = zeros(length(n.elist),1);
    data.max_f_count = 1;
    data.fixed_member{1} = [];
    
    %% Find initial and final position

    % Initial Position setting

    d_min = 0.1;    % set 0 if you not want to check collision
    MaxStaticCoeff = 0.5;

    % Form of surface_normal_vector
    % [ v_n, node_number_n; ...]

    surface_normal_vector = [];

    for normal_index = 1 : length(fixed_node)
        for ground_index = 1 : length(Ground)
            temp_pos = n.pos(fixed_node(normal_index),:);
            if inpolygon(n.pos(fixed_node(normal_index),1), n.pos(fixed_node(normal_index),2), Ground(ground_index).Vertices(1:3,1), Ground(ground_index).Vertices(1:3,2)) == 1
                surface_normal_vector = [surface_normal_vector; NVector_Find(fixed_node(normal_index), Ground, ground_index)];
                break;
            end
        end
    end                


    x_ini = reshape(n.pos,[numel(n.pos),1]);
    L_ini = strut_length_cal(n);

    % Center of mass matrix
    M = [ones(1,length(n.pos))  zeros(1,length(n.pos))  zeros(1,length(n.pos))
         zeros(1,length(n.pos)) ones(1,length(n.pos))  zeros(1,length(n.pos))
         zeros(1,length(n.pos)) zeros(1,length(n.pos))  zeros(1,length(n.pos))]/length(n.pos);

    % Variables for initial data write
    angle = angle_cal(n);
    link_angle = link_angle_cal(n);
    dihedral_angle = dihedral_angle_cal(n.link_connection,n.elist,n.pos);



    % initial truss force calculation
%     [P_c, P_t, Reaction, Req_Fric, Normal_Force] = calc_comp_tens_force(n.elist,n.pos,fixed_node,surface_normal_vector,MaxStaticCoeff);

    %% Data write (initial configurtaion t = 0, index = 1)
    index = 1;

    % constraint data
    data.desired_L_min = L_min;
    data.desired_L_max = L_max;
    data.desired_d_min = d_min;
    data.desired_angle_min = angle_min;
    data.desired_angle_max = angle_max;
    data.desired_dihedral_angle_min = dihedral_angle_min;
    data.desired_vel_max = vel_max;
    data.desired_acc_max = acc_max;
    data.n_ini = n;
    data.L_nom = L_nom;
    data.M = M;
    data.dt = dt;
    data.non_front_node = non_front_node;
    data.mass_sphere = mass_sphere;
    data.mass_member = mass_member;
    data.com_member = com_member;
    data.max_compressive_force = comp_strength;
    data.max_tensile_force = tens_strength;
    data.desired_stb_margin = stb_margin;

    % configuration data
    data.x(index,:) = x_ini;
    data.dx(index,:) = zeros(1,length(x_ini));
    data.L(index,:) = L_ini;
    data.L_min(index,:) = min(data.L(index,:));
    data.L_max(index,:) = max(data.L(index,:));
    data.dL(index,:) = zeros(1,length(n.elist));
    data.vel_max(index,1) = max(abs(data.dL(index,:)));
    data.x_cm(index,:) = cm_cal(n.elist,data.x(index,:)');
    data.stability(index,1) = 1;
    data.rolling(index,1) = 0;
    data.rotate_node(index,:) = [NaN NaN];
    data.angle_min(index,1) = min(angle);
    data.angle_max(index,1) = max(link_angle);
    data.dihedral_angle_min(index,1) = min(dihedral_angle);
    data.fixed_node{index,1} = fixed_node;

    % force data
%     data.compressive_force(index,:) = P_c;
%     data.tensile_force(index,:) = P_t;
%     data.Reaction{index,1} = Reaction;
%     data.Reaction_Req_Fric{index,1} = Req_Fric;
%     data.Reaction_Normal_Force{index,1} = Normal_Force;
%     data.MaxStaticCoeff = MaxStaticCoeff;
%     data.normal_vector{index,1} = surface_normal_vector;



    data.front_node{index,1} = [];
    data.path_index(index) = 0;

    data.manip_min = manip_min;

%     constraint = constraint_gen(x_ini,n,data,fixed_node,surface_normal_vector,data.MaxStaticCoeff);
%     data.constraint{index,1} = constraint;
%     data.d_min(index,1) = d_min_cal(constraint,data,n);
    data.stb_margin = [];

end