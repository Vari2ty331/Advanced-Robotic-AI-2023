function [constraint,dfdx,P] = constraint_gen(x,n,data,fixed_node,surface_normal_vector,MaxStaticCoeff,f_count)
% Constraint index information
% index 1~24 : Minimum length and maximum length constraints
% index 25~60 : Minimum angle constraints
% index 61~84 : Maximum angle constraints
% index 85~108 : Dihedral angle constraints
% index 109 : Manipulability constraint
% index 110 : Tensile strength constraint (Max tensile force)
% index 111~113 : Friction constraints
% index 114~end : Collision between members

if nargout < 2
    CalDiff = 0;
else
    CalDiff = 1;
end

delta = 1e-6;

pos = reshape(x,size(n.pos));
elist = n.elist;
strength_check = 1;

%% data read
link_connection = n.link_connection;
elist = n.elist;

L_min = data.desired_L_min;
L_max = data.desired_L_max;
d_min = data.desired_d_min;
angle_min = data.desired_angle_min;
angle_max = data.desired_angle_max;
dihedral_angle_min = data.desired_dihedral_angle_min;

mass_sphere = data.mass_sphere;
mass_member = data.mass_member;
com_member = data.com_member;
max_compressive_force = data.max_compressive_force;
max_tensile_force = data.max_tensile_force;

fixed_member = data.fixed_member{end};

index = 1;
constraint = 0;

%% member length constraint
dfdx = zeros(1,length(x));

for k = 1:length(elist)
    i = elist(k,1);
    j = elist(k,2);
    xi = x(i);
    xj = x(j);
    yi = x(length(pos) + i);
    yj = x(length(pos) + j);
    zi = x(2*length(pos) + i);
    zj = x(2*length(pos) + j);
    
    % minimum length constraint
    constraint(index) = norm([xi-xj yi-yj zi-zj])^2 - L_min(k)^2;
    
    if CalDiff == 1
        dfdx(index,i) = 2*(xi - xj);
        dfdx(index,j) = 2*(xj - xi);
        dfdx(index,length(pos) + i) = 2*(yi - yj);
        dfdx(index,length(pos) + j) = 2*(yj - yi);
        dfdx(index,2*length(pos) + i) = 2*(zi - zj);
        dfdx(index,2*length(pos) + j) = 2*(zj - zi);
    end
    
    index = index + 1;
    
    % maximum length constraint
    constraint(index) = L_max(k)^2 - norm([xi-xj yi-yj zi-zj])^2;

    if CalDiff == 1
        dfdx(index,i) = -2*(xi - xj);
        dfdx(index,j) = -2*(xj - xi);
        dfdx(index,length(pos) + i) = -2*(yi - yj);
        dfdx(index,length(pos) + j) = -2*(yj - yi);
        dfdx(index,2*length(pos) + i) = -2*(zi - zj);
        dfdx(index,2*length(pos) + j) = -2*(zj - zi); 
    end
    
    index = index + 1;
end

%% ground contact constraint
% for k = 1:length(pos)
%     constraint(index) = x(2*length(pos) + k,1);
%     dfdx(index,2*length(pos) + k) = 1;
%     index = index + 1;
% end

%% minimum angle constraint
adj = adj_gen(elist);

for i = 1:length(pos)
    for j = 1:length(adj{i})
        for k = j+1:length(adj{i})
            % node number
            p1 = i;
            p2 = adj{i}(j);
            p3 = adj{i}(k);
            
            % node position
            x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
            x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
            x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);
            
            v1 = [x2 y2 z2] - [x1 y1 z1];
            v2 = [x3 y3 z3] - [x1 y1 z1];
            
%             % minimum angle calculate 
%             member1 = edge_find(p1,p2,elist);
%             member2 = edge_find(p1,p3,elist);
%             end1 = find(elist(member1,:) == p2);
%             end2 = find(elist(member2,:) == p3);
%             node_combine = sort([n.member_end(member1,end1) n.member_end(member2,end2)]);
%             
            % 1: master drum,  2: master,  3: passive drum,  4: passive           
            
%             if isequal(node_combine, [1 3])
%                 angle_min = 33.5/180*pi;
%             elseif isequal(node_combine, [1 4])
%                 angle_min = 33.5/180*pi;
%             elseif isequal(node_combine, [2 3])
%                 angle_min = 33.5/180*pi;
%             elseif isequal(node_combine, [2 4])
%                 angle_min = 33.5/180*pi;
%             elseif isequal(node_combine, [3 3])
%                 angle_min = 26.6/180*pi;
%             elseif isequal(node_combine, [3 4])
%                 angle_min = 19.6/180*pi;
%             elseif isequal(node_combine, [4 4])
%                 angle_min = 19.6/180*pi;
%             elseif isequal(node_combine, [1 5])
%                 angle_min = 15/180*pi;
%             elseif isequal(node_combine, [2 5])
%                 angle_min = 15/180*pi;
%             elseif isequal(node_combine, [3 5])
%                 angle_min = 15/180*pi;
%             elseif isequal(node_combine, [4 5])
%                 angle_min = 15/180*pi;
%             elseif isequal(node_combine, [5 5])
%                 angle_min = 15/180*pi;
%             end
                        
            % minimum angle constraint
            constraint(index) = vector_angle(v1,v2) - angle_min;          
            
            if CalDiff == 1
                fun = @(x) acos( dot( [x(2) x(5) x(8)]-[x(1) x(4) x(7)] , [x(3) x(6) x(9)]-[x(1) x(4) x(7)] ) / norm( [x(2) x(5) x(8)]-[x(1) x(4) x(7)] ) / norm( [x(3) x(6) x(9)]-[x(1) x(4) x(7)] ) );
                x0 = [x1 x2 x3 y1 y2 y3 z1 z2 z3];
                grad = grad_simple(fun,x0,delta);
                dfdx(index,p1) = grad(1);
                dfdx(index,p2) = grad(2);
                dfdx(index,p3) = grad(3);
                dfdx(index,length(pos)+p1) = grad(4);
                dfdx(index,length(pos)+p2) = grad(5);
                dfdx(index,length(pos)+p3) = grad(6);
                dfdx(index,2*length(pos)+p1) = grad(7);
                dfdx(index,2*length(pos)+p2) = grad(8);
                dfdx(index,2*length(pos)+p3) = grad(9);
            end
                
            index = index + 1;
        end
    end
end

%% maximum angle constraint
for i = 1:length(link_connection)
    for j = 1:length(link_connection{i})
        % node number
        p1 = i;
        p2 = elist(link_connection{i}(j,1),:);
        p3 = elist(link_connection{i}(j,2),:);

        p2(p2 == p1) = [];
        p3(p3 == p1) = [];

        % node position
        x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
        x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
        x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);

        v1 = [x2 y2 z2] - [x1 y1 z1];
        v2 = [x3 y3 z3] - [x1 y1 z1];

        constraint(index) = angle_max - vector_angle(v1,v2);

        if CalDiff == 1
            fun = @(x) acos( dot( [x(2) x(5) x(8)]-[x(1) x(4) x(7)] , [x(3) x(6) x(9)]-[x(1) x(4) x(7)] ) / norm( [x(2) x(5) x(8)]-[x(1) x(4) x(7)] ) / norm( [x(3) x(6) x(9)]-[x(1) x(4) x(7)] ) );
            x0 = [x1 x2 x3 y1 y2 y3 z1 z2 z3];
            grad = grad_simple(fun,x0,delta);
            % give minus sine to differentiation of minimum angle constraint
            dfdx(index,p1) = -grad(1);
            dfdx(index,p2) = -grad(2);
            dfdx(index,p3) = -grad(3);
            dfdx(index,length(pos)+p1) = -grad(4);
            dfdx(index,length(pos)+p2) = -grad(5);
            dfdx(index,length(pos)+p3) = -grad(6);
            dfdx(index,2*length(pos)+p1) = -grad(7);
            dfdx(index,2*length(pos)+p2) = -grad(8);
            dfdx(index,2*length(pos)+p3) = -grad(9);
        end
        
        index = index + 1;           
    end
end

%% Dihedral angle constraint
dihedral_angle_connection = dihedral_angle_connection_gen(link_connection);
for i = 1:size(dihedral_angle_connection,1)
    dihedral_angle = dihedral_angle_sub_cal(dihedral_angle_connection(i,:),elist,pos);
    constraint(index) = dihedral_angle - dihedral_angle_min;
    
    if CalDiff == 1
        fun = @(x) dihedral_angle_sub_cal(dihedral_angle_connection(i,:),elist,x);    
        x0 = pos;
        grad = grad_simple(fun,x0,delta);
        dfdx(index,:) = grad;
    end
    
    index = index + 1;
end


%% Manipulability constraint

manip_min = data.manip_min;

manip = manipulability_cal(elist,pos);
constraint(index) = manip - manip_min;

if CalDiff == 1
    fun = @(pos) manipulability_cal(elist,pos);
    x0 = pos;
    dfdx(index,:) = grad_simple(fun,x0,delta);
% grad = grad_simple(fun,x0,delta);
% 
% for i = 1:numel(pos)
%     node_num = ceil(i/3);
%     if mod(i,3) == 1
%         dfdx(index,node_num) = grad(i);
%     elseif mod(i,3) == 2
%         dfdx(index,length(pos) + node_num) = grad(i);
%     elseif mod(i,3) == 0
%         dfdx(index,2*length(pos) + node_num) = grad(i);
%     end
% end
end

index = index + 1;

%% Strength constraint
P = truss_force_cal(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff);

if strength_check
% % Compressive strength constraint
% 
% constraint(index) = max_compressive_force - max(max(P)); % compressive force constraint
% 
% fun = @(pos) -max(max(truss_force_cal(elist,pos,mass_node,mass_member,fixed_node)));
% x0 = pos;
% grad = grad_simple(fun,x0,delta);
% 
% for i = 1:numel(pos)
%     node_num = ceil(i/3);
%     if mod(i,3) == 1
%         dfdx(index,node_num) = grad(i);
%     elseif mod(i,3) == 2
%         dfdx(index,length(pos) + node_num) = grad(i);
%     elseif mod(i,3) == 0
%         dfdx(index,2*length(pos) + node_num) = grad(i);
%     end
% end
% 
% index = index + 1;

% Tensile strength constraint
force_index = 3; % force_index = 3: force applied at tensioner, force_index = 4: force applied at zipper drive. (+): compressive, (-): tensile
P_t = P(:,force_index); 
constraint(index) = max_tensile_force + min(P_t); 

if CalDiff == 1
    fun = @(pos) min(truss_force_cal(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff,force_index)); 
    x0 = pos;
    grad = grad_simple(fun,x0,delta);

    for i = 1:numel(pos)
        node_num = ceil(i/3);
        if mod(i,3) == 1
            dfdx(index,node_num) = grad(i);
        elseif mod(i,3) == 2
            dfdx(index,length(pos) + node_num) = grad(i);
        elseif mod(i,3) == 0
            dfdx(index,2*length(pos) + node_num) = grad(i);
        end
    end
end

index = index + 1;

end


%% Friction constraint

% Change the shape of Friction related vectors.

fric_cons = comp_fric_cal(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff); % difference between the max static friction and required friction force.

if CalDiff == 1
    fric_cons_fun = @(pos) comp_fric_cal(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff);
    x0 = pos;
    fric_cons_grad = grad_matrix(fric_cons_fun,x0,delta);

    dfdx = [dfdx; fric_cons_grad;];
end

for i = 1 : length(fixed_node)
    constraint(index) = fric_cons(i);
    index = index + 1;
end

%% collision avoid constraint
clist = clist_gen(elist);

if d_min > 0

for k = 1:length(clist)

    % %%%%% line to line collision check %%%%%% 
    % node number
    p1 = elist(clist(k,1),1);
    p2 = elist(clist(k,1),2);
    p3 = elist(clist(k,2),1);
    p4 = elist(clist(k,2),2);
           
    x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
    x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
    x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);
    x4 = pos(p4,1);     y4 = pos(p4,2);     z4 = pos(p4,3);
    
    t1 = (x1^2*y3^2 - 2*x1^2*y3*y4 + x1^2*y4^2 + x1^2*z3^2 - 2*x1^2*z3*z4 + x1^2*z4^2 - 2*x1*x3*y1*y3 + 2*x1*x3*y1*y4 + x1*x3*y3*y4 + y2*x1*x3*y3 - x1*x3*y4^2 - y2*x1*x3*y4 - 2*x1*x3*z1*z3 + 2*x1*x3*z1*z4 + x1*x3*z3*z4 + z2*x1*x3*z3 - x1*x3*z4^2 - z2*x1*x3*z4 + 2*x1*x4*y1*y3 - 2*x1*x4*y1*y4 - x1*x4*y3^2 + x1*x4*y3*y4 - y2*x1*x4*y3 + y2*x1*x4*y4 + 2*x1*x4*z1*z3 - 2*x1*x4*z1*z4 - x1*x4*z3^2 + x1*x4*z3*z4 - z2*x1*x4*z3 + z2*x1*x4*z4 - x2*x1*y3^2 + 2*x2*x1*y3*y4 - x2*x1*y4^2 - x2*x1*z3^2 + 2*x2*x1*z3*z4 - x2*x1*z4^2 + x3^2*y1^2 - x3^2*y1*y4 - y2*x3^2*y1 + y2*x3^2*y4 + x3^2*z1^2 - x3^2*z1*z4 - z2*x3^2*z1 + z2*x3^2*z4 - 2*x3*x4*y1^2 + x3*x4*y1*y3 + x3*x4*y1*y4 + 2*y2*x3*x4*y1 - y2*x3*x4*y3 - y2*x3*x4*y4 - 2*x3*x4*z1^2 + x3*x4*z1*z3 + x3*x4*z1*z4 + 2*z2*x3*x4*z1 - z2*x3*x4*z3 - z2*x3*x4*z4 + x2*x3*y1*y3 - x2*x3*y1*y4 - x2*x3*y3*y4 + x2*x3*y4^2 + x2*x3*z1*z3 - x2*x3*z1*z4 - x2*x3*z3*z4 + x2*x3*z4^2 + x4^2*y1^2 - x4^2*y1*y3 - y2*x4^2*y1 + y2*x4^2*y3 + x4^2*z1^2 - x4^2*z1*z3 - z2*x4^2*z1 + z2*x4^2*z3 - x2*x4*y1*y3 + x2*x4*y1*y4 + x2*x4*y3^2 - x2*x4*y3*y4 - x2*x4*z1*z3 + x2*x4*z1*z4 + x2*x4*z3^2 - x2*x4*z3*z4 + y1^2*z3^2 - 2*y1^2*z3*z4 + y1^2*z4^2 - 2*y1*y3*z1*z3 + 2*y1*y3*z1*z4 + y1*y3*z3*z4 + z2*y1*y3*z3 - y1*y3*z4^2 - z2*y1*y3*z4 + 2*y1*y4*z1*z3 - 2*y1*y4*z1*z4 - y1*y4*z3^2 + y1*y4*z3*z4 - z2*y1*y4*z3 + z2*y1*y4*z4 - y2*y1*z3^2 + 2*y2*y1*z3*z4 - y2*y1*z4^2 + y3^2*z1^2 - y3^2*z1*z4 - z2*y3^2*z1 + z2*y3^2*z4 - 2*y3*y4*z1^2 + y3*y4*z1*z3 + y3*y4*z1*z4 + 2*z2*y3*y4*z1 - z2*y3*y4*z3 - z2*y3*y4*z4 + y2*y3*z1*z3 - y2*y3*z1*z4 - y2*y3*z3*z4 + y2*y3*z4^2 + y4^2*z1^2 - y4^2*z1*z3 - z2*y4^2*z1 + z2*y4^2*z3 - y2*y4*z1*z3 + y2*y4*z1*z4 + y2*y4*z3^2 - y2*y4*z3*z4)/(x1^2*y3^2 - 2*x1^2*y3*y4 + x1^2*y4^2 + x1^2*z3^2 - 2*x1^2*z3*z4 + x1^2*z4^2 - 2*x1*x2*y3^2 + 4*x1*x2*y3*y4 - 2*x1*x2*y4^2 - 2*x1*x2*z3^2 + 4*x1*x2*z3*z4 - 2*x1*x2*z4^2 - 2*x1*x3*y1*y3 + 2*x1*x3*y1*y4 + 2*x1*x3*y2*y3 - 2*x1*x3*y2*y4 - 2*x1*x3*z1*z3 + 2*x1*x3*z1*z4 + 2*x1*x3*z2*z3 - 2*x1*x3*z2*z4 + 2*x1*x4*y1*y3 - 2*x1*x4*y1*y4 - 2*x1*x4*y2*y3 + 2*x1*x4*y2*y4 + 2*x1*x4*z1*z3 - 2*x1*x4*z1*z4 - 2*x1*x4*z2*z3 + 2*x1*x4*z2*z4 + x2^2*y3^2 - 2*x2^2*y3*y4 + x2^2*y4^2 + x2^2*z3^2 - 2*x2^2*z3*z4 + x2^2*z4^2 + 2*x2*x3*y1*y3 - 2*x2*x3*y1*y4 - 2*x2*x3*y2*y3 + 2*x2*x3*y2*y4 + 2*x2*x3*z1*z3 - 2*x2*x3*z1*z4 - 2*x2*x3*z2*z3 + 2*x2*x3*z2*z4 - 2*x2*x4*y1*y3 + 2*x2*x4*y1*y4 + 2*x2*x4*y2*y3 - 2*x2*x4*y2*y4 - 2*x2*x4*z1*z3 + 2*x2*x4*z1*z4 + 2*x2*x4*z2*z3 - 2*x2*x4*z2*z4 + x3^2*y1^2 - 2*x3^2*y1*y2 + x3^2*y2^2 + x3^2*z1^2 - 2*x3^2*z1*z2 + x3^2*z2^2 - 2*x3*x4*y1^2 + 4*x3*x4*y1*y2 - 2*x3*x4*y2^2 - 2*x3*x4*z1^2 + 4*x3*x4*z1*z2 - 2*x3*x4*z2^2 + x4^2*y1^2 - 2*x4^2*y1*y2 + x4^2*y2^2 + x4^2*z1^2 - 2*x4^2*z1*z2 + x4^2*z2^2 + y1^2*z3^2 - 2*y1^2*z3*z4 + y1^2*z4^2 - 2*y1*y2*z3^2 + 4*y1*y2*z3*z4 - 2*y1*y2*z4^2 - 2*y1*y3*z1*z3 + 2*y1*y3*z1*z4 + 2*y1*y3*z2*z3 - 2*y1*y3*z2*z4 + 2*y1*y4*z1*z3 - 2*y1*y4*z1*z4 - 2*y1*y4*z2*z3 + 2*y1*y4*z2*z4 + y2^2*z3^2 - 2*y2^2*z3*z4 + y2^2*z4^2 + 2*y2*y3*z1*z3 - 2*y2*y3*z1*z4 - 2*y2*y3*z2*z3 + 2*y2*y3*z2*z4 - 2*y2*y4*z1*z3 + 2*y2*y4*z1*z4 + 2*y2*y4*z2*z3 - 2*y2*y4*z2*z4 + y3^2*z1^2 - 2*y3^2*z1*z2 + y3^2*z2^2 - 2*y3*y4*z1^2 + 4*y3*y4*z1*z2 - 2*y3*y4*z2^2 + y4^2*z1^2 - 2*y4^2*z1*z2 + y4^2*z2^2);
    t2 = (y4*x1^2*y2 - x1^2*y2*y3 + x1^2*y3^2 - y4*x1^2*y3 - x1^2*z2*z3 + z4*x1^2*z2 + x1^2*z3^2 - z4*x1^2*z3 + x1*x2*y1*y3 - y4*x1*x2*y1 + x1*x2*y2*y3 - y4*x1*x2*y2 - 2*x1*x2*y3^2 + 2*y4*x1*x2*y3 + x1*x2*z1*z3 - z4*x1*x2*z1 + x1*x2*z2*z3 - z4*x1*x2*z2 - 2*x1*x2*z3^2 + 2*z4*x1*x2*z3 + x1*x3*y1*y2 - 2*x1*x3*y1*y3 + y4*x1*x3*y1 - x1*x3*y2^2 + 2*x1*x3*y2*y3 - y4*x1*x3*y2 + x1*x3*z1*z2 - 2*x1*x3*z1*z3 + z4*x1*x3*z1 - x1*x3*z2^2 + 2*x1*x3*z2*z3 - z4*x1*x3*z2 - x4*x1*y1*y2 + x4*x1*y1*y3 + x4*x1*y2^2 - x4*x1*y2*y3 - x4*x1*z1*z2 + x4*x1*z1*z3 + x4*x1*z2^2 - x4*x1*z2*z3 - x2^2*y1*y3 + y4*x2^2*y1 + x2^2*y3^2 - y4*x2^2*y3 - x2^2*z1*z3 + z4*x2^2*z1 + x2^2*z3^2 - z4*x2^2*z3 - x2*x3*y1^2 + x2*x3*y1*y2 + 2*x2*x3*y1*y3 - y4*x2*x3*y1 - 2*x2*x3*y2*y3 + y4*x2*x3*y2 - x2*x3*z1^2 + x2*x3*z1*z2 + 2*x2*x3*z1*z3 - z4*x2*x3*z1 - 2*x2*x3*z2*z3 + z4*x2*x3*z2 + x4*x2*y1^2 - x4*x2*y1*y2 - x4*x2*y1*y3 + x4*x2*y2*y3 + x4*x2*z1^2 - x4*x2*z1*z2 - x4*x2*z1*z3 + x4*x2*z2*z3 + x3^2*y1^2 - 2*x3^2*y1*y2 + x3^2*y2^2 + x3^2*z1^2 - 2*x3^2*z1*z2 + x3^2*z2^2 - x4*x3*y1^2 + 2*x4*x3*y1*y2 - x4*x3*y2^2 - x4*x3*z1^2 + 2*x4*x3*z1*z2 - x4*x3*z2^2 - y1^2*z2*z3 + z4*y1^2*z2 + y1^2*z3^2 - z4*y1^2*z3 + y1*y2*z1*z3 - z4*y1*y2*z1 + y1*y2*z2*z3 - z4*y1*y2*z2 - 2*y1*y2*z3^2 + 2*z4*y1*y2*z3 + y1*y3*z1*z2 - 2*y1*y3*z1*z3 + z4*y1*y3*z1 - y1*y3*z2^2 + 2*y1*y3*z2*z3 - z4*y1*y3*z2 - y4*y1*z1*z2 + y4*y1*z1*z3 + y4*y1*z2^2 - y4*y1*z2*z3 - y2^2*z1*z3 + z4*y2^2*z1 + y2^2*z3^2 - z4*y2^2*z3 - y2*y3*z1^2 + y2*y3*z1*z2 + 2*y2*y3*z1*z3 - z4*y2*y3*z1 - 2*y2*y3*z2*z3 + z4*y2*y3*z2 + y4*y2*z1^2 - y4*y2*z1*z2 - y4*y2*z1*z3 + y4*y2*z2*z3 + y3^2*z1^2 - 2*y3^2*z1*z2 + y3^2*z2^2 - y4*y3*z1^2 + 2*y4*y3*z1*z2 - y4*y3*z2^2)/(x1^2*y3^2 - 2*x1^2*y3*y4 + x1^2*y4^2 + x1^2*z3^2 - 2*x1^2*z3*z4 + x1^2*z4^2 - 2*x1*x2*y3^2 + 4*x1*x2*y3*y4 - 2*x1*x2*y4^2 - 2*x1*x2*z3^2 + 4*x1*x2*z3*z4 - 2*x1*x2*z4^2 - 2*x1*x3*y1*y3 + 2*x1*x3*y1*y4 + 2*x1*x3*y2*y3 - 2*x1*x3*y2*y4 - 2*x1*x3*z1*z3 + 2*x1*x3*z1*z4 + 2*x1*x3*z2*z3 - 2*x1*x3*z2*z4 + 2*x1*x4*y1*y3 - 2*x1*x4*y1*y4 - 2*x1*x4*y2*y3 + 2*x1*x4*y2*y4 + 2*x1*x4*z1*z3 - 2*x1*x4*z1*z4 - 2*x1*x4*z2*z3 + 2*x1*x4*z2*z4 + x2^2*y3^2 - 2*x2^2*y3*y4 + x2^2*y4^2 + x2^2*z3^2 - 2*x2^2*z3*z4 + x2^2*z4^2 + 2*x2*x3*y1*y3 - 2*x2*x3*y1*y4 - 2*x2*x3*y2*y3 + 2*x2*x3*y2*y4 + 2*x2*x3*z1*z3 - 2*x2*x3*z1*z4 - 2*x2*x3*z2*z3 + 2*x2*x3*z2*z4 - 2*x2*x4*y1*y3 + 2*x2*x4*y1*y4 + 2*x2*x4*y2*y3 - 2*x2*x4*y2*y4 - 2*x2*x4*z1*z3 + 2*x2*x4*z1*z4 + 2*x2*x4*z2*z3 - 2*x2*x4*z2*z4 + x3^2*y1^2 - 2*x3^2*y1*y2 + x3^2*y2^2 + x3^2*z1^2 - 2*x3^2*z1*z2 + x3^2*z2^2 - 2*x3*x4*y1^2 + 4*x3*x4*y1*y2 - 2*x3*x4*y2^2 - 2*x3*x4*z1^2 + 4*x3*x4*z1*z2 - 2*x3*x4*z2^2 + x4^2*y1^2 - 2*x4^2*y1*y2 + x4^2*y2^2 + x4^2*z1^2 - 2*x4^2*z1*z2 + x4^2*z2^2 + y1^2*z3^2 - 2*y1^2*z3*z4 + y1^2*z4^2 - 2*y1*y2*z3^2 + 4*y1*y2*z3*z4 - 2*y1*y2*z4^2 - 2*y1*y3*z1*z3 + 2*y1*y3*z1*z4 + 2*y1*y3*z2*z3 - 2*y1*y3*z2*z4 + 2*y1*y4*z1*z3 - 2*y1*y4*z1*z4 - 2*y1*y4*z2*z3 + 2*y1*y4*z2*z4 + y2^2*z3^2 - 2*y2^2*z3*z4 + y2^2*z4^2 + 2*y2*y3*z1*z3 - 2*y2*y3*z1*z4 - 2*y2*y3*z2*z3 + 2*y2*y3*z2*z4 - 2*y2*y4*z1*z3 + 2*y2*y4*z1*z4 + 2*y2*y4*z2*z3 - 2*y2*y4*z2*z4 + y3^2*z1^2 - 2*y3^2*z1*z2 + y3^2*z2^2 - 2*y3*y4*z1^2 + 4*y3*y4*z1*z2 - 2*y3*y4*z2^2 + y4^2*z1^2 - 2*y4^2*z1*z2 + y4^2*z2^2);
    dist2 = (x1 - x3 - t1*(x1 - x2) + t2*(x3 - x4))^2 + (y1 - y3 - t1*(y1 - y2) + t2*(y3 - y4))^2 + (z1 - z3 - t1*(z1 - z2) + t2*(z3 - z4))^2;
    
    if (t1<=1) && (t1>=0) && (t2<=1) && (t2>=0)
        constraint(index) = dist2 - d_min^2;
    else
        constraint(index) = inf;
    end
    
    % gradient calculation
    if CalDiff == 1
        fun = @(x) (x(1) - x(3) - ((x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + x(1)*x(3)*x(7)*x(8) + x(6)*x(1)*x(3)*x(7) - x(1)*x(3)*x(8)^2 - x(6)*x(1)*x(3)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + x(1)*x(3)*x(11)*x(12) + x(10)*x(1)*x(3)*x(11) - x(1)*x(3)*x(12)^2 - x(10)*x(1)*x(3)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - x(1)*x(4)*x(7)^2 + x(1)*x(4)*x(7)*x(8) - x(6)*x(1)*x(4)*x(7) + x(6)*x(1)*x(4)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - x(1)*x(4)*x(11)^2 + x(1)*x(4)*x(11)*x(12) - x(10)*x(1)*x(4)*x(11) + x(10)*x(1)*x(4)*x(12) - x(2)*x(1)*x(7)^2 + 2*x(2)*x(1)*x(7)*x(8) - x(2)*x(1)*x(8)^2 - x(2)*x(1)*x(11)^2 + 2*x(2)*x(1)*x(11)*x(12) - x(2)*x(1)*x(12)^2 + x(3)^2*x(5)^2 - x(3)^2*x(5)*x(8) - x(6)*x(3)^2*x(5) + x(6)*x(3)^2*x(8) + x(3)^2*x(9)^2 - x(3)^2*x(9)*x(12) - x(10)*x(3)^2*x(9) + x(10)*x(3)^2*x(12) - 2*x(3)*x(4)*x(5)^2 + x(3)*x(4)*x(5)*x(7) + x(3)*x(4)*x(5)*x(8) + 2*x(6)*x(3)*x(4)*x(5) - x(6)*x(3)*x(4)*x(7) - x(6)*x(3)*x(4)*x(8) - 2*x(3)*x(4)*x(9)^2 + x(3)*x(4)*x(9)*x(11) + x(3)*x(4)*x(9)*x(12) + 2*x(10)*x(3)*x(4)*x(9) - x(10)*x(3)*x(4)*x(11) - x(10)*x(3)*x(4)*x(12) + x(2)*x(3)*x(5)*x(7) - x(2)*x(3)*x(5)*x(8) - x(2)*x(3)*x(7)*x(8) + x(2)*x(3)*x(8)^2 + x(2)*x(3)*x(9)*x(11) - x(2)*x(3)*x(9)*x(12) - x(2)*x(3)*x(11)*x(12) + x(2)*x(3)*x(12)^2 + x(4)^2*x(5)^2 - x(4)^2*x(5)*x(7) - x(6)*x(4)^2*x(5) + x(6)*x(4)^2*x(7) + x(4)^2*x(9)^2 - x(4)^2*x(9)*x(11) - x(10)*x(4)^2*x(9) + x(10)*x(4)^2*x(11) - x(2)*x(4)*x(5)*x(7) + x(2)*x(4)*x(5)*x(8) + x(2)*x(4)*x(7)^2 - x(2)*x(4)*x(7)*x(8) - x(2)*x(4)*x(9)*x(11) + x(2)*x(4)*x(9)*x(12) + x(2)*x(4)*x(11)^2 - x(2)*x(4)*x(11)*x(12) + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + x(5)*x(7)*x(11)*x(12) + x(10)*x(5)*x(7)*x(11) - x(5)*x(7)*x(12)^2 - x(10)*x(5)*x(7)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - x(5)*x(8)*x(11)^2 + x(5)*x(8)*x(11)*x(12) - x(10)*x(5)*x(8)*x(11) + x(10)*x(5)*x(8)*x(12) - x(6)*x(5)*x(11)^2 + 2*x(6)*x(5)*x(11)*x(12) - x(6)*x(5)*x(12)^2 + x(7)^2*x(9)^2 - x(7)^2*x(9)*x(12) - x(10)*x(7)^2*x(9) + x(10)*x(7)^2*x(12) - 2*x(7)*x(8)*x(9)^2 + x(7)*x(8)*x(9)*x(11) + x(7)*x(8)*x(9)*x(12) + 2*x(10)*x(7)*x(8)*x(9) - x(10)*x(7)*x(8)*x(11) - x(10)*x(7)*x(8)*x(12) + x(6)*x(7)*x(9)*x(11) - x(6)*x(7)*x(9)*x(12) - x(6)*x(7)*x(11)*x(12) + x(6)*x(7)*x(12)^2 + x(8)^2*x(9)^2 - x(8)^2*x(9)*x(11) - x(10)*x(8)^2*x(9) + x(10)*x(8)^2*x(11) - x(6)*x(8)*x(9)*x(11) + x(6)*x(8)*x(9)*x(12) + x(6)*x(8)*x(11)^2 - x(6)*x(8)*x(11)*x(12))/(x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(2)*x(7)^2 + 4*x(1)*x(2)*x(7)*x(8) - 2*x(1)*x(2)*x(8)^2 - 2*x(1)*x(2)*x(11)^2 + 4*x(1)*x(2)*x(11)*x(12) - 2*x(1)*x(2)*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + 2*x(1)*x(3)*x(6)*x(7) - 2*x(1)*x(3)*x(6)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + 2*x(1)*x(3)*x(10)*x(11) - 2*x(1)*x(3)*x(10)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - 2*x(1)*x(4)*x(6)*x(7) + 2*x(1)*x(4)*x(6)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - 2*x(1)*x(4)*x(10)*x(11) + 2*x(1)*x(4)*x(10)*x(12) + x(2)^2*x(7)^2 - 2*x(2)^2*x(7)*x(8) + x(2)^2*x(8)^2 + x(2)^2*x(11)^2 - 2*x(2)^2*x(11)*x(12) + x(2)^2*x(12)^2 + 2*x(2)*x(3)*x(5)*x(7) - 2*x(2)*x(3)*x(5)*x(8) - 2*x(2)*x(3)*x(6)*x(7) + 2*x(2)*x(3)*x(6)*x(8) + 2*x(2)*x(3)*x(9)*x(11) - 2*x(2)*x(3)*x(9)*x(12) - 2*x(2)*x(3)*x(10)*x(11) + 2*x(2)*x(3)*x(10)*x(12) - 2*x(2)*x(4)*x(5)*x(7) + 2*x(2)*x(4)*x(5)*x(8) + 2*x(2)*x(4)*x(6)*x(7) - 2*x(2)*x(4)*x(6)*x(8) - 2*x(2)*x(4)*x(9)*x(11) + 2*x(2)*x(4)*x(9)*x(12) + 2*x(2)*x(4)*x(10)*x(11) - 2*x(2)*x(4)*x(10)*x(12) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - 2*x(3)*x(4)*x(5)^2 + 4*x(3)*x(4)*x(5)*x(6) - 2*x(3)*x(4)*x(6)^2 - 2*x(3)*x(4)*x(9)^2 + 4*x(3)*x(4)*x(9)*x(10) - 2*x(3)*x(4)*x(10)^2 + x(4)^2*x(5)^2 - 2*x(4)^2*x(5)*x(6) + x(4)^2*x(6)^2 + x(4)^2*x(9)^2 - 2*x(4)^2*x(9)*x(10) + x(4)^2*x(10)^2 + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(6)*x(11)^2 + 4*x(5)*x(6)*x(11)*x(12) - 2*x(5)*x(6)*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + 2*x(5)*x(7)*x(10)*x(11) - 2*x(5)*x(7)*x(10)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - 2*x(5)*x(8)*x(10)*x(11) + 2*x(5)*x(8)*x(10)*x(12) + x(6)^2*x(11)^2 - 2*x(6)^2*x(11)*x(12) + x(6)^2*x(12)^2 + 2*x(6)*x(7)*x(9)*x(11) - 2*x(6)*x(7)*x(9)*x(12) - 2*x(6)*x(7)*x(10)*x(11) + 2*x(6)*x(7)*x(10)*x(12) - 2*x(6)*x(8)*x(9)*x(11) + 2*x(6)*x(8)*x(9)*x(12) + 2*x(6)*x(8)*x(10)*x(11) - 2*x(6)*x(8)*x(10)*x(12) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - 2*x(7)*x(8)*x(9)^2 + 4*x(7)*x(8)*x(9)*x(10) - 2*x(7)*x(8)*x(10)^2 + x(8)^2*x(9)^2 - 2*x(8)^2*x(9)*x(10) + x(8)^2*x(10)^2))*(x(1) - x(2)) + ((x(8)*x(1)^2*x(6) - x(1)^2*x(6)*x(7) + x(1)^2*x(7)^2 - x(8)*x(1)^2*x(7) - x(1)^2*x(10)*x(11) + x(12)*x(1)^2*x(10) + x(1)^2*x(11)^2 - x(12)*x(1)^2*x(11) + x(1)*x(2)*x(5)*x(7) - x(8)*x(1)*x(2)*x(5) + x(1)*x(2)*x(6)*x(7) - x(8)*x(1)*x(2)*x(6) - 2*x(1)*x(2)*x(7)^2 + 2*x(8)*x(1)*x(2)*x(7) + x(1)*x(2)*x(9)*x(11) - x(12)*x(1)*x(2)*x(9) + x(1)*x(2)*x(10)*x(11) - x(12)*x(1)*x(2)*x(10) - 2*x(1)*x(2)*x(11)^2 + 2*x(12)*x(1)*x(2)*x(11) + x(1)*x(3)*x(5)*x(6) - 2*x(1)*x(3)*x(5)*x(7) + x(8)*x(1)*x(3)*x(5) - x(1)*x(3)*x(6)^2 + 2*x(1)*x(3)*x(6)*x(7) - x(8)*x(1)*x(3)*x(6) + x(1)*x(3)*x(9)*x(10) - 2*x(1)*x(3)*x(9)*x(11) + x(12)*x(1)*x(3)*x(9) - x(1)*x(3)*x(10)^2 + 2*x(1)*x(3)*x(10)*x(11) - x(12)*x(1)*x(3)*x(10) - x(4)*x(1)*x(5)*x(6) + x(4)*x(1)*x(5)*x(7) + x(4)*x(1)*x(6)^2 - x(4)*x(1)*x(6)*x(7) - x(4)*x(1)*x(9)*x(10) + x(4)*x(1)*x(9)*x(11) + x(4)*x(1)*x(10)^2 - x(4)*x(1)*x(10)*x(11) - x(2)^2*x(5)*x(7) + x(8)*x(2)^2*x(5) + x(2)^2*x(7)^2 - x(8)*x(2)^2*x(7) - x(2)^2*x(9)*x(11) + x(12)*x(2)^2*x(9) + x(2)^2*x(11)^2 - x(12)*x(2)^2*x(11) - x(2)*x(3)*x(5)^2 + x(2)*x(3)*x(5)*x(6) + 2*x(2)*x(3)*x(5)*x(7) - x(8)*x(2)*x(3)*x(5) - 2*x(2)*x(3)*x(6)*x(7) + x(8)*x(2)*x(3)*x(6) - x(2)*x(3)*x(9)^2 + x(2)*x(3)*x(9)*x(10) + 2*x(2)*x(3)*x(9)*x(11) - x(12)*x(2)*x(3)*x(9) - 2*x(2)*x(3)*x(10)*x(11) + x(12)*x(2)*x(3)*x(10) + x(4)*x(2)*x(5)^2 - x(4)*x(2)*x(5)*x(6) - x(4)*x(2)*x(5)*x(7) + x(4)*x(2)*x(6)*x(7) + x(4)*x(2)*x(9)^2 - x(4)*x(2)*x(9)*x(10) - x(4)*x(2)*x(9)*x(11) + x(4)*x(2)*x(10)*x(11) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - x(4)*x(3)*x(5)^2 + 2*x(4)*x(3)*x(5)*x(6) - x(4)*x(3)*x(6)^2 - x(4)*x(3)*x(9)^2 + 2*x(4)*x(3)*x(9)*x(10) - x(4)*x(3)*x(10)^2 - x(5)^2*x(10)*x(11) + x(12)*x(5)^2*x(10) + x(5)^2*x(11)^2 - x(12)*x(5)^2*x(11) + x(5)*x(6)*x(9)*x(11) - x(12)*x(5)*x(6)*x(9) + x(5)*x(6)*x(10)*x(11) - x(12)*x(5)*x(6)*x(10) - 2*x(5)*x(6)*x(11)^2 + 2*x(12)*x(5)*x(6)*x(11) + x(5)*x(7)*x(9)*x(10) - 2*x(5)*x(7)*x(9)*x(11) + x(12)*x(5)*x(7)*x(9) - x(5)*x(7)*x(10)^2 + 2*x(5)*x(7)*x(10)*x(11) - x(12)*x(5)*x(7)*x(10) - x(8)*x(5)*x(9)*x(10) + x(8)*x(5)*x(9)*x(11) + x(8)*x(5)*x(10)^2 - x(8)*x(5)*x(10)*x(11) - x(6)^2*x(9)*x(11) + x(12)*x(6)^2*x(9) + x(6)^2*x(11)^2 - x(12)*x(6)^2*x(11) - x(6)*x(7)*x(9)^2 + x(6)*x(7)*x(9)*x(10) + 2*x(6)*x(7)*x(9)*x(11) - x(12)*x(6)*x(7)*x(9) - 2*x(6)*x(7)*x(10)*x(11) + x(12)*x(6)*x(7)*x(10) + x(8)*x(6)*x(9)^2 - x(8)*x(6)*x(9)*x(10) - x(8)*x(6)*x(9)*x(11) + x(8)*x(6)*x(10)*x(11) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - x(8)*x(7)*x(9)^2 + 2*x(8)*x(7)*x(9)*x(10) - x(8)*x(7)*x(10)^2)/(x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(2)*x(7)^2 + 4*x(1)*x(2)*x(7)*x(8) - 2*x(1)*x(2)*x(8)^2 - 2*x(1)*x(2)*x(11)^2 + 4*x(1)*x(2)*x(11)*x(12) - 2*x(1)*x(2)*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + 2*x(1)*x(3)*x(6)*x(7) - 2*x(1)*x(3)*x(6)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + 2*x(1)*x(3)*x(10)*x(11) - 2*x(1)*x(3)*x(10)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - 2*x(1)*x(4)*x(6)*x(7) + 2*x(1)*x(4)*x(6)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - 2*x(1)*x(4)*x(10)*x(11) + 2*x(1)*x(4)*x(10)*x(12) + x(2)^2*x(7)^2 - 2*x(2)^2*x(7)*x(8) + x(2)^2*x(8)^2 + x(2)^2*x(11)^2 - 2*x(2)^2*x(11)*x(12) + x(2)^2*x(12)^2 + 2*x(2)*x(3)*x(5)*x(7) - 2*x(2)*x(3)*x(5)*x(8) - 2*x(2)*x(3)*x(6)*x(7) + 2*x(2)*x(3)*x(6)*x(8) + 2*x(2)*x(3)*x(9)*x(11) - 2*x(2)*x(3)*x(9)*x(12) - 2*x(2)*x(3)*x(10)*x(11) + 2*x(2)*x(3)*x(10)*x(12) - 2*x(2)*x(4)*x(5)*x(7) + 2*x(2)*x(4)*x(5)*x(8) + 2*x(2)*x(4)*x(6)*x(7) - 2*x(2)*x(4)*x(6)*x(8) - 2*x(2)*x(4)*x(9)*x(11) + 2*x(2)*x(4)*x(9)*x(12) + 2*x(2)*x(4)*x(10)*x(11) - 2*x(2)*x(4)*x(10)*x(12) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - 2*x(3)*x(4)*x(5)^2 + 4*x(3)*x(4)*x(5)*x(6) - 2*x(3)*x(4)*x(6)^2 - 2*x(3)*x(4)*x(9)^2 + 4*x(3)*x(4)*x(9)*x(10) - 2*x(3)*x(4)*x(10)^2 + x(4)^2*x(5)^2 - 2*x(4)^2*x(5)*x(6) + x(4)^2*x(6)^2 + x(4)^2*x(9)^2 - 2*x(4)^2*x(9)*x(10) + x(4)^2*x(10)^2 + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(6)*x(11)^2 + 4*x(5)*x(6)*x(11)*x(12) - 2*x(5)*x(6)*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + 2*x(5)*x(7)*x(10)*x(11) - 2*x(5)*x(7)*x(10)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - 2*x(5)*x(8)*x(10)*x(11) + 2*x(5)*x(8)*x(10)*x(12) + x(6)^2*x(11)^2 - 2*x(6)^2*x(11)*x(12) + x(6)^2*x(12)^2 + 2*x(6)*x(7)*x(9)*x(11) - 2*x(6)*x(7)*x(9)*x(12) - 2*x(6)*x(7)*x(10)*x(11) + 2*x(6)*x(7)*x(10)*x(12) - 2*x(6)*x(8)*x(9)*x(11) + 2*x(6)*x(8)*x(9)*x(12) + 2*x(6)*x(8)*x(10)*x(11) - 2*x(6)*x(8)*x(10)*x(12) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - 2*x(7)*x(8)*x(9)^2 + 4*x(7)*x(8)*x(9)*x(10) - 2*x(7)*x(8)*x(10)^2 + x(8)^2*x(9)^2 - 2*x(8)^2*x(9)*x(10) + x(8)^2*x(10)^2))*(x(3) - x(4)))^2 + (x(5) - x(7) - ((x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + x(1)*x(3)*x(7)*x(8) + x(6)*x(1)*x(3)*x(7) - x(1)*x(3)*x(8)^2 - x(6)*x(1)*x(3)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + x(1)*x(3)*x(11)*x(12) + x(10)*x(1)*x(3)*x(11) - x(1)*x(3)*x(12)^2 - x(10)*x(1)*x(3)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - x(1)*x(4)*x(7)^2 + x(1)*x(4)*x(7)*x(8) - x(6)*x(1)*x(4)*x(7) + x(6)*x(1)*x(4)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - x(1)*x(4)*x(11)^2 + x(1)*x(4)*x(11)*x(12) - x(10)*x(1)*x(4)*x(11) + x(10)*x(1)*x(4)*x(12) - x(2)*x(1)*x(7)^2 + 2*x(2)*x(1)*x(7)*x(8) - x(2)*x(1)*x(8)^2 - x(2)*x(1)*x(11)^2 + 2*x(2)*x(1)*x(11)*x(12) - x(2)*x(1)*x(12)^2 + x(3)^2*x(5)^2 - x(3)^2*x(5)*x(8) - x(6)*x(3)^2*x(5) + x(6)*x(3)^2*x(8) + x(3)^2*x(9)^2 - x(3)^2*x(9)*x(12) - x(10)*x(3)^2*x(9) + x(10)*x(3)^2*x(12) - 2*x(3)*x(4)*x(5)^2 + x(3)*x(4)*x(5)*x(7) + x(3)*x(4)*x(5)*x(8) + 2*x(6)*x(3)*x(4)*x(5) - x(6)*x(3)*x(4)*x(7) - x(6)*x(3)*x(4)*x(8) - 2*x(3)*x(4)*x(9)^2 + x(3)*x(4)*x(9)*x(11) + x(3)*x(4)*x(9)*x(12) + 2*x(10)*x(3)*x(4)*x(9) - x(10)*x(3)*x(4)*x(11) - x(10)*x(3)*x(4)*x(12) + x(2)*x(3)*x(5)*x(7) - x(2)*x(3)*x(5)*x(8) - x(2)*x(3)*x(7)*x(8) + x(2)*x(3)*x(8)^2 + x(2)*x(3)*x(9)*x(11) - x(2)*x(3)*x(9)*x(12) - x(2)*x(3)*x(11)*x(12) + x(2)*x(3)*x(12)^2 + x(4)^2*x(5)^2 - x(4)^2*x(5)*x(7) - x(6)*x(4)^2*x(5) + x(6)*x(4)^2*x(7) + x(4)^2*x(9)^2 - x(4)^2*x(9)*x(11) - x(10)*x(4)^2*x(9) + x(10)*x(4)^2*x(11) - x(2)*x(4)*x(5)*x(7) + x(2)*x(4)*x(5)*x(8) + x(2)*x(4)*x(7)^2 - x(2)*x(4)*x(7)*x(8) - x(2)*x(4)*x(9)*x(11) + x(2)*x(4)*x(9)*x(12) + x(2)*x(4)*x(11)^2 - x(2)*x(4)*x(11)*x(12) + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + x(5)*x(7)*x(11)*x(12) + x(10)*x(5)*x(7)*x(11) - x(5)*x(7)*x(12)^2 - x(10)*x(5)*x(7)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - x(5)*x(8)*x(11)^2 + x(5)*x(8)*x(11)*x(12) - x(10)*x(5)*x(8)*x(11) + x(10)*x(5)*x(8)*x(12) - x(6)*x(5)*x(11)^2 + 2*x(6)*x(5)*x(11)*x(12) - x(6)*x(5)*x(12)^2 + x(7)^2*x(9)^2 - x(7)^2*x(9)*x(12) - x(10)*x(7)^2*x(9) + x(10)*x(7)^2*x(12) - 2*x(7)*x(8)*x(9)^2 + x(7)*x(8)*x(9)*x(11) + x(7)*x(8)*x(9)*x(12) + 2*x(10)*x(7)*x(8)*x(9) - x(10)*x(7)*x(8)*x(11) - x(10)*x(7)*x(8)*x(12) + x(6)*x(7)*x(9)*x(11) - x(6)*x(7)*x(9)*x(12) - x(6)*x(7)*x(11)*x(12) + x(6)*x(7)*x(12)^2 + x(8)^2*x(9)^2 - x(8)^2*x(9)*x(11) - x(10)*x(8)^2*x(9) + x(10)*x(8)^2*x(11) - x(6)*x(8)*x(9)*x(11) + x(6)*x(8)*x(9)*x(12) + x(6)*x(8)*x(11)^2 - x(6)*x(8)*x(11)*x(12))/(x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(2)*x(7)^2 + 4*x(1)*x(2)*x(7)*x(8) - 2*x(1)*x(2)*x(8)^2 - 2*x(1)*x(2)*x(11)^2 + 4*x(1)*x(2)*x(11)*x(12) - 2*x(1)*x(2)*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + 2*x(1)*x(3)*x(6)*x(7) - 2*x(1)*x(3)*x(6)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + 2*x(1)*x(3)*x(10)*x(11) - 2*x(1)*x(3)*x(10)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - 2*x(1)*x(4)*x(6)*x(7) + 2*x(1)*x(4)*x(6)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - 2*x(1)*x(4)*x(10)*x(11) + 2*x(1)*x(4)*x(10)*x(12) + x(2)^2*x(7)^2 - 2*x(2)^2*x(7)*x(8) + x(2)^2*x(8)^2 + x(2)^2*x(11)^2 - 2*x(2)^2*x(11)*x(12) + x(2)^2*x(12)^2 + 2*x(2)*x(3)*x(5)*x(7) - 2*x(2)*x(3)*x(5)*x(8) - 2*x(2)*x(3)*x(6)*x(7) + 2*x(2)*x(3)*x(6)*x(8) + 2*x(2)*x(3)*x(9)*x(11) - 2*x(2)*x(3)*x(9)*x(12) - 2*x(2)*x(3)*x(10)*x(11) + 2*x(2)*x(3)*x(10)*x(12) - 2*x(2)*x(4)*x(5)*x(7) + 2*x(2)*x(4)*x(5)*x(8) + 2*x(2)*x(4)*x(6)*x(7) - 2*x(2)*x(4)*x(6)*x(8) - 2*x(2)*x(4)*x(9)*x(11) + 2*x(2)*x(4)*x(9)*x(12) + 2*x(2)*x(4)*x(10)*x(11) - 2*x(2)*x(4)*x(10)*x(12) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - 2*x(3)*x(4)*x(5)^2 + 4*x(3)*x(4)*x(5)*x(6) - 2*x(3)*x(4)*x(6)^2 - 2*x(3)*x(4)*x(9)^2 + 4*x(3)*x(4)*x(9)*x(10) - 2*x(3)*x(4)*x(10)^2 + x(4)^2*x(5)^2 - 2*x(4)^2*x(5)*x(6) + x(4)^2*x(6)^2 + x(4)^2*x(9)^2 - 2*x(4)^2*x(9)*x(10) + x(4)^2*x(10)^2 + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(6)*x(11)^2 + 4*x(5)*x(6)*x(11)*x(12) - 2*x(5)*x(6)*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + 2*x(5)*x(7)*x(10)*x(11) - 2*x(5)*x(7)*x(10)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - 2*x(5)*x(8)*x(10)*x(11) + 2*x(5)*x(8)*x(10)*x(12) + x(6)^2*x(11)^2 - 2*x(6)^2*x(11)*x(12) + x(6)^2*x(12)^2 + 2*x(6)*x(7)*x(9)*x(11) - 2*x(6)*x(7)*x(9)*x(12) - 2*x(6)*x(7)*x(10)*x(11) + 2*x(6)*x(7)*x(10)*x(12) - 2*x(6)*x(8)*x(9)*x(11) + 2*x(6)*x(8)*x(9)*x(12) + 2*x(6)*x(8)*x(10)*x(11) - 2*x(6)*x(8)*x(10)*x(12) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - 2*x(7)*x(8)*x(9)^2 + 4*x(7)*x(8)*x(9)*x(10) - 2*x(7)*x(8)*x(10)^2 + x(8)^2*x(9)^2 - 2*x(8)^2*x(9)*x(10) + x(8)^2*x(10)^2))*(x(5) - x(6)) + ((x(8)*x(1)^2*x(6) - x(1)^2*x(6)*x(7) + x(1)^2*x(7)^2 - x(8)*x(1)^2*x(7) - x(1)^2*x(10)*x(11) + x(12)*x(1)^2*x(10) + x(1)^2*x(11)^2 - x(12)*x(1)^2*x(11) + x(1)*x(2)*x(5)*x(7) - x(8)*x(1)*x(2)*x(5) + x(1)*x(2)*x(6)*x(7) - x(8)*x(1)*x(2)*x(6) - 2*x(1)*x(2)*x(7)^2 + 2*x(8)*x(1)*x(2)*x(7) + x(1)*x(2)*x(9)*x(11) - x(12)*x(1)*x(2)*x(9) + x(1)*x(2)*x(10)*x(11) - x(12)*x(1)*x(2)*x(10) - 2*x(1)*x(2)*x(11)^2 + 2*x(12)*x(1)*x(2)*x(11) + x(1)*x(3)*x(5)*x(6) - 2*x(1)*x(3)*x(5)*x(7) + x(8)*x(1)*x(3)*x(5) - x(1)*x(3)*x(6)^2 + 2*x(1)*x(3)*x(6)*x(7) - x(8)*x(1)*x(3)*x(6) + x(1)*x(3)*x(9)*x(10) - 2*x(1)*x(3)*x(9)*x(11) + x(12)*x(1)*x(3)*x(9) - x(1)*x(3)*x(10)^2 + 2*x(1)*x(3)*x(10)*x(11) - x(12)*x(1)*x(3)*x(10) - x(4)*x(1)*x(5)*x(6) + x(4)*x(1)*x(5)*x(7) + x(4)*x(1)*x(6)^2 - x(4)*x(1)*x(6)*x(7) - x(4)*x(1)*x(9)*x(10) + x(4)*x(1)*x(9)*x(11) + x(4)*x(1)*x(10)^2 - x(4)*x(1)*x(10)*x(11) - x(2)^2*x(5)*x(7) + x(8)*x(2)^2*x(5) + x(2)^2*x(7)^2 - x(8)*x(2)^2*x(7) - x(2)^2*x(9)*x(11) + x(12)*x(2)^2*x(9) + x(2)^2*x(11)^2 - x(12)*x(2)^2*x(11) - x(2)*x(3)*x(5)^2 + x(2)*x(3)*x(5)*x(6) + 2*x(2)*x(3)*x(5)*x(7) - x(8)*x(2)*x(3)*x(5) - 2*x(2)*x(3)*x(6)*x(7) + x(8)*x(2)*x(3)*x(6) - x(2)*x(3)*x(9)^2 + x(2)*x(3)*x(9)*x(10) + 2*x(2)*x(3)*x(9)*x(11) - x(12)*x(2)*x(3)*x(9) - 2*x(2)*x(3)*x(10)*x(11) + x(12)*x(2)*x(3)*x(10) + x(4)*x(2)*x(5)^2 - x(4)*x(2)*x(5)*x(6) - x(4)*x(2)*x(5)*x(7) + x(4)*x(2)*x(6)*x(7) + x(4)*x(2)*x(9)^2 - x(4)*x(2)*x(9)*x(10) - x(4)*x(2)*x(9)*x(11) + x(4)*x(2)*x(10)*x(11) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - x(4)*x(3)*x(5)^2 + 2*x(4)*x(3)*x(5)*x(6) - x(4)*x(3)*x(6)^2 - x(4)*x(3)*x(9)^2 + 2*x(4)*x(3)*x(9)*x(10) - x(4)*x(3)*x(10)^2 - x(5)^2*x(10)*x(11) + x(12)*x(5)^2*x(10) + x(5)^2*x(11)^2 - x(12)*x(5)^2*x(11) + x(5)*x(6)*x(9)*x(11) - x(12)*x(5)*x(6)*x(9) + x(5)*x(6)*x(10)*x(11) - x(12)*x(5)*x(6)*x(10) - 2*x(5)*x(6)*x(11)^2 + 2*x(12)*x(5)*x(6)*x(11) + x(5)*x(7)*x(9)*x(10) - 2*x(5)*x(7)*x(9)*x(11) + x(12)*x(5)*x(7)*x(9) - x(5)*x(7)*x(10)^2 + 2*x(5)*x(7)*x(10)*x(11) - x(12)*x(5)*x(7)*x(10) - x(8)*x(5)*x(9)*x(10) + x(8)*x(5)*x(9)*x(11) + x(8)*x(5)*x(10)^2 - x(8)*x(5)*x(10)*x(11) - x(6)^2*x(9)*x(11) + x(12)*x(6)^2*x(9) + x(6)^2*x(11)^2 - x(12)*x(6)^2*x(11) - x(6)*x(7)*x(9)^2 + x(6)*x(7)*x(9)*x(10) + 2*x(6)*x(7)*x(9)*x(11) - x(12)*x(6)*x(7)*x(9) - 2*x(6)*x(7)*x(10)*x(11) + x(12)*x(6)*x(7)*x(10) + x(8)*x(6)*x(9)^2 - x(8)*x(6)*x(9)*x(10) - x(8)*x(6)*x(9)*x(11) + x(8)*x(6)*x(10)*x(11) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - x(8)*x(7)*x(9)^2 + 2*x(8)*x(7)*x(9)*x(10) - x(8)*x(7)*x(10)^2)/(x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(2)*x(7)^2 + 4*x(1)*x(2)*x(7)*x(8) - 2*x(1)*x(2)*x(8)^2 - 2*x(1)*x(2)*x(11)^2 + 4*x(1)*x(2)*x(11)*x(12) - 2*x(1)*x(2)*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + 2*x(1)*x(3)*x(6)*x(7) - 2*x(1)*x(3)*x(6)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + 2*x(1)*x(3)*x(10)*x(11) - 2*x(1)*x(3)*x(10)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - 2*x(1)*x(4)*x(6)*x(7) + 2*x(1)*x(4)*x(6)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - 2*x(1)*x(4)*x(10)*x(11) + 2*x(1)*x(4)*x(10)*x(12) + x(2)^2*x(7)^2 - 2*x(2)^2*x(7)*x(8) + x(2)^2*x(8)^2 + x(2)^2*x(11)^2 - 2*x(2)^2*x(11)*x(12) + x(2)^2*x(12)^2 + 2*x(2)*x(3)*x(5)*x(7) - 2*x(2)*x(3)*x(5)*x(8) - 2*x(2)*x(3)*x(6)*x(7) + 2*x(2)*x(3)*x(6)*x(8) + 2*x(2)*x(3)*x(9)*x(11) - 2*x(2)*x(3)*x(9)*x(12) - 2*x(2)*x(3)*x(10)*x(11) + 2*x(2)*x(3)*x(10)*x(12) - 2*x(2)*x(4)*x(5)*x(7) + 2*x(2)*x(4)*x(5)*x(8) + 2*x(2)*x(4)*x(6)*x(7) - 2*x(2)*x(4)*x(6)*x(8) - 2*x(2)*x(4)*x(9)*x(11) + 2*x(2)*x(4)*x(9)*x(12) + 2*x(2)*x(4)*x(10)*x(11) - 2*x(2)*x(4)*x(10)*x(12) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - 2*x(3)*x(4)*x(5)^2 + 4*x(3)*x(4)*x(5)*x(6) - 2*x(3)*x(4)*x(6)^2 - 2*x(3)*x(4)*x(9)^2 + 4*x(3)*x(4)*x(9)*x(10) - 2*x(3)*x(4)*x(10)^2 + x(4)^2*x(5)^2 - 2*x(4)^2*x(5)*x(6) + x(4)^2*x(6)^2 + x(4)^2*x(9)^2 - 2*x(4)^2*x(9)*x(10) + x(4)^2*x(10)^2 + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(6)*x(11)^2 + 4*x(5)*x(6)*x(11)*x(12) - 2*x(5)*x(6)*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + 2*x(5)*x(7)*x(10)*x(11) - 2*x(5)*x(7)*x(10)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - 2*x(5)*x(8)*x(10)*x(11) + 2*x(5)*x(8)*x(10)*x(12) + x(6)^2*x(11)^2 - 2*x(6)^2*x(11)*x(12) + x(6)^2*x(12)^2 + 2*x(6)*x(7)*x(9)*x(11) - 2*x(6)*x(7)*x(9)*x(12) - 2*x(6)*x(7)*x(10)*x(11) + 2*x(6)*x(7)*x(10)*x(12) - 2*x(6)*x(8)*x(9)*x(11) + 2*x(6)*x(8)*x(9)*x(12) + 2*x(6)*x(8)*x(10)*x(11) - 2*x(6)*x(8)*x(10)*x(12) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - 2*x(7)*x(8)*x(9)^2 + 4*x(7)*x(8)*x(9)*x(10) - 2*x(7)*x(8)*x(10)^2 + x(8)^2*x(9)^2 - 2*x(8)^2*x(9)*x(10) + x(8)^2*x(10)^2))*(x(7) - x(8)))^2 + (x(9) - x(11) - ((x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + x(1)*x(3)*x(7)*x(8) + x(6)*x(1)*x(3)*x(7) - x(1)*x(3)*x(8)^2 - x(6)*x(1)*x(3)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + x(1)*x(3)*x(11)*x(12) + x(10)*x(1)*x(3)*x(11) - x(1)*x(3)*x(12)^2 - x(10)*x(1)*x(3)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - x(1)*x(4)*x(7)^2 + x(1)*x(4)*x(7)*x(8) - x(6)*x(1)*x(4)*x(7) + x(6)*x(1)*x(4)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - x(1)*x(4)*x(11)^2 + x(1)*x(4)*x(11)*x(12) - x(10)*x(1)*x(4)*x(11) + x(10)*x(1)*x(4)*x(12) - x(2)*x(1)*x(7)^2 + 2*x(2)*x(1)*x(7)*x(8) - x(2)*x(1)*x(8)^2 - x(2)*x(1)*x(11)^2 + 2*x(2)*x(1)*x(11)*x(12) - x(2)*x(1)*x(12)^2 + x(3)^2*x(5)^2 - x(3)^2*x(5)*x(8) - x(6)*x(3)^2*x(5) + x(6)*x(3)^2*x(8) + x(3)^2*x(9)^2 - x(3)^2*x(9)*x(12) - x(10)*x(3)^2*x(9) + x(10)*x(3)^2*x(12) - 2*x(3)*x(4)*x(5)^2 + x(3)*x(4)*x(5)*x(7) + x(3)*x(4)*x(5)*x(8) + 2*x(6)*x(3)*x(4)*x(5) - x(6)*x(3)*x(4)*x(7) - x(6)*x(3)*x(4)*x(8) - 2*x(3)*x(4)*x(9)^2 + x(3)*x(4)*x(9)*x(11) + x(3)*x(4)*x(9)*x(12) + 2*x(10)*x(3)*x(4)*x(9) - x(10)*x(3)*x(4)*x(11) - x(10)*x(3)*x(4)*x(12) + x(2)*x(3)*x(5)*x(7) - x(2)*x(3)*x(5)*x(8) - x(2)*x(3)*x(7)*x(8) + x(2)*x(3)*x(8)^2 + x(2)*x(3)*x(9)*x(11) - x(2)*x(3)*x(9)*x(12) - x(2)*x(3)*x(11)*x(12) + x(2)*x(3)*x(12)^2 + x(4)^2*x(5)^2 - x(4)^2*x(5)*x(7) - x(6)*x(4)^2*x(5) + x(6)*x(4)^2*x(7) + x(4)^2*x(9)^2 - x(4)^2*x(9)*x(11) - x(10)*x(4)^2*x(9) + x(10)*x(4)^2*x(11) - x(2)*x(4)*x(5)*x(7) + x(2)*x(4)*x(5)*x(8) + x(2)*x(4)*x(7)^2 - x(2)*x(4)*x(7)*x(8) - x(2)*x(4)*x(9)*x(11) + x(2)*x(4)*x(9)*x(12) + x(2)*x(4)*x(11)^2 - x(2)*x(4)*x(11)*x(12) + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + x(5)*x(7)*x(11)*x(12) + x(10)*x(5)*x(7)*x(11) - x(5)*x(7)*x(12)^2 - x(10)*x(5)*x(7)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - x(5)*x(8)*x(11)^2 + x(5)*x(8)*x(11)*x(12) - x(10)*x(5)*x(8)*x(11) + x(10)*x(5)*x(8)*x(12) - x(6)*x(5)*x(11)^2 + 2*x(6)*x(5)*x(11)*x(12) - x(6)*x(5)*x(12)^2 + x(7)^2*x(9)^2 - x(7)^2*x(9)*x(12) - x(10)*x(7)^2*x(9) + x(10)*x(7)^2*x(12) - 2*x(7)*x(8)*x(9)^2 + x(7)*x(8)*x(9)*x(11) + x(7)*x(8)*x(9)*x(12) + 2*x(10)*x(7)*x(8)*x(9) - x(10)*x(7)*x(8)*x(11) - x(10)*x(7)*x(8)*x(12) + x(6)*x(7)*x(9)*x(11) - x(6)*x(7)*x(9)*x(12) - x(6)*x(7)*x(11)*x(12) + x(6)*x(7)*x(12)^2 + x(8)^2*x(9)^2 - x(8)^2*x(9)*x(11) - x(10)*x(8)^2*x(9) + x(10)*x(8)^2*x(11) - x(6)*x(8)*x(9)*x(11) + x(6)*x(8)*x(9)*x(12) + x(6)*x(8)*x(11)^2 - x(6)*x(8)*x(11)*x(12))/(x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(2)*x(7)^2 + 4*x(1)*x(2)*x(7)*x(8) - 2*x(1)*x(2)*x(8)^2 - 2*x(1)*x(2)*x(11)^2 + 4*x(1)*x(2)*x(11)*x(12) - 2*x(1)*x(2)*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + 2*x(1)*x(3)*x(6)*x(7) - 2*x(1)*x(3)*x(6)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + 2*x(1)*x(3)*x(10)*x(11) - 2*x(1)*x(3)*x(10)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - 2*x(1)*x(4)*x(6)*x(7) + 2*x(1)*x(4)*x(6)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - 2*x(1)*x(4)*x(10)*x(11) + 2*x(1)*x(4)*x(10)*x(12) + x(2)^2*x(7)^2 - 2*x(2)^2*x(7)*x(8) + x(2)^2*x(8)^2 + x(2)^2*x(11)^2 - 2*x(2)^2*x(11)*x(12) + x(2)^2*x(12)^2 + 2*x(2)*x(3)*x(5)*x(7) - 2*x(2)*x(3)*x(5)*x(8) - 2*x(2)*x(3)*x(6)*x(7) + 2*x(2)*x(3)*x(6)*x(8) + 2*x(2)*x(3)*x(9)*x(11) - 2*x(2)*x(3)*x(9)*x(12) - 2*x(2)*x(3)*x(10)*x(11) + 2*x(2)*x(3)*x(10)*x(12) - 2*x(2)*x(4)*x(5)*x(7) + 2*x(2)*x(4)*x(5)*x(8) + 2*x(2)*x(4)*x(6)*x(7) - 2*x(2)*x(4)*x(6)*x(8) - 2*x(2)*x(4)*x(9)*x(11) + 2*x(2)*x(4)*x(9)*x(12) + 2*x(2)*x(4)*x(10)*x(11) - 2*x(2)*x(4)*x(10)*x(12) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - 2*x(3)*x(4)*x(5)^2 + 4*x(3)*x(4)*x(5)*x(6) - 2*x(3)*x(4)*x(6)^2 - 2*x(3)*x(4)*x(9)^2 + 4*x(3)*x(4)*x(9)*x(10) - 2*x(3)*x(4)*x(10)^2 + x(4)^2*x(5)^2 - 2*x(4)^2*x(5)*x(6) + x(4)^2*x(6)^2 + x(4)^2*x(9)^2 - 2*x(4)^2*x(9)*x(10) + x(4)^2*x(10)^2 + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(6)*x(11)^2 + 4*x(5)*x(6)*x(11)*x(12) - 2*x(5)*x(6)*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + 2*x(5)*x(7)*x(10)*x(11) - 2*x(5)*x(7)*x(10)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - 2*x(5)*x(8)*x(10)*x(11) + 2*x(5)*x(8)*x(10)*x(12) + x(6)^2*x(11)^2 - 2*x(6)^2*x(11)*x(12) + x(6)^2*x(12)^2 + 2*x(6)*x(7)*x(9)*x(11) - 2*x(6)*x(7)*x(9)*x(12) - 2*x(6)*x(7)*x(10)*x(11) + 2*x(6)*x(7)*x(10)*x(12) - 2*x(6)*x(8)*x(9)*x(11) + 2*x(6)*x(8)*x(9)*x(12) + 2*x(6)*x(8)*x(10)*x(11) - 2*x(6)*x(8)*x(10)*x(12) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - 2*x(7)*x(8)*x(9)^2 + 4*x(7)*x(8)*x(9)*x(10) - 2*x(7)*x(8)*x(10)^2 + x(8)^2*x(9)^2 - 2*x(8)^2*x(9)*x(10) + x(8)^2*x(10)^2))*(x(9) - x(10)) + ((x(8)*x(1)^2*x(6) - x(1)^2*x(6)*x(7) + x(1)^2*x(7)^2 - x(8)*x(1)^2*x(7) - x(1)^2*x(10)*x(11) + x(12)*x(1)^2*x(10) + x(1)^2*x(11)^2 - x(12)*x(1)^2*x(11) + x(1)*x(2)*x(5)*x(7) - x(8)*x(1)*x(2)*x(5) + x(1)*x(2)*x(6)*x(7) - x(8)*x(1)*x(2)*x(6) - 2*x(1)*x(2)*x(7)^2 + 2*x(8)*x(1)*x(2)*x(7) + x(1)*x(2)*x(9)*x(11) - x(12)*x(1)*x(2)*x(9) + x(1)*x(2)*x(10)*x(11) - x(12)*x(1)*x(2)*x(10) - 2*x(1)*x(2)*x(11)^2 + 2*x(12)*x(1)*x(2)*x(11) + x(1)*x(3)*x(5)*x(6) - 2*x(1)*x(3)*x(5)*x(7) + x(8)*x(1)*x(3)*x(5) - x(1)*x(3)*x(6)^2 + 2*x(1)*x(3)*x(6)*x(7) - x(8)*x(1)*x(3)*x(6) + x(1)*x(3)*x(9)*x(10) - 2*x(1)*x(3)*x(9)*x(11) + x(12)*x(1)*x(3)*x(9) - x(1)*x(3)*x(10)^2 + 2*x(1)*x(3)*x(10)*x(11) - x(12)*x(1)*x(3)*x(10) - x(4)*x(1)*x(5)*x(6) + x(4)*x(1)*x(5)*x(7) + x(4)*x(1)*x(6)^2 - x(4)*x(1)*x(6)*x(7) - x(4)*x(1)*x(9)*x(10) + x(4)*x(1)*x(9)*x(11) + x(4)*x(1)*x(10)^2 - x(4)*x(1)*x(10)*x(11) - x(2)^2*x(5)*x(7) + x(8)*x(2)^2*x(5) + x(2)^2*x(7)^2 - x(8)*x(2)^2*x(7) - x(2)^2*x(9)*x(11) + x(12)*x(2)^2*x(9) + x(2)^2*x(11)^2 - x(12)*x(2)^2*x(11) - x(2)*x(3)*x(5)^2 + x(2)*x(3)*x(5)*x(6) + 2*x(2)*x(3)*x(5)*x(7) - x(8)*x(2)*x(3)*x(5) - 2*x(2)*x(3)*x(6)*x(7) + x(8)*x(2)*x(3)*x(6) - x(2)*x(3)*x(9)^2 + x(2)*x(3)*x(9)*x(10) + 2*x(2)*x(3)*x(9)*x(11) - x(12)*x(2)*x(3)*x(9) - 2*x(2)*x(3)*x(10)*x(11) + x(12)*x(2)*x(3)*x(10) + x(4)*x(2)*x(5)^2 - x(4)*x(2)*x(5)*x(6) - x(4)*x(2)*x(5)*x(7) + x(4)*x(2)*x(6)*x(7) + x(4)*x(2)*x(9)^2 - x(4)*x(2)*x(9)*x(10) - x(4)*x(2)*x(9)*x(11) + x(4)*x(2)*x(10)*x(11) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - x(4)*x(3)*x(5)^2 + 2*x(4)*x(3)*x(5)*x(6) - x(4)*x(3)*x(6)^2 - x(4)*x(3)*x(9)^2 + 2*x(4)*x(3)*x(9)*x(10) - x(4)*x(3)*x(10)^2 - x(5)^2*x(10)*x(11) + x(12)*x(5)^2*x(10) + x(5)^2*x(11)^2 - x(12)*x(5)^2*x(11) + x(5)*x(6)*x(9)*x(11) - x(12)*x(5)*x(6)*x(9) + x(5)*x(6)*x(10)*x(11) - x(12)*x(5)*x(6)*x(10) - 2*x(5)*x(6)*x(11)^2 + 2*x(12)*x(5)*x(6)*x(11) + x(5)*x(7)*x(9)*x(10) - 2*x(5)*x(7)*x(9)*x(11) + x(12)*x(5)*x(7)*x(9) - x(5)*x(7)*x(10)^2 + 2*x(5)*x(7)*x(10)*x(11) - x(12)*x(5)*x(7)*x(10) - x(8)*x(5)*x(9)*x(10) + x(8)*x(5)*x(9)*x(11) + x(8)*x(5)*x(10)^2 - x(8)*x(5)*x(10)*x(11) - x(6)^2*x(9)*x(11) + x(12)*x(6)^2*x(9) + x(6)^2*x(11)^2 - x(12)*x(6)^2*x(11) - x(6)*x(7)*x(9)^2 + x(6)*x(7)*x(9)*x(10) + 2*x(6)*x(7)*x(9)*x(11) - x(12)*x(6)*x(7)*x(9) - 2*x(6)*x(7)*x(10)*x(11) + x(12)*x(6)*x(7)*x(10) + x(8)*x(6)*x(9)^2 - x(8)*x(6)*x(9)*x(10) - x(8)*x(6)*x(9)*x(11) + x(8)*x(6)*x(10)*x(11) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - x(8)*x(7)*x(9)^2 + 2*x(8)*x(7)*x(9)*x(10) - x(8)*x(7)*x(10)^2)/(x(1)^2*x(7)^2 - 2*x(1)^2*x(7)*x(8) + x(1)^2*x(8)^2 + x(1)^2*x(11)^2 - 2*x(1)^2*x(11)*x(12) + x(1)^2*x(12)^2 - 2*x(1)*x(2)*x(7)^2 + 4*x(1)*x(2)*x(7)*x(8) - 2*x(1)*x(2)*x(8)^2 - 2*x(1)*x(2)*x(11)^2 + 4*x(1)*x(2)*x(11)*x(12) - 2*x(1)*x(2)*x(12)^2 - 2*x(1)*x(3)*x(5)*x(7) + 2*x(1)*x(3)*x(5)*x(8) + 2*x(1)*x(3)*x(6)*x(7) - 2*x(1)*x(3)*x(6)*x(8) - 2*x(1)*x(3)*x(9)*x(11) + 2*x(1)*x(3)*x(9)*x(12) + 2*x(1)*x(3)*x(10)*x(11) - 2*x(1)*x(3)*x(10)*x(12) + 2*x(1)*x(4)*x(5)*x(7) - 2*x(1)*x(4)*x(5)*x(8) - 2*x(1)*x(4)*x(6)*x(7) + 2*x(1)*x(4)*x(6)*x(8) + 2*x(1)*x(4)*x(9)*x(11) - 2*x(1)*x(4)*x(9)*x(12) - 2*x(1)*x(4)*x(10)*x(11) + 2*x(1)*x(4)*x(10)*x(12) + x(2)^2*x(7)^2 - 2*x(2)^2*x(7)*x(8) + x(2)^2*x(8)^2 + x(2)^2*x(11)^2 - 2*x(2)^2*x(11)*x(12) + x(2)^2*x(12)^2 + 2*x(2)*x(3)*x(5)*x(7) - 2*x(2)*x(3)*x(5)*x(8) - 2*x(2)*x(3)*x(6)*x(7) + 2*x(2)*x(3)*x(6)*x(8) + 2*x(2)*x(3)*x(9)*x(11) - 2*x(2)*x(3)*x(9)*x(12) - 2*x(2)*x(3)*x(10)*x(11) + 2*x(2)*x(3)*x(10)*x(12) - 2*x(2)*x(4)*x(5)*x(7) + 2*x(2)*x(4)*x(5)*x(8) + 2*x(2)*x(4)*x(6)*x(7) - 2*x(2)*x(4)*x(6)*x(8) - 2*x(2)*x(4)*x(9)*x(11) + 2*x(2)*x(4)*x(9)*x(12) + 2*x(2)*x(4)*x(10)*x(11) - 2*x(2)*x(4)*x(10)*x(12) + x(3)^2*x(5)^2 - 2*x(3)^2*x(5)*x(6) + x(3)^2*x(6)^2 + x(3)^2*x(9)^2 - 2*x(3)^2*x(9)*x(10) + x(3)^2*x(10)^2 - 2*x(3)*x(4)*x(5)^2 + 4*x(3)*x(4)*x(5)*x(6) - 2*x(3)*x(4)*x(6)^2 - 2*x(3)*x(4)*x(9)^2 + 4*x(3)*x(4)*x(9)*x(10) - 2*x(3)*x(4)*x(10)^2 + x(4)^2*x(5)^2 - 2*x(4)^2*x(5)*x(6) + x(4)^2*x(6)^2 + x(4)^2*x(9)^2 - 2*x(4)^2*x(9)*x(10) + x(4)^2*x(10)^2 + x(5)^2*x(11)^2 - 2*x(5)^2*x(11)*x(12) + x(5)^2*x(12)^2 - 2*x(5)*x(6)*x(11)^2 + 4*x(5)*x(6)*x(11)*x(12) - 2*x(5)*x(6)*x(12)^2 - 2*x(5)*x(7)*x(9)*x(11) + 2*x(5)*x(7)*x(9)*x(12) + 2*x(5)*x(7)*x(10)*x(11) - 2*x(5)*x(7)*x(10)*x(12) + 2*x(5)*x(8)*x(9)*x(11) - 2*x(5)*x(8)*x(9)*x(12) - 2*x(5)*x(8)*x(10)*x(11) + 2*x(5)*x(8)*x(10)*x(12) + x(6)^2*x(11)^2 - 2*x(6)^2*x(11)*x(12) + x(6)^2*x(12)^2 + 2*x(6)*x(7)*x(9)*x(11) - 2*x(6)*x(7)*x(9)*x(12) - 2*x(6)*x(7)*x(10)*x(11) + 2*x(6)*x(7)*x(10)*x(12) - 2*x(6)*x(8)*x(9)*x(11) + 2*x(6)*x(8)*x(9)*x(12) + 2*x(6)*x(8)*x(10)*x(11) - 2*x(6)*x(8)*x(10)*x(12) + x(7)^2*x(9)^2 - 2*x(7)^2*x(9)*x(10) + x(7)^2*x(10)^2 - 2*x(7)*x(8)*x(9)^2 + 4*x(7)*x(8)*x(9)*x(10) - 2*x(7)*x(8)*x(10)^2 + x(8)^2*x(9)^2 - 2*x(8)^2*x(9)*x(10) + x(8)^2*x(10)^2))*(x(11) - x(12)))^2;
        x0 = [x1 x2 x3 x4 y1 y2 y3 y4 z1 z2 z3 z4];
        grad = grad_simple(fun,x0,delta);
        dfdx(index,p1) = grad(1);
        dfdx(index,p2) = grad(2);
        dfdx(index,p3) = grad(3);
        dfdx(index,p4) = grad(4);
        dfdx(index,length(pos)+p1) = grad(5);
        dfdx(index,length(pos)+p2) = grad(6);
        dfdx(index,length(pos)+p3) = grad(7);
        dfdx(index,length(pos)+p4) = grad(8);
        dfdx(index,2*length(pos)+p1) = grad(9);
        dfdx(index,2*length(pos)+p2) = grad(10);
        dfdx(index,2*length(pos)+p3) = grad(11);
        dfdx(index,2*length(pos)+p4) = grad(12);
    end
    
    index = index + 1;
    
    % %%%%% point to line collision check 1 %%%%%%
    % node number
    p1 = elist(clist(k,1),1);
    p2 = elist(clist(k,2),1);
    p3 = elist(clist(k,2),2);
    
    x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
    x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
    x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        constraint(index) = dist2 - d_min^2;
    else
        constraint(index) = inf;
    end
    
    % gradient calculation
    if CalDiff == 1
        fun = @(x) (x(1) - x(2) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(2) - x(3)))^2 + (x(4) - x(5) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(5) - x(6)))^2 + (x(7) - x(8) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(8) - x(9)))^2;
        x0 = [x1 x2 x3 y1 y2 y3 z1 z2 z3];
        grad = grad_simple(fun,x0,delta);
        dfdx(index,p1) = grad(1);
        dfdx(index,p2) = grad(2);
        dfdx(index,p3) = grad(3);
        dfdx(index,length(pos)+p1) = grad(4);
        dfdx(index,length(pos)+p2) = grad(5);
        dfdx(index,length(pos)+p3) = grad(6);
        dfdx(index,2*length(pos)+p1) = grad(7);
        dfdx(index,2*length(pos)+p2) = grad(8);
        dfdx(index,2*length(pos)+p3) = grad(9);
    end
    
    index = index + 1;
    
    % %%%%% point to line collision check 2 %%%%%%
    % node number
    p1 = elist(clist(k,1),2);
    p2 = elist(clist(k,2),1);
    p3 = elist(clist(k,2),2);
    
    x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
    x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
    x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        constraint(index) = dist2 - d_min^2;
    else
        constraint(index) = inf;
    end
    
    % gradient calculation
    if CalDiff == 1
        fun = @(x) (x(1) - x(2) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(2) - x(3)))^2 + (x(4) - x(5) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(5) - x(6)))^2 + (x(7) - x(8) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(8) - x(9)))^2;
        x0 = [x1 x2 x3 y1 y2 y3 z1 z2 z3];
        grad = grad_simple(fun,x0,delta);
        dfdx(index,p1) = grad(1);
        dfdx(index,p2) = grad(2);
        dfdx(index,p3) = grad(3);
        dfdx(index,length(pos)+p1) = grad(4);
        dfdx(index,length(pos)+p2) = grad(5);
        dfdx(index,length(pos)+p3) = grad(6);
        dfdx(index,2*length(pos)+p1) = grad(7);
        dfdx(index,2*length(pos)+p2) = grad(8);
        dfdx(index,2*length(pos)+p3) = grad(9);
    end
    
    index = index + 1;
    
    % %%%%% point to line collision check 3 %%%%%%
    % node number
    p1 = elist(clist(k,2),1);
    p2 = elist(clist(k,1),1);
    p3 = elist(clist(k,1),2);
    
    x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
    x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
    x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        constraint(index) = dist2 - d_min^2;
    else
        constraint(index) = inf;
    end
    
    % gradient calculation
    if CalDiff == 1
        fun = @(x) (x(1) - x(2) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(2) - x(3)))^2 + (x(4) - x(5) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(5) - x(6)))^2 + (x(7) - x(8) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(8) - x(9)))^2;
        x0 = [x1 x2 x3 y1 y2 y3 z1 z2 z3];
        grad = grad_simple(fun,x0,delta);
        dfdx(index,p1) = grad(1);
        dfdx(index,p2) = grad(2);
        dfdx(index,p3) = grad(3);
        dfdx(index,length(pos)+p1) = grad(4);
        dfdx(index,length(pos)+p2) = grad(5);
        dfdx(index,length(pos)+p3) = grad(6);
        dfdx(index,2*length(pos)+p1) = grad(7);
        dfdx(index,2*length(pos)+p2) = grad(8);
        dfdx(index,2*length(pos)+p3) = grad(9);
    end
    
    index = index + 1;
    
    % %%%%% point to line collision check 4 %%%%%%
    % node number
    p1 = elist(clist(k,2),2);
    p2 = elist(clist(k,1),1);
    p3 = elist(clist(k,1),2);
    
    x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
    x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
    x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        constraint(index) = dist2 - d_min^2;
    else
        constraint(index) = inf;
    end
    
    % gradient calculation
    if CalDiff == 1
        fun = @(x) (x(1) - x(2) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(2) - x(3)))^2 + (x(4) - x(5) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(5) - x(6)))^2 + (x(7) - x(8) + (-(2*(x(1) - x(2))*(x(2) - x(3)) + 2*(x(4) - x(5))*(x(5) - x(6)) + 2*(x(7) - x(8))*(x(8) - x(9)))/(2*(x(2) - x(3))^2 + 2*(x(5) - x(6))^2 + 2*(x(8) - x(9))^2))*(x(8) - x(9)))^2;
        x0 = [x1 x2 x3 y1 y2 y3 z1 z2 z3];
        grad = grad_simple(fun,x0,delta);
        dfdx(index,p1) = grad(1);
        dfdx(index,p2) = grad(2);
        dfdx(index,p3) = grad(3);
        dfdx(index,length(pos)+p1) = grad(4);
        dfdx(index,length(pos)+p2) = grad(5);
        dfdx(index,length(pos)+p3) = grad(6);
        dfdx(index,2*length(pos)+p1) = grad(7);
        dfdx(index,2*length(pos)+p2) = grad(8);
        dfdx(index,2*length(pos)+p3) = grad(9);
    end
    
    index = index + 1;

end % for end

constraint = constraint';

end % end if d_min > 0 


