function [ pass ] = check_constraints_new_sh( truss, x )
%% ======================================================== %% 
%   truss: a struct with the following fields:
%       elist: edge list
%       adj: node adjacency matrix
%       edge_adj: edge adjacency matrix
%       new_ntype: Number of edges x 2, Corresponds to elist, Passive (0), Master (1)
%   while the latter two fields could be computed from the edge list,
%   check_constraints is called many times for a fixed truss, so
%   recomputing them every time would be a waste
%
%   x is the stacked column vector of node locations
%   ground_nodes is arrays of node numbers that are attached to the ground
%   mass is the mass of nodes and members
%       mass.node_m: node mass
%       mass.member_m: member mass
%% ======================================================== %% 

%% ======================================================== %% 

% Min angle: p. node / p. node : 21.2, m. node / p. node : 31.2
% Min length : 1.0, Max length : 2.5 // 2.0
% Max comp. force = 1000, Max tens force = 89
% Min distance btw. members = 0.16

%% ======================================================== %% 

pos = reshape(x, 3, []);

global IM_num;

pass = force_test(truss,pos) && length_test(truss.elist, pos, truss.exlen) && singularity_avoidance(truss.elist, pos, IM_num) ...
    && collision_test(truss.elist, truss.edge_adj, pos, IM_num)  && dihedral_angle_test(truss.link_connection,truss.elist,pos)...
    && com_test_new(truss.elist,pos,truss.mass, 0.05) && min_angle_constraint_new(truss.edge_adj, truss.elist, pos, truss.new_ntype) && max_angle_test(truss.link_connection,truss.elist,pos);
% 
% ========================================================================
if ~force_test(truss,pos)
    disp('Force test false');
end
if ~min_angle_constraint_new(truss.edge_adj, truss.elist, pos, truss.new_ntype)
    disp('Minimum angle constraint test false');
end
if ~max_angle_test(truss.link_connection, truss.elist, pos)
    disp('Maximum angle constraint test false');
end
if ~dihedral_angle_test(truss.link_connection, truss.elist, pos)
    disp('Dihedral angle constraint test false');
end
if ~collision_test(truss.elist, truss.edge_adj, pos, IM_num)
    disp('Collision test false');
end
if ~com_test_new(truss.elist,pos,truss.mass, 0.05)
    disp('COM test false');
end
if ~length_test(truss.elist, pos, truss.exlen)
    disp('Length test false');
end
if ~singularity_avoidance(truss.elist, pos,IM_num)
    disp('Singularity test false');
end
% ========================================================================
end

function pass = min_angle_constraint_new(edge_adjmatrix, elist, pos, new_ntype)
% Min angle: pp : 22, pm : 32, pa : 25, aa : 28

pp_angle = 25 + 1; pm_angle = 32 + 1; am_angle = 32 + 1; pa_angle = 25 + 1; aa_angle = 25 + 1;
% p:passive, m:master, a:active

pass = true;

for i = 1 : size(edge_adjmatrix,2)-1
    for k = i+1 : size(edge_adjmatrix,2)
        if edge_adjmatrix(i,k) == 1 % edge i and edge k are adjacant
            edge1 = elist(i,:);
            edge2 = elist(k,:);
            
            times = sum([edge1 edge2]==[edge1 edge2]'); % 1 or 2
            for n = 1:length(times)
                if(times(n) == 2)
                    break;
                end
            end
            connecting_node = edge1(n);
            
            vec1 = pos(:,edge1(edge1~=connecting_node)) - pos(:,connecting_node);
            vec2 = pos(:,edge2(edge2~=connecting_node)) - pos(:,connecting_node);
            angle_deg = rad2deg(acos(dot(vec1,vec2) / (norm(vec1)*norm(vec2))));
            
            ntype_of_edge1 = new_ntype(i,:);
            ntype_of_edge2 = new_ntype(k,:);            
            n1 = ntype_of_edge1(edge1==connecting_node); % node type of connecting node of edge1
            n2 = ntype_of_edge2(edge2==connecting_node); % node type of connecting node of edge2
            
            if (n1 + n2 == 0) % passive node / passive node
                if (angle_deg < pp_angle)
                    pass = false;
                    return;
                end
            elseif (n1 + n2 == 1) % passive node / master node
                if (angle_deg < pm_angle)
                    pass = false;
                    return;
                end
            elseif (n1 + n2 == 3) % passive / active
                if (angle_deg < pa_angle)
                    pass = false;
                    return;
                end
            elseif (n1 + n2 == 4) % active / master
                if (angle_deg < am_angle)
                    pass = false;
                    return;
                end
            elseif (n1 + n2 == 6) % active / active
                if (angle_deg < aa_angle)
                    pass = false;
                    return;
                end
            else
                disp('ERROR: CHECK NTYPE PLEASE')
                pass = false;
                return;
            end
        end
    end
end

end

function pass = max_angle_test(link_connection,elist,pos)
    pass = 1;
    maximum_angle = 155 - 1;
    for i = 1:size(link_connection,1)
        for j = 1:size(link_connection{i},1)
            edge1 = elist(link_connection{i}(j,1),:);
            edge2 = elist(link_connection{i}(j,2),:);
            
            p_0 = pos(:,i); % position of intersecting node
            p_1 = pos(:,edge1(find(edge1 ~= i))); % position of non-intersecting node 1 
            p_2 = pos(:,edge2(find(edge2 ~= i))); % position of non-intersecting node 2
            
            vec1 = p_1 - p_0;
            vec2 = p_2 - p_0;
            angle_deg = rad2deg(acos(dot(vec1,vec2) / (norm(vec1)*norm(vec2))));
            if angle_deg >= maximum_angle
                pass = 0;
                break;
            end
        end
    end
end

function pass = dihedral_angle_test(link_connection,elist,pos)
    pass = 1;
    min_dihedral_angle = 45 + 1;
    dihedral_angles = dihedral_angle(link_connection,elist,pos);
    if ~all(dihedral_angles >= min_dihedral_angle)
        pass = 0;
    end
end

function pass = singularity_avoidance(elist, pos, IM_num)

min_cond = 0.01; %0.02;
num_of_nodes = IM_num - 1;
S = svd(length_jacobian(elist, pos));
pass = min(S)/max(S) > min_cond;
if rank(length_jacobian(elist, pos)) >= 3*num_of_nodes-6
    pass = true;
end

end

function pass = collision_test(elist, edge_adjmatrix, pos, IM_num)
% Min distance btw. members = 0.114

min_coll_dist = 0.114 + 0.02;

% Imaginary node makes error
if size(pos,2) == IM_num
    [edge_to_remove, ~] = find(elist == size(pos,2));
    elist(edge_to_remove,:) = [];    
    edge_adjmatrix = edgelist_to_edge_adjmatrix(elist);
end

dist_result = collision_dist(elist, edge_adjmatrix, pos);
[min_value, ~] = min(dist_result(dist_result > 1e-4));
pass = min_value > min_coll_dist;

end

function pass = length_test(elist, pos, exlen)
% Min length : 1.0, Max length : 2.5 // 2.0

% % Imaginary node makes error
% if size(pos,2) == IM_num
%     [edge_to_remove, ~] = find(elist == size(pos,2));
%     elist(edge_to_remove,:) = [];
% end

L = lengths(elist, pos) - exlen;

pass = 0.7 + 0.01 <= min(L) && max(L) <= 1.4 - 0.01;


% pass = 1.1 <= min(l) && max(l) <= 2.3 ;
% pass = 1.0 <= min(l) && max(l) <= 2.26 ;

end

function pass = com_test_new(elist, pos, mass, stb_margin)

floor_level = 0;

% All nodes above ground
if (sum(pos(3,:) < floor_level) > 0)
    pass = false;
    return;
end

if (length(mass.node_m) ~= length(pos)) 
    disp('ERROR: DIMENSIONS OF MASS AND POS NOT SAME')
end

mempos = zeros(2, length(elist)); % mempos : Center position of members
for i = 1:length(elist)
    mempos(:,i) = (pos(1:2,elist(i,1)) + pos(1:2,elist(i,2)))./2;
end

total_mass = sum(mass.node_m) + sum(mass.member_m);
com_projected = (sum(mass.node_m .* pos(1:2,:)',1) + sum(mass.member_m .* mempos',1)) / total_mass; % COM projected to ground

groundnode_index = (pos(3,:) - floor_level) < 1e-5;
groundnode_pos_x = pos(1,groundnode_index);
groundnode_pos_y = pos(2,groundnode_index);
idx_convhull = convhull(groundnode_pos_x,groundnode_pos_y); % indices of convex hull boundary nodes ex) 1 2 3 1

pos_g = [groundnode_pos_x;groundnode_pos_y];

convhull_pos = pos_g(1:2,idx_convhull(1:end-1));
convhull_center = mean(convhull_pos,2);

margin_hull_pos = zeros(size(convhull_pos));
for i = 1:size(margin_hull_pos,2)
    margin_hull_pos(:,i) = stb_margin * convhull_center + (1-stb_margin) * convhull_pos(:,i);
end

if size(margin_hull_pos,2) > 2
    pass = inpolygon(com_projected(1),com_projected(2),margin_hull_pos(1,:),margin_hull_pos(2,:));
else
    pass = false;
end

end

function pass = force_test(truss,pos)
% VTT Backup/PlanningWork/truss_fundamentals/truss analysis example
% Max comp. force = 1000, Max tens force = 89

% plus: compressive force / minus: tensile force
axial_force = truss_force_cal_plot(struct('pos',pos,'elist',truss.elist),truss.ground_node,truss.mass,false);

% RRT path smoothing -> may exceed the limit -> set tolerance
max_comp_force = 1000 - 5 ;
max_tens_force = 118 - 5;

pass = true;
for i = 1:length(axial_force)
    if axial_force(i) > 0   % compressive force
        if axial_force(i) > max_comp_force
            pass = false;
        end
    else                    % tensile force or zero
        if abs(axial_force(i)) > max_tens_force
            pass = false;
        end
    end
end

end
















