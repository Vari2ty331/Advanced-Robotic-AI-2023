function [ pass ] = check_constraints_truss( truss, x )
%check_constraints tests to see if a configuration satisfies the
%constraints
%   truss: a struct with the following fields:
%       elist: edge list
%       adj: node adjacency matrix
%       edge_adj: edge adjacency matrix
%   while the latter two fields could be computed from the edge list,
%   check_constraints is called many times for a fixed truss, so
%   recomputing them every time would be a waste
%
%   x is the stacked column vector of node locations

pos = reshape(x, 3, []);

pass = length_test(truss.elist, pos) && ...
    angle_constraint(truss,pos);
 
% 
% if isfield(truss, 'ntype')
%     if size(truss.pos,2) == 10
%         pass = angle_constraint(truss.adj, pos, truss.ntype) && collision_test_UR(truss.elist, truss.edge_adj, pos) && com_test(pos) && length_test2(truss.elist, pos) && singularity_avoidance(truss.elist, pos);% && com_test(pos) && (sum(pos(3,:) >= -1e-04) == size(pos,2)); %&& collision_test(truss.elist, truss.edge_adj, pos);
% %         pass = true;
%     else
%         pass = angle_constraint(truss.adj, pos, truss.ntype)&& collision_test(truss.elist, truss.edge_adj, pos) && com_test(pos) && length_test(truss.elist, pos) && singularity_avoidance(truss.elist, pos)&& (sum(pos(3,:) >= -1e-04) == size(pos,2));% && com_test(pos) && (sum(pos(3,:) >= -1e-04) == size(pos,2)); %&& collision_test(truss.elist, truss.edge_adj, pos);
%     end
% else
%     pass = angle_constraint(truss.adj, pos)&& collision_test(truss.elist, truss.edge_adj, pos) && com_test(pos) && length_test(truss.elist, pos) && singularity_avoidance(truss.elist, pos);% && com_test(pos) && (sum(pos(3,:) >= -1e-04) == size(pos,2)); %&& collision_test(truss.elist, truss.edge_adj, pos);
% end

end

function pass = angle_constraint(n, pos)

pos = pos';

angle = [];
adj = adj_gen(n.elist);
index = 1;
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

            angle(index,1) = vector_angle(v1,v2);
            index = index + 1;
        end
    end
end

pass = min(angle) >= 32/180*pi;

end


function pass = singularity_avoidance(elist, pos)
    min_cond = 0;%0.01; 
    S = svd(length_jacobian(elist, pos));
    pass = min(S)/max(S) > min_cond;
    if rank(length_jacobian(elist, pos)) >= 21
        pass = true;
    end
end

function pass = collision_test(elist, edge_adjmatrix, pos)
    min_coll_dist = 0.15;
    dist_result = collision_dist(elist, edge_adjmatrix, pos);
    [min_value, min_index] = min(dist_result(dist_result > 1e-4));
    pass = min_value > min_coll_dist;
end

function pass = collision_test_UR(elist, edge_adjmatrix, pos)
    min_coll_dist = 0.15;
    [edge_to_remove, ~] = find(elist == 10);
    elist(edge_to_remove,:) = [];
    edge_adjmatrix = edgelist_to_edge_adjmatrix(elist);
    dist_result = collision_dist(elist, edge_adjmatrix, pos);
    [min_value, min_index] = min(dist_result(dist_result > 1e-4));
    pass = min_value > min_coll_dist;
end

function pass = length_test(elist, pos)
%     pass = true;
    l = lengths(elist, pos);
    pass = 0.7 <= min(l) && max(l) <= 1.4;
end

function pass = length_test2(elist, pos)
%     pass = true;
    [edge_to_remove, ~] = find(elist == 10);
    elist(edge_to_remove,:) = [];
    l = lengths(elist, pos);
    pass = 0.6 <= min(l) && max(l) <= 1.986;
end

% function pass = length_test_member_move(elist, pos)
% %     pass = true;
%     l = lengths(elist, pos);
%     pass = 0.5 <= min(l) && max(l) <= 1.8;
% end

function pass = com_test(pos)
    com=mean(pos,2);
%     groundnode_index=find(pos(3,:)==-1);
    groundnode_index=find(pos(3,:) < 1e-5);
    xv=pos(1,groundnode_index);
    yv=pos(2,groundnode_index);
    if length(groundnode_index)>2
        k=convhull(xv,yv);
        pass=inpolygon(com(1),com(2),xv(k),yv(k));
    else
        pass=false;
    end
end
