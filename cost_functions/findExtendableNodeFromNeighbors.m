function [nearest_node,new_foot] = findExtendableNodeFromNeighbors(tree,neighbor_nodes)
L_nom = 1.25;
step = 0.15;
query_node = neighbor_nodes;

end_center = tree.T(end).center_point;

query_node(end) = [];

query_bool = [];

for i = query_node'
query_base_foot_coords = tree.T(i).n;
query_center = tree.T(i).center_point;
new_distance = sqrt(sum((query_center - end_center).^2,2));
new_init_distances(find(query_node == i)) = tree.T(i).dist_init + new_distance;
query_lengths = sqrt(sum((query_base_foot_coords - end_center).^2,2));
if sum(query_lengths~=max(query_lengths)) < 2
    temp = query_lengths(query_lengths==max(query_lengths));
    temp2 = query_lengths(query_lengths~=max(query_lengths));
    query_lengths(1) = temp(1);
    query_lengths(2) = temp2;
    query_lengths(3) = temp(1)+1;
end
query_close_foot_coords = query_base_foot_coords(query_lengths~=max(query_lengths),:);
query_new_foot_coords = end_center*3 - query_close_foot_coords(1,:) - query_close_foot_coords(2,:);
query_bool =[query_bool sqrt(sum((query_close_foot_coords - query_new_foot_coords).^2,2))> L_nom-step & sqrt(sum((query_close_foot_coords - query_new_foot_coords).^2,2))<L_nom+step];
end
query_bool = all(query_bool);
new_init_distances = new_init_distances(query_bool);

% new_foot_coords = end_center*3 - close_foot_coords(1,:) - close_foot_coords(2,:);
% 
% query_node = query_node(sqrt(sum((close_foot_coords - new_foot_coords).^2,2))> L_nom-step & sqrt(sum((close_foot_coords - new_foot_coords).^2,2))<L_nom+step);
nearest_node = query_node(new_init_distances==min(new_init_distances));

if length(nearest_node) > 1
    nearest_node = max(nearest_node);
end

base_foot_coords = tree.T(nearest_node).n;
lengths = sqrt(sum((base_foot_coords - end_center).^2,2));

if sum(lengths~=max(lengths)) < 2
    temp = lengths(lengths==max(lengths));
    temp2 = lengths(lengths~=max(lengths));
    lengths(1) = temp(1);
    lengths(2) = temp2;
    lengths(3) = temp(1)+1;
end

close_foot_coords = base_foot_coords(lengths~=max(lengths),:);



% if(sum(lengths~=max(lengths))>1)
%     close_foot_coords = base_foot_coords(lengths~=max(lengths),:);
% else
%     temp = base_foot_coords(lengths==max(lengths),:);
%     close_foot_coords(1,:) = temp(1,:);
%     close_foot_coords(2,:) = base_foot_coords(lengths==min(lengths));
% end

new_foot = end_center*3 - close_foot_coords(1,:) - close_foot_coords(2,:);


end