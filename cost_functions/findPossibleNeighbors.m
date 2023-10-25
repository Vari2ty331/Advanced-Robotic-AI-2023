function [tree, neighboring_nodes, flag] = findPossibleNeighbors(currentTree,range)

flag = 0;
neighboring_nodes = NaN;
input_tree = currentTree.T;
tree = currentTree;

reference_range = range;

temp = [input_tree.center_point];
temp = reshape(temp,[3,length(temp)/3])';

last_center = input_tree(end).center_point;
distance_from_current_node = temp - last_center;


distances = sqrt(distance_from_current_node(:,1).^2+distance_from_current_node(:,2).^2+distance_from_current_node(:,3).^2);
is_in_range = distances < reference_range;

if sum(is_in_range)>2
    flag = 1;
    neighboring_nodes = find(is_in_range == 1);
end

end