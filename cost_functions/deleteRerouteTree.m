function outtree = deleteRerouteTree(tree,nearest_node)

stopflag = 1;
delete_indecies = [];


% tree.T(nearest_node).rerouted = 'delete';
for i = 1:length(tree.T)-1
    if tree.T(i).Parent_Index == nearest_node
        tree.T(i).rerouted = 'delete';
    end
end

temp_tree = tree;

while stopflag
    for itr = 1:length(tree.T)-1
        if strcmp(tree.T(itr).rerouted,'delete')
            delete_indecies = [delete_indecies itr];
        end

        if any(tree.T(itr).Parent_Index == delete_indecies)
            tree.T(itr).rerouted = 'delete';
        end
    end
    if strcmp([temp_tree.T.rerouted],[tree.T.rerouted])
        break
    else
        temp_tree = tree;
    end
end

% rerouted_indices = find(temp == 1);
%
% parent_indices = [tree.T.Parent_Index];
%
% for i = rerouted_indices
%
%     temp = find(parent_indices == i);
%     rerouted_indices = [rerouted_indices temp];
%
% end
%
% if any(tree.T(end).Parent_Index == rerouted_indices)
%     outflag = 1;
%     tree.T(end) = [];
% else
%     outflag = 0;
%     tree.T(rerouted_indices) = [];
% tree.No_Node = length(tree.T);
% tree = makeFootReference(tree,1);
% end

temp_tree = tree;
delete_nodes = [];

for i = 1:length(tree.T)-1
    if strcmp(tree.T(i).rerouted,'delete')
        delete_nodes = [delete_nodes i];
    end
end

delete_nodes = unique(delete_nodes);

for i = 1:length(temp_tree.T)-1
    for j = delete_nodes
        if tree.T(i).Parent_Index > j
            temp_tree.T(i).Parent_Index = temp_tree.T(i).Parent_Index - 1;
        end
    end
end

temp_tree.T(delete_nodes) = [];

tree = temp_tree;
tree.No_Node = length(tree.T);
tree = makeFootReference(tree,1);
outtree = tree;

end