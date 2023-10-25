function [ tree ] = addNewT(tree, NearestNode_Index, new_node, L_nom, Ground)

N_foot = tree.reference(NearestNode_Index,:);

N_foot_index = [];
parent_index = [];

for i = 1:tree.No_Node
    for j = 1:size(tree.T(i).foot,1)
        if tree.T(i).foot(j,1) == N_foot(1) && tree.T(i).foot(j,2) == N_foot(2)
            if ~isempty(N_foot_index) || ~isempty(parent_index)
                % disp('redundant occurs during finding parent')
            end
            N_foot_index = j;
            parent_index = i;
        end
    end
end

tree.No_Node = tree.No_Node + 1;
tree.T(tree.No_Node).face = 3;
tree.T(tree.No_Node).Parent_Index = parent_index;
temp = tree.T(parent_index).n;
close_index(1) = knnsearch(N_foot,temp);
temp(close_index(1),:) = NaN;
close_index(2) = knnsearch(N_foot,temp);
tree.T(tree.No_Node).n = tree.T(parent_index).n(close_index,:);
tree.T(tree.No_Node).n(3,:) = new_node;

% 
% 
% 
% if tree.T(parent_index).face == 3
%     asdf = fix((N_foot_index+1)/2);
%     tree.No_Node = tree.No_Node + 1;
%     tree.T(tree.No_Node).Parent_Index = parent_index;
%     tree.T(tree.No_Node).n = tree.T(parent_index).n;
%     tree.T(tree.No_Node).n(asdf,:) = [];
%     if mod(N_foot_index,2)
%         tree.T(tree.No_Node).n(4,:) = new_node;
%         tree.T(tree.No_Node).n(3,:) = new_node;
%     end
% end


tree.T(tree.No_Node) = foot_gen_poly(tree.T(tree.No_Node),L_nom,Ground);
tree = makeFootReference(tree,1);