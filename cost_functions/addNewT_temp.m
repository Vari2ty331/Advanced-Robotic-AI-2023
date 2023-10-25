function [ tree ] = addNewT_temp(tree, NearestNode_Index, new_node, L_nom, Ground)

N_foot = tree.reference(NearestNode_Index,:);

N_foot_index = [];
parent_index = [];

% for i = 1:tree.No_Node
%     for j = 1:size(tree.T(i).foot,1)
%         if tree.T(i).foot(j,1) == N_foot(1) && tree.T(i).foot(j,2) == N_foot(2)
%             if ~isempty(N_foot_index) || ~isempty(parent_index)
%                 disp('redundant occurs during finding parent')
%             end
%             N_foot_index = j;
% %             parent_index = i;
%             parent_index = NearestNode_Index;
%         end
%     end
% end



parent_index = NearestNode_Index;
tree.No_Node = tree.No_Node + 1;
tree.T(tree.No_Node).Parent_Index = parent_index;
tree.T(tree.No_Node).n = tree.T(parent_index).n;
% tree.T(tree.No_Node).n(N_foot_index,:) = [];
tree.T(tree.No_Node).n(sqrt(sum((tree.T(tree.No_Node).n - new_node).^2,2))==max(sqrt(sum((tree.T(tree.No_Node).n - new_node).^2,2))),:) = [];
tree.T(tree.No_Node).n(3,:) = new_node;
tree.T(tree.No_Node).rerouted = 'rerouted';

tree.T(tree.No_Node) = foot_gen_temp(tree.T(tree.No_Node),L_nom,Ground);
tree = makeFootReference(tree,1);