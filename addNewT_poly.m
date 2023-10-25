function [ tree ] = addNewT_poly(tree, NearestNode_Index, new_node, L_nom, Ground)

N_foot = tree.reference(NearestNode_Index,:);
N_pair = tree.pair(NearestNode_Index,:);

N_foot_index = [];
parent_index = [];

if length(NearestNode_Index) < 2

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

else

    for i = 1:tree.No_Node
        for j = 1:size(tree.T(i).foot,1)
            if tree.T(i).foot(j,1) == N_foot(1,1) && tree.T(i).foot(j,2) == N_foot(1,2)
                if ~isempty(N_foot_index) || ~isempty(parent_index)
                    % disp('redundant occurs during finding parent')
                end
                N_foot_index = j;
                parent_index = i;
            end
        end
    end

end

tree.No_Node = tree.No_Node + 1;
% tree.T(tree.No_Node).Parent_Index = parent_index;
% tree.T(tree.No_Node).n = tree.T(parent_index).n;
% tree.T(tree.No_Node).n(N_foot_index,:) = [];
% tree.T(tree.No_Node).n(3,:) = new_node;

%
%
%
tree.T(tree.No_Node).Parent_Index = parent_index;
tree.T(tree.No_Node).face = 5;


if tree.T(parent_index).face == 3
    asdf = fix((N_foot_index+2)/3);
    tree.T(tree.No_Node).n = tree.T(parent_index).n;
    tree.T(tree.No_Node).n(asdf,:) = [];
    if norm(tree.T(tree.No_Node).n(2,:) - new_node(1,:)) < norm(tree.T(tree.No_Node).n(2,:) - new_node(3,:))
        tree.T(tree.No_Node).n(3,:) = new_node(1,:);
        tree.T(tree.No_Node).n(4,:) = new_node(2,:);
        tree.T(tree.No_Node).n(5,:) = new_node(3,:);
    else
        tree.T(tree.No_Node).n(5,:) = new_node(1,:);
        tree.T(tree.No_Node).n(4,:) = new_node(2,:);
        tree.T(tree.No_Node).n(3,:) = new_node(3,:);
    end
else
    asdf = 0;
end


tree.T(tree.No_Node) = foot_gen_poly(tree.T(tree.No_Node),L_nom,Ground);
tree = makeFootReference(tree,1);