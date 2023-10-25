function [ tree ] = addGroundNodeIndex(tree)

Polygon_Tolerence = 1e-4;

% Array used for find front node
% If non-rotation node is 2, use Node_Conversion(2) to find front node.
Node_Conversion = [ 4, 5, 6, 1, 2, 3];      

index = 1;

for m = 1 : 3
    rotation_node_flag = 0;
    for j = 1 : 3
        if norm(tree.T(end).n(j,:) - tree.T(tree.T(end).Parent_Index).n(m,:)) <= Polygon_Tolerence
            rotation_node_flag = 1;
            tree.T(end).node(index) = tree.T(tree.T(end).Parent_Index).node(m);
            index = index + 1;
            break;
        end
    end
    if rotation_node_flag == 0
        tree.T(tree.T(end).Parent_Index).rear_node = tree.T(tree.T(end).Parent_Index).node(m);
        tree.T(end).node(3) = Node_Conversion(tree.T(tree.T(end).Parent_Index).node(m));
    end
end