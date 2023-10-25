function [ Tree ] = multipleAddNewNode(tree, NearestNode_Index, new_node)

Tree = tree;
Tree.Position = [Tree.Position; new_node];
Tree.Parent_Index = [Tree.Parent_Index; NearestNode_Index];

end