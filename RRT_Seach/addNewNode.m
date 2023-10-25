function [ Tree ] = addNewNode(tree, NearestNode_Index, new_node)

Tree = tree;
Tree.No_Node = Tree.No_Node + 1;
Tree.Node(Tree.No_Node).Position = new_node;
Tree.Node(Tree.No_Node).Parent_Index = NearestNode_Index;

Tree = makeReference(Tree);