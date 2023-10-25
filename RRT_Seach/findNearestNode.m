function [NN_Index, New_Node_Candidate] = findNearestNode(tree, new_node, stepsize)

% Tree = makeReference(tree);
NN_Index = knnsearch(new_node', tree.reference);

vec_rand = new_node - tree.Node(NN_Index).Position;
New_Step_Start = stepsize*vec_rand/norm(vec_rand);
New_Node_Candidate = tree.Node(NN_Index).Position + New_Step_Start;