function [NN_Index, New_Node_Candidate] = multipleFindNearestNode(local_tree, new_node, stepsize)

% Tree = makeReference(tree);
numTree = numel(local_tree);
NN_Index = -ones(numTree, 1);
New_Node_Candidate = Inf(size(new_node, 2), numTree)';

for t = 1:numTree
    if local_tree(t).valid
        NN_Index(t,1) = knnsearch(new_node, local_tree(t).Position);
        vec_rand = new_node - local_tree(t).Position(NN_Index(t,1),:);
        New_Step_Start = stepsize*vec_rand/norm(vec_rand);
        New_Node_Candidate(t,:) = local_tree(t).Position(NN_Index(t,1),:) + New_Step_Start;
    end 
end

end