function [NN_Index, New_Node_Candidate] = findNearestFoot_poly(tree, new_node, stepsize, Ground)

NN_Index = knnsearch(new_node', tree.reference);

if tree.pair(NN_Index,1) == tree.pair(NN_Index,2)

    vec_rand = new_node' - tree.reference(NN_Index,:);
    New_Step_Start = stepsize*vec_rand/norm(vec_rand);

    New_Node_Initial = tree.reference(NN_Index,:) + New_Step_Start;
    New_Node_Candidate = New_Node_Initial;

else
    [row,col] = find(tree.pair == NN_Index);
    if any(col == 1)
        NPair_Index = NN_Index + 1;
        NPair_Index2 = NN_Index + 2;
        NN_Index = [NN_Index NPair_Index NPair_Index2];
    elseif any(col == 2)
        NPair_Index = NN_Index - 1;
        NPair_Index2 = NN_Index + 1;
        NN_Index = [NPair_Index NN_Index NPair_Index2];
    elseif any(col == 3)
        NPair_Index = NN_Index - 2;
        NPair_Index2 = NN_Index - 1;
        NN_Index = [NPair_Index NPair_Index2 NN_Index];
    end

    
    vec_rand = new_node' - tree.reference(NN_Index,:);
    New_Step_Start = stepsize*vec_rand/norm(vec_rand);

    New_Node_Initial = tree.reference(NN_Index,:) + New_Step_Start;
    New_Node_Candidate = New_Node_Initial;
end
end