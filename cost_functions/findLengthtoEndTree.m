function outflag = findLengthtoEndTree(tree,near_node)
tree = costFunctionFromStart(tree);
stopflag = 1;
node_ladder = length(tree.T);
dist_from_ends_old = 0;
while stopflag
    node_recorder = [];
    counter = 0;
    ladder_starter = node_ladder;
    while ladder_starter ~= near_node
        counter = counter + 1;
        node_recorder(counter) = tree.T(ladder_starter).Parent_Index;
        ladder_starter = tree.T(ladder_starter).Parent_Index;
        if ladder_starter < near_node
            node_recorder = [];
            break
        end        
    end
    node_ladder = node_ladder - 1;
    if node_ladder <= 0
        outflag = 1;
        break;
    end
    temp = [];
    for i = node_recorder
        temp = [temp tree.T(i).dist_last];
    end

    dist_from_ends_new = sum(temp) + tree.T(node_ladder).dist_last;

    if dist_from_ends_new > dist_from_ends_old
        dist_from_ends_old = dist_from_ends_new;
    elseif node_ladder < near_node
        final_dist = dist_from_ends_old;
        stopflag = 0;
    end

end

if dist_from_ends_old > tree.T(near_node).dist_init
    outflag = 0;
else
    outflag = 1;
end

end