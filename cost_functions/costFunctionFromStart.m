function [treeWithCost,flag] = costFunctionFromStart(tree)

currentTree = tree.T;
flag = 1;

currentTree(1).dist_init = [];
currentTree(1).dist_last = [];
currentTree(1).center_point = [];

for itr = 1:length(currentTree)
    if itr < 2
        dist = 0;
        center = sum(currentTree(itr).n/3,[3 1]);
        last_dist = 0;
    else
        center_distance = norm(sum(currentTree(itr).n,[3 1])/3-sum(currentTree(currentTree(itr).Parent_Index).n,[3 1])/3);
        dist = currentTree(currentTree(itr).Parent_Index).dist_init + center_distance;
        center = sum(currentTree(itr).n/3,[3 1]);
        last_dist = center_distance;
    end
    currentTree(itr).dist_init = dist;
    currentTree(itr).center_point = center;
    currentTree(itr).dist_last = last_dist;
    if itr == length(currentTree) && last_dist == 0
        flag = 0;
    end
end


tree.T = currentTree;
treeWithCost = tree;


end