function [ Check_Result, i] = Node_Tree_Distance_Check( Node, Tree, stepsize )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

Check_Result = 0;
% for i=1:Tree.No_Node
%     dist = Distance(Node, Tree.Node(i).Position);
%     if dist <= stepsize
%         Check_Result = 1;
%         return;
%     end
% end

[i, dist] = knnsearch(Node', Tree.reference);
if dist <= stepsize
    Check_Result = 1;
end

end

