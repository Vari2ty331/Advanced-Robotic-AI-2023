function value = truss_force_cal_fun(X, elist, fixed_node)

Reaction = X(6*length(elist)+1:end);

value = norm(Reaction);

% value = 0;
% 
% for i = 1:length(elist)
%     if isempty(find(fixed_node == elist(i,1),1)) == 0 && isempty(find(fixed_node == elist(i,2),1)) == 0   
%         value = value + (norm(X(6*i - 5: 6*i)))^2; 
%     end
% end
