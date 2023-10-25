function grad = grad_matrix(fun,x0,delta)

% calculating gradient of matrix

grad = [];

for i = 1:numel(x0)
    del = x0;
    del(i) = del(i) + delta;
    del_p = del;
    
    del = x0;
    del(i) = del(i) - delta;
    del_m = del;
    
    grad(:,i) = ( (fun(del_p)).' - (fun(del_m)).' )/ (2*delta);
end

% function = row vector
% change function to column vector and enter it to grad.
