function grad = grad_simple(fun,x0,delta)

for i = 1:numel(x0)
    del = x0;
    del(i) = del(i) + delta;
    del_p = del;
    
    del = x0;
    del(i) = del(i) - delta;
    del_m = del;
         
%     fun(del_p)
%     fun(del_m)
    
    grad(i) = ( fun(del_p) - fun(del_m) )/ (2*delta);
end

