function d_min = d_min_cal(constraint,data,n)
% constraint is n by 1 vector

clist = clist_gen(n.elist);
d_min_index = 5*length(clist);

if d_min_index > length(constraint)
    d_min = NaN;
else
    d_min = min(constraint(end-d_min_index+1:end)) + data.desired_d_min^2;
    d_min = sqrt(d_min);
end