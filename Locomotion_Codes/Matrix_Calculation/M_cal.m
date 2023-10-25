function M = M_cal(n)

adj = adj_gen(n.elist);
M = zeros(3,3*length(n.pos));


for i = 1:length(n.pos)
    M(1,i) = length(adj{i});
    M(2,length(n.pos)+i) = length(adj{i});
    M(3,2*length(n.pos)+i) = length(adj{i});
end

M = M/(2*length(n.elist));

    
