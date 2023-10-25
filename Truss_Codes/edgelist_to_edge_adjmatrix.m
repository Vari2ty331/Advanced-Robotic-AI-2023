function lg_adjmatrix = edgelist_to_edge_adjmatrix( elist )
%edgelist_to_edge_adjmatrix generates the adjacency matrix of the line graph of a truss

n1 = elist(:,1);
n2 = elist(:,2);
lg_adjmatrix = zeros(length(n1));

for ii = 1:length(n1)
    nbr = n1 == n1(ii) | n1 == n2(ii) | n2 == n1(ii) | n2 == n2(ii);
    nbr(ii) = false;
    lg_adjmatrix(:,ii) = nbr;
end
end

