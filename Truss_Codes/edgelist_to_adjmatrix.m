function adjm = edgelist_to_adjmatrix( elist )
%edgelist_to_edge_adjmatrix generates the node adjacency matrix
n = max(elist(:));
A = accumarray(elist, 1, [n, n]);
adj = A + A';
assert(~any(adj(:) > 1), 'VTT:edgelist_to_adjmatrix', 'Duplicate edge')
assert(all(diag(adj) == 0), 'VTT:edgelist_to_adjmatrix', 'Node connected to itself')
adjm = logical(adj);
end
