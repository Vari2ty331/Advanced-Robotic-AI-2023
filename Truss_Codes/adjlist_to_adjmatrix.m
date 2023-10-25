function adjmatrix = adjlist_to_adjmatrix( n, check_sym )
%adjacency_matrix Constructs the adjacency matrix for a set of nodes
%   Also checks some conditions to prevent errors
%   For simple graphs only
%   can optionally disable the symmetry check (I use this for partially completed ccs)


if nargin < 2
   check_sym = true; % check symmetry by default
end

adjmatrix = zeros(length(n));
for ii = 1:length(n)
    adjmatrix(ii, n(ii).adj) = adjmatrix(ii, n(ii).adj) + 1;
end
assert(~any(adjmatrix(:) > 1), 'VTT:adjlist_to_adjmatrix', 'Duplicate edge')
assert(all(diag(adjmatrix) == 0), 'VTT:adjlist_to_adjmatrix', 'Node connected to itself')
if check_sym
    assert(issymmetric(adjmatrix), 'VTT:adjlist_to_adjmatrix', 'Adjacency matrix not symmetric')
else
    if ~issymmetric(adjmatrix)
        warning('VTT:adjlist_to_adjmatrix', 'Adjacency matrix not symmetric (Configuration map not undirected)')
    end
end

end

