function [ truss, rigid ] = build_truss( n, m )
%build_truss constructs a truss struct from the node locations and adjacency list
%   An optional member struct m can be specified to preserve the member
%      labeling when converting a legacy truss struct
%   A truss struct contains:
%      elist - an edgelist, an M-by-2 list of pairs of connected nodes
%      pos - a 3-by-N list of node positions in space
%      h - an optional handle to hold figure information

if nargin < 2
    % edge numbering is assigned based off the ordering from find() on the initial configuration
    adjmatrix = adjlist_to_adjmatrix(n);
    if ~all(sum(adjmatrix) >= 3)
        warning('VTT:build_truss', 'Node with fewer than 3 members')
    end
    [n1, n2] = find(triu(adjmatrix));
    edges = [n1, n2];
else
    % edge numbering taken from existing m struct
    edges = [m.n1, m.n2];
end

pos = [n.pos];

truss.elist = edges;
truss.pos = pos';
rigid = is_inf_rigid(edges, pos);

end




