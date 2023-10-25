function [ m, n ] = build_legacy_truss( elist, pos )
%build_legacy_truss creates the m and n substructs of the old struct system given the new truss info
%   Use this to interface with old functions that require the old struct

% Why did I move away from the old system?
% The old system had one struct for members, and one struct for nodes.
% The original idea was that functions to do with members could just access
% the m struct, and similar for nodes and n. However, they were heavily 
% interconnected, and usually you just ended up having to pass both.
% The new system is basically just [m.n1, m.n2] and [n.pos], but storing
% things in this way means it is much faster (array indexing is faster than
% struct field access), and also the graph connectivity information is
% cleanly separated from the embedding (position) information.


m.n1 = elist(:,1);
m.n2 = elist(:,2);
% calculate initial length of members
m.l = lengths(elist, pos);
m.names = assign_names(length(m.l));

adjm = edgelist_to_adjmatrix(elist);
n = adjmatrix_to_adjlist(adjm);
for ii = size(adjm,1):-1:1
    n(ii).pos = pos(:,ii);
end

% Update node struct with member names at each node
n(end).m = [];
for ii = 1:length(m.l)
    n(m.n1(ii)).m(end+1) = ii;
    n(m.n2(ii)).m(end+1) = ii;
end

end

function n = adjmatrix_to_adjlist(adjm)
for ii = size(adjm,1):-1:1
    n(ii).adj = find(adjm(:,ii))';
end
end

function names = assign_names(len)
if len <= 26
    names = char((1:len) + 64)';
elseif len <= 52
    names = char([(1:26) + 64, (27:len) + 70])';
else
    assert(false, 'Too many members');
end
end

