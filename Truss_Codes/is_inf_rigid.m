function rigid = is_inf_rigid(elist, pos )
%is_inf_rigid Checks a truss for infinitesimal rigidity
%   see rigidity theory notes

rigid = rank(rigidity_matrix(elist, pos)) == numel(pos) - 6; %3n-6
end
