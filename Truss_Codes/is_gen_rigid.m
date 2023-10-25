function gen_rigid = is_gen_rigid( elist )
%is_gen_rigid Checks a truss for generic rigidity
%   Actually, this is not guaranteed to work if you get unlucky with rng
%   and get 2 singularity configurations. This should be rare.

% test 2 random configurations and see if they are all rigid

for ii = 2:-1:1
    pos = rand([3, max(elist(:))]);
    rigid(ii) = is_inf_rigid(elist, pos);
end

gen_rigid = any(rigid);

if gen_rigid && ~all(rigid)
    warning('VTT:is_gen_rigid', 'Randomly found a singularity configuration')
end

end