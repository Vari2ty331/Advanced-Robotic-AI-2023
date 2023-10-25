function [ min_angle_deg, max_angle_deg ] = min_member_angle( adjm, pos )
%minmemberangle calculates the current smallest angle between adjacent members
%   Optional second output is the largest angle
%   adjm is the node adjacency matrix


% Note about the persistent variable:
% About 70% of the execution time of this was just calling nchoosek to get
% all possible pairs of members at a node. There are only a few possible
% inputs to this (all possible integers corresponding to the number of
% members at a node), so I just precompute the answer for 3 to 20
% (hopefully larger than is ever needed) and look up the answer instead

persistent precomputed_pairs;

if isempty(precomputed_pairs) 
    disp 'min_member_angle: Initializing persistent variable'
    for ii = 20:-1:3
        precomputed_pairs{ii} = nchoosek(1:ii, 2);
    end
end


overall_min_angle = pi;
overall_max_angle = pi;

% At each node, take the angles between each pair of vectors
for ii = 1:length(adjm)
    nbrs = adjm(:,ii);
    nbr_pos = pos(:,nbrs);
    vects = nbr_pos - pos(:,ii);
    colnorm = sqrt(sum(vects.^2));
    nvects = vects./colnorm;
    %pairs = nchoosek(1:size(nvects,2), 2);
    pairs = precomputed_pairs{size(nvects,2)};
    a = nvects(:,pairs(:,1));
    b = nvects(:,pairs(:,2));
    dots = sum(a.*b);
    min_angle = acos(max(dots));
    max_angle = acos(min(dots));
    if min_angle < overall_min_angle
        overall_min_angle = min_angle;
    end
    if max_angle < overall_max_angle
        overall_max_angle = max_angle;
    end
end

min_angle_deg = 180/pi*overall_min_angle;
max_angle_deg = 180/pi*overall_max_angle;

end

