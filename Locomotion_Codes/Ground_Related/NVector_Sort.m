function NVector_Sorted = NVector_Sort(surface_normal_vector, fixed_node)

% Sorting the normal vectors w.r.t. the fixed node index.
% This function can also be used for remove vector that not contacting to
% the ground anymore.

NVector_Sorted = [];
NVector_Size = size(surface_normal_vector);

for fixed_node_index = 1 : length(fixed_node)
    for NVector_index = 1 : NVector_Size(1)
        if surface_normal_vector(NVector_index, 4) == fixed_node(fixed_node_index)
            NVector_Sorted = [NVector_Sorted; surface_normal_vector(NVector_index, :);];
            break;
        end
    end
end

end

    