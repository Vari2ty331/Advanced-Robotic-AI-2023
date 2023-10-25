
% Calculating surface normal vector

surface_normal_vector_Temp = [];

Collision_Tol = 1e-1;
for node_index = 1 : size(temp_tree.T(end).n, 2)
    [~, Ground_Distance] = Ground_Collision(Ground, temp_tree.T(end).n(node_index, :));             % Calculating distance between the ground meshes and the node.
    if ~isempty(find(isnan(Ground_Distance),1))
        Ground_index_min = find(isnan(Ground_Distance),1);
        Ground_Distance_min = 0;
    else
        [Ground_Distance_min, Ground_index_min] = min(Ground_Distance);                             % Find the min distance and corresponding index of the ground mesh.
    end
    if (Ground_Distance_min <= Collision_Tol)
        surface_normal_vector_Temp = [surface_normal_vector_Temp; NVector_Find(temp_tree.T(end).node(node_index), Ground, Ground_index_min);];
    end
end
OptPose_Temp = reshape(temp_tree.T(end).OptPose, [1, numel(temp_tree.T(end).OptPose)]);
constraint_Temp = constraint_gen(OptPose_Temp,n,data,temp_tree.T(end).node,surface_normal_vector_Temp,data.MaxStaticCoeff);

surface_normal_vector_Temp = [];