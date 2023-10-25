function surface_normal_vector = NVector_Find(fixed_node_index, Ground, Collided_Ground_Index)

% Find a surface normal vector for each fixed node.
% fixed_node_index : Target fixed node
% Ground_Index : the index of Ground mesh that collides with the node.
% surface_normal_vector = [ v1, node_number_1;]

Normal_Vector = cross((Ground(Collided_Ground_Index).Vertices(2,:) - Ground(Collided_Ground_Index).Vertices(1,:)),(Ground(Collided_Ground_Index).Vertices(3,:) - Ground(Collided_Ground_Index).Vertices(1,:)));
Normal_Vector =  Normal_Vector/norm(Normal_Vector);
if Normal_Vector(3) < 0
    Normal_Vector = -Normal_Vector;
end


surface_normal_vector = [Normal_Vector, fixed_node_index];