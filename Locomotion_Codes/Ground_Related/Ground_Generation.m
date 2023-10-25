function Ground = Ground_Generation(Ground_Set)

% Ground_Set : Contains the ground point that a form of [x_i, y_i, z_i;]
% The point is on the top of the surface. Do not add bottom virtual points.
% It is automatically added by this code.

DT = delaunay(Ground_Set(:,1), Ground_Set(:,2));
trimesh(DT, Ground_Set(:,1), Ground_Set(:,2), Ground_Set(:,3));
z_value = min(Ground_Set(:,3)) - 1;

DT_Size = size(DT);

for k = 1 : DT_Size(1)
    Ground_Mesh_Temp = Ground_Set(DT(k,:), :);
    Ground_Mesh_Temp = [Ground_Mesh_Temp; Ground_Set(DT(k,:), 1:2), ones(3,1)*z_value ;];
    Ground(k) = collisionMesh(Ground_Mesh_Temp);
end

% Code for testing ground generation

% for k = 1 : length(Ground)
%     hold on
%     show(Ground(k))
% end

end