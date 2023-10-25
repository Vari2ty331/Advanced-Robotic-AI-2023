function Ground_member_collision = foot_planning_member_col_check(temp_tree, Ground)

% This code finds the collision between members on the ground and ground
% meshes for support polygon planning.

% Ground_inrange : the matrix that indicates if corresponding ground mesh
% is in range of collision check region.
% Row: ground mesh index
% Column: member mesh index number
% Ground_collision : The matrix that indicates if corresponding ground mesh
% is collided with members. The structure is same as Ground_inrange.

polygon = temp_tree.T(end).n;

Point_Temp = [];

for index = 1 : 3
    for k = 1 : 3
        if index ~= k
            Point_Temp = [Point_Temp; polygon(k,:);];
        end
    end
    MemberMesh(index) = collisionMesh(Point_Temp);
    Point_Temp = [];
end

Ground_Size = size(Ground);
Ground_inrange = zeros(3,Ground_Size(2));   % The index of ground mesh that in range of detecting collision.

% Find the ground meshes that included on the collision check region.
for k = 1 : Ground_Size(2)
    GroundVertices_Projected(:,:,k) = Ground(k).Vertices(1:3,1:2);
    for index = 1 : 3
    	Intersections = polyxpoly(MemberMesh(index).Vertices(:,1), MemberMesh(index).Vertices(:,2), GroundVertices_Projected(:,1,k),GroundVertices_Projected(:,2,k));
        if isempty(Intersections) == 0
            Ground_inrange(index,k) = 1;        
        end
    end
end

Ground_member_collision = zeros(3,Ground_Size(2));
for index = 1 : 3
    for k = 1 : Ground_Size(2)
        if Ground_inrange(index,k) == 1
            Ground_member_collision(index, k) = checkCollision(MemberMesh(index),Ground(k));
        end
    end
end
