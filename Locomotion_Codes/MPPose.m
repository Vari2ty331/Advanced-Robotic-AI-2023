function [Pose_Start_ini, Pose_End_ini, MotionPrimitive_Transform, Rotation_Angle, Polygon_Center_1] = MPPose(tree_pre_T, tree_current_T, n, data_MotionPrimitive)
    % This function returns the initial and end pose of motion primitive.
    % The node number of motion primitive is matched w.r.t. support
    % polygon. The center of first polygon of motion primitive is moved to
    % the center of initial polygon. The direction is also set as same.
    % The function also returns the transformation matrix.

    Node_Conversion = [ 4, 5, 6, 1, 2, 3];

    % Match the node number of the motion primitive to support polygon
    % Transform the motion primitive for every time step
    % Make transform matrix for one step
    MotionPrimitive_Transform = zeros(6,6);             % initialize transform matrix
    MotionPrimitive_Transform(tree_pre_T.rear_node,1) = 1;                         % Change node 1 of motion primitive to rear node
    MotionPrimitive_Transform(Node_Conversion(tree_pre_T.rear_node),4) = 1;        % Change node 4 of motion primitive to front node
    
    Pos_Rear = tree_pre_T.n(tree_pre_T.node == tree_pre_T.rear_node,:);                          % Position of rear node
    Pos_Front = tree_current_T.n(tree_current_T.node == Node_Conversion(tree_pre_T.rear_node),:);    % Position of front node (on next polygon)
    Step_Direction = Pos_Front - Pos_Rear;              % The axis that determines rotation angle(Vector between two centers of adjacent polygons).
    
    for node_index = 1 : 3
        % Find rotation node index between three ground node
        if tree_pre_T.node(node_index) ~= tree_pre_T.rear_node
            Node_Direction = cross(Step_Direction, tree_pre_T.n(tree_pre_T.node == tree_pre_T.node(node_index),:) - Pos_Rear);     % Cross prodcut of step direction and the rotation node direction
            if Node_Direction(3) >= 0
                MotionPrimitive_Transform(tree_pre_T.node(node_index),2) = 1;
                MotionPrimitive_Transform(Node_Conversion(tree_pre_T.node(node_index)),5) = 1;
            else
                MotionPrimitive_Transform(tree_pre_T.node(node_index),3) = 1;
                MotionPrimitive_Transform(Node_Conversion(tree_pre_T.node(node_index)),6) = 1;
            end
        end
    end
    
    % Find the center of polygon (using this to move the motion primitive)
    Polygon_Center_1 = [0, 0, 0];               % Center of previous support polygon
    Polygon_Center_2 = [0, 0, 0];               % Center of next support polygon
    for foot_index = 1 : 3
        Polygon_Center_1 = Polygon_Center_1 + tree_pre_T.n(foot_index,:)/3;
        Polygon_Center_2 = Polygon_Center_2 + tree_current_T.n(foot_index,:)/3;
    end
    Pose_Reference = Polygon_Center_2 - Polygon_Center_1;           % The axis that determines rotation angle(Vector between two centers of adjacent polygons).
    
    % FInd the rotating angle
    Rotation_Angle = acos(dot(Pose_Reference, [1, 0, 0])/norm(Pose_Reference));                 % Find angle from dot product
    Rotation_Angle_Direction = cross([1,0,0], Pose_Reference);                                  % Cross product between step direction and (1,0,0)
    Rotation_Angle = Rotation_Angle * sign(Rotation_Angle_Direction(3));                        % Find angle direction from cross product
    
    % Find the optimal pose for the support polygon
    Pose_Start_ini = MotionPrimitive_Transform * reshape(data_MotionPrimitive.x(1,:),size(n.pos));           % Initial starting pose of previous support polygon
    Pose_End_ini = MotionPrimitive_Transform * reshape(data_MotionPrimitive.x(end,:),size(n.pos));           % Initial ending pose of next support polygon
    for k = 1 : length(Pose_Start_ini)
        Pose_Start_ini(k,:) = (Rotating_Line((Pose_Start_ini(k,:)).', [0,0,0], [0,0,1], Rotation_Angle)).';
        Pose_Start_ini(k,:) = Pose_Start_ini(k,:) + Polygon_Center_1;
        Pose_End_ini(k,:) = (Rotating_Line((Pose_End_ini(k,:)).', [0,0,0], [0,0,1], Rotation_Angle)).';
        Pose_End_ini(k,:) = Pose_End_ini(k,:) + Polygon_Center_1;        
    end
    
end