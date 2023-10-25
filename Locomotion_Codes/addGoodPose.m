function [ tree ] = addGoodPose(tree, n, data_MotionPrimitive, L_nom)

    [Pose_Start_ini, Pose_End_ini] = MPPose(tree.T(tree.T(end).Parent_Index), tree.T(end), n, data_MotionPrimitive);

    Polygon_HighPoint_1 = max(tree.T(tree.T(end).Parent_Index).n(:,3));
    Polygon_HighPoint_2 = max(tree.T(end).n(:,3));
    Polygon_HeightDiff = [ 0, 0, Polygon_HighPoint_2 - Polygon_HighPoint_1;];    


    Pose_Start = Pose_Start_ini;
    j = 1;
    for k = tree.T(tree.T(end).Parent_Index).node
        Pose_Start(k,:) = tree.T(tree.T(end).Parent_Index).n(j,:);
        j = j + 1;
    end
    Pose_End = Pose_End_ini + Polygon_HeightDiff;
    j = 1;
    for k = tree.T(end).node
        Pose_End(k,:) = tree.T(end).n(j,:);
        j = j + 1;
    end
    
    x_length = length(data_MotionPrimitive.x(1,:));
    Cost_Fun_OptPose = @(x) norm(L_nom*ones(12,1) - lengths(n.elist,reshape(x,size(n.pos))));                   % Minimize the difference from L_nom
    options = optimoptions('fmincon','Display','none');
    % If first setp, generate the optimal pose for start. If not, use the
    % previous final optimal pose for start
    if tree.T(end).Parent_Index == 1
        % Making Constraints for Pose_Start
        Cons_OptPose_A = zeros(x_length,x_length);                                      % Equality constraint matrix Ax = B
        Cons_OptPose_B = zeros(x_length,1);                                             % Equality constraint vector Ax = B
        j = 1;
        for k = tree.T(tree.T(end).Parent_Index).node
            Cons_OptPose_A(k,k) = 1;                                                    % x position of ground node (A)
            Cons_OptPose_A(k + x_length/3,k + x_length/3) = 1;                          % y position of ground node (A)
            Cons_OptPose_A(k + 2*x_length/3,k + 2*x_length/3) = 1;                      % z position of ground node (A)
            Cons_OptPose_B(k) = tree.T(tree.T(end).Parent_Index).n(j,1);                     % x position of ground node (B)
            Cons_OptPose_B(k + x_length/3) = tree.T(tree.T(end).Parent_Index).n(j,2);        % y position of ground node (B)
            Cons_OptPose_B(k + 2*x_length/3) = Polygon_HighPoint_1;      % z position of ground node (B)
            j = j + 1;
        end
        % Optimizing Pose_Start and put it in the Final_Path_T
        tree.T(tree.T(end).Parent_Index).OptPose = reshape(fmincon(Cost_Fun_OptPose, Pose_Start, [], [], Cons_OptPose_A, Cons_OptPose_B, [] ,[] ,[] ,options), size(n.pos));
        j = 1;
        for k = tree.T(tree.T(end).Parent_Index).node
            tree.T(tree.T(end).Parent_Index).OptPose(k,1) = tree.T(tree.T(end).Parent_Index).n(j,1);
            tree.T(tree.T(end).Parent_Index).OptPose(k,2) = tree.T(tree.T(end).Parent_Index).n(j,2);
            tree.T(tree.T(end).Parent_Index).OptPose(k,3) = tree.T(tree.T(end).Parent_Index).n(j,3);
            j = j + 1;
        end
        OptX_Start = tree.T(tree.T(end).Parent_Index).OptPose;
    else
        OptX_Start = tree.T(tree.T(end).Parent_Index).OptPose;
    end
    
    % Making Constraints for Pose_End
    Cons_OptPose_A = zeros(x_length,x_length);                                      % Equality constraint matrix Ax = B
    Cons_OptPose_B = zeros(x_length,1);                                             % Equality constraint vector Ax = B
    j = 1;
    for k = tree.T(end).node
        Cons_OptPose_A(k,k) = 1;                                                        % x position of ground node (A)
        Cons_OptPose_A(k + x_length/3,k + x_length/3) = 1;                              % y position of ground node (A)
        Cons_OptPose_A(k + 2*x_length/3,k + 2*x_length/3) = 1;                          % z position of ground node (A)
        Cons_OptPose_B(k) = tree.T(end).n(j,1);                     % x position of ground node (B)
        Cons_OptPose_B(k + x_length/3) = tree.T(end).n(j,2);        % y position of ground node (B)
        Cons_OptPose_B(k + 2*x_length/3) = Polygon_HighPoint_2;      % z position of ground node (B)
        j = j + 1;
    end
    % Optimizing Pose_End and put it in the Final_Path_T
    tree.T(end).OptPose = reshape(fmincon(Cost_Fun_OptPose, Pose_End, [], [], Cons_OptPose_A, Cons_OptPose_B, [] ,[] ,[] ,options), size(n.pos));
    j = 1;
    for k = tree.T(end).node
        tree.T(end).OptPose(k,1) = tree.T(end).n(j,1);
        tree.T(end).OptPose(k,2) = tree.T(end).n(j,2);
        tree.T(end).OptPose(k,3) = tree.T(end).n(j,3);
        j = j + 1;
    end
    
end