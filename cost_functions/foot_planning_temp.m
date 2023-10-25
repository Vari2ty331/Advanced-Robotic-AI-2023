function [Final_Path_T,StartTree] = foot_planning_temp(n, param, Ground, data, data_MotionPrimitive,walls)

% clear StartTree Final_Path_Start Final_Path_T obstacle

L_nom = param.L_nom;

%% RRT
disp('RRT planning...');
graph_plot = 1;
Check_Result_Goal = 0;

StartTree.stepsize = param.stepsize;
neighbor_range = 0.8;
% param.mixing_factor = 0;

while(Check_Result_Goal == 0)
    % 1 step setting
    clear StartTree
    Current_dist = 0;
    StartTree.T.rerouted = [];
    iter = 1;

    StartTree.T(1).n = param.Start_T;

    StartTree.T(1).Parent_Index = 0;
    StartTree.T(1).foot = [];
    StartTree.No_Node = 1;
    StartTree.T(1).node = [1, 2, 3];

    StartTree.T(1) = foot_gen(StartTree.T(1),L_nom,Ground);
    StartTree.reference = [];
    StartTree = makeFootReference(StartTree);

    while(iter <= param.maxIter)
        if rand(1) > param.mixing_factor
            Random_Node = GetRandomNode(param.Dim,param.Config_Limit);
            new_node = Random_Node;
        else
            new_node = param.goal';
        end
        % find nearest foot
        [NN_Start, New_Node_Candidate_Start] = findNearestFoot(StartTree, new_node, param.stepsize, Ground);
        % check constraint
        temp_tree_1 = addNewT(StartTree, NN_Start, New_Node_Candidate_Start, L_nom, Ground);
%         temp_tree_1 = addGroundNodeIndex(temp_tree_1);
%         temp_tree_1 = addGoodPose(temp_tree_1, n, data_MotionPrimitive, L_nom);
%         temp_tree = temp_tree_1;

        %cost function here
        temp_tree_2 = costFunctionFromStart(temp_tree_1);
        range = temp_tree_2.T(end).dist_last;
        [temp_tree_2,neighbor_nodes,neighbor_flag] = findPossibleNeighbors(temp_tree_2,range);

        if(neighbor_flag == 1)
            if(checkNearNodeExtendable(temp_tree_2,neighbor_nodes))
                [Nearest_neighbor_node,New_near_foot] = findExtendableNodeFromNeighbors(temp_tree_2,neighbor_nodes);
                if(checkFootCenterInsideTriangle(temp_tree_2,Nearest_neighbor_node))
                    temp_temp_tree = addNewT_temp(StartTree,Nearest_neighbor_node,New_near_foot,L_nom,Ground);
                    tree_is_longer = findLengthtoEndTree(temp_temp_tree,Nearest_neighbor_node);
                    temp_temp_tree = deleteRerouteTree(temp_temp_tree,Nearest_neighbor_node);
%                     temp_temp_tree = addGroundNodeIndex(temp_temp_tree);
%                     temp_temp_tree = addGoodPose(temp_temp_tree, n, data_MotionPrimitive, L_nom);
                    temp_tree = temp_temp_tree;
                    disp('passed')
                else
                    temp_tree = temp_tree_1;
                end
            else
                temp_tree = temp_tree_1;
            end
        else
            tree_is_longer = 1;
            temp_tree = temp_tree_1;
        end
        flag_Start = 1; % new triangle added if flag_Start == 1
        foot_geometry = verifyNewFootGeometry(temp_tree);
        [temp_tree,flag_Start] = costFunctionFromStart(temp_tree);
        flag_Start = flag_Start && foot_geometry && tree_is_longer;


        %neighborcheck



        % Obstacle check
        if exist('obstacle', 'var') ~= 0
            polygon = temp_tree.T(end).n;
            if ~isempty(find(imag(temp_tree.T(end).n),1))
                % Prevent imaginary
                flag_Start = 0;
            end
            if ~isempty(obstacle)
                foot_planning_obstacle_check;
                if min(dist_con) < 0
                    flag_Start = 0;
                end

                % retract new node in opposite direction
                if ~flag_Start
                    for i = 6:-1:1
                        stepsize = temp_tree.T(end).stepsize;
                        temp_stepsize = (i-4)/3*stepsize;
                        [NN_Start, New_Node_Candidate_Start] = findNearestFoot(StartTree, new_node, temp_stepsize);
                        temp_tree = addNewT(StartTree, NN_Start, New_Node_Candidate_Start);
                        polygon = temp_tree.T(end).n;
                        foot_planning_obstacle_check;
                        if min(dist_con) < 0
                            flag_Start = 0;
                        end
                        if flag_Start
                            break;
                        end
                    end
                end

            else
                dist_con = inf;
            end
        end



        %                 %Support polygon verification
        %                 foot_planning_polygon_verification
        %                 if ~isempty(find(constraint_Temp < 0, 1))
        %                     disp('Support polygon not vaild')
        %                     flag_Start = 0;
        %                 end

        % Member collision check
%         Ground_member_collision = foot_planning_member_col_check(temp_tree, Ground);
%         if isempty(find(Ground_member_collision,1)) == 0
%             disp('Member collided with the ground')
%             flag_Start = 0;
%         end

        % Member collision check by wall

        last_foot = temp_tree.T(end).n;
        last_foot = last_foot(:,1:2);
        last_foot = last_foot([1 2 3 1],:);
        last_foot = last_foot';
        wall_starts = reshape([walls.wall.Start],[3,length([walls.wall.Start])/3])';
        wall_starts = wall_starts(:,1:2)';
        wall_ends = reshape([walls.wall.End],[3,length([walls.wall.End])/3])';
        wall_ends = wall_ends(:,1:2)';
        wall_x = [];
        wall_y = [];
        for i = 1:length(wall_starts)
            wall_x = [wall_x wall_starts(1,i) wall_ends(1,i) NaN];
            wall_y = [wall_y wall_starts(2,i) wall_ends(2,i) NaN];
        end

        [intersection_x,intersection_y] = polyxpoly(wall_x,wall_y,last_foot(1,:),last_foot(2,:));

        if ~isempty(intersection_x) && ~isempty(intersection_y)
%             disp('Member collided with wall')
            flag_Start = 0;
        end


        if flag_Start
            StartTree_temp = addNewT(StartTree, NN_Start, New_Node_Candidate_Start, L_nom, Ground);
%             StartTree_temp = addGroundNodeIndex(StartTree_temp);
%             StartTree_temp = addGoodPose(StartTree_temp, n, data_MotionPrimitive, L_nom);
%             StartTree = StartTree_temp;

            StartTree_temp_temp = costFunctionFromStart(StartTree_temp);
            range = StartTree_temp_temp.T(end).dist_last;
            [StartTree_temp_temp,neighbor_nodes,neighbor_flag] = findPossibleNeighbors(StartTree_temp_temp,range);
            if(neighbor_flag == 1)
                if(checkNearNodeExtendable(StartTree_temp_temp,neighbor_nodes))
                    [Nearest_neighbor_node,New_near_foot] = findExtendableNodeFromNeighbors(StartTree_temp_temp,neighbor_nodes);
                    if(checkFootCenterInsideTriangle(StartTree_temp_temp,Nearest_neighbor_node))
                        StartTree_temp_temp = addNewT_temp(StartTree,Nearest_neighbor_node,New_near_foot,L_nom,Ground);
                        StartTree_temp_temp = deleteRerouteTree(StartTree_temp_temp,Nearest_neighbor_node);
%                         StartTree_temp_temp = addGroundNodeIndex(StartTree_temp_temp);
%                         StartTree_temp_temp = addGoodPose(StartTree_temp_temp, n, data_MotionPrimitive, L_nom);
                        StartTree = StartTree_temp_temp;
                    else
                        StartTree = StartTree_temp;
                    end
                else
                    StartTree = StartTree_temp;
                end
            else
                StartTree = StartTree_temp;
            end


            StartTree = costFunctionFromStart(StartTree);

            [Check_Result_Goal,dist] = Node_Goal_Check(StartTree.T(end),param.goal);

%             if Current_dist > StartTree.T(end).dist_init && Check_Result_Goal
%                 Current_Node = StartTree.No_Node;
%                 break
%             end

            if Check_Result_Goal
                Current_dist = StartTree.T(end).dist_init;
                disp('goal reached')
                break
            end


        end



%                 if mod(iter, param.maxIter) == 0
%                     disp(['iteration: ', num2str(iter)]);
%                 end
        disp(['iteration: ', num2str(iter)]);
        if iter == param.maxIter
            disp(['reach maximum iteration: ',num2str(param.maxIter)]);
        end

        for i = 1:length(StartTree.T)
            temp(i,:) = StartTree.T(i).center_point;
        end

        close_center = knnsearch(param.goal,temp);
        goal_dist = norm(param.goal - temp(close_center));

        disp(num2str(goal_dist))

        if goal_dist < 2
            asdf = 0;
        end


        iter = iter +1;
        if any(any(isnan([StartTree.reference;])))
            asdf = 0;
        end
%         if iter == 1434
%             asdf = 0;
%         end
% 
%         if iter == 1
%             plot_count = 1;
%             plot_polygonplanning_debugging_3;
%         else
%             plot_count = 0;
%             plot_polygonplanning_debugging_3;
%         end

%         if iter == 275
%             disp('wait...')
%             pause(1);
%         end
    end % while end

end

%% get final path
disp('getting final path...');

No_FinalNode_Start = 1;
Current_Node = StartTree.No_Node;
while(StartTree.T(Current_Node).Parent_Index ~= 0)
    Final_Path_Start(No_FinalNode_Start) = StartTree.T(Current_Node).Parent_Index;
    Current_Node = StartTree.T(Current_Node).Parent_Index;
    No_FinalNode_Start = No_FinalNode_Start + 1;
end

for i=1: No_FinalNode_Start-1
    Final_Path_T(i).n = StartTree.T(Final_Path_Start(No_FinalNode_Start-i)).n;
    Final_Path_T(i).n;
    %     Final_Path_T(i).OptPose = StartTree.T(Final_Path_Start(No_FinalNode_Start-i)).OptPose;
    %     StartTree.T(Final_Path_Start(No_FinalNode_Start-i)).OptPose
    %     Final_Path_T(i).OptPose
end
Final_Path_T(No_FinalNode_Start).n = StartTree.T(StartTree.No_Node).n;
% Final_Path_T(No_FinalNode_Start).OptPose = StartTree.T(StartTree.No_Node).OptPose;

%% Finding ground node index numbers

Final_Path_T(1).node = [1, 2, 3];
Final_Path_T = foot_GroundNodeIndex(Final_Path_T);


%% plot


if graph_plot == 1
    plot_polygonplanning
end

end
