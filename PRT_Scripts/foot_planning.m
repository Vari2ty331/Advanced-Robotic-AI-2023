function [Final_Path_T,StartTree] = foot_planning(n, param, Ground, data, data_MotionPrimitive)

% clear StartTree Final_Path_Start Final_Path_T obstacle

L_nom = param.L_nom;

%% RRT
disp('RRT planning...');
graph_plot = 1;
Check_Result_Goal = 0;

StartTree.stepsize = param.stepsize;
neighbor_range = 0.8;

while(Check_Result_Goal == 0)

    % 1 step setting
    clear StartTree
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
        temp_tree = addNewT(StartTree, NN_Start, New_Node_Candidate_Start, L_nom, Ground);
        temp_tree = addGroundNodeIndex(temp_tree);
        temp_tree = addGoodPose(temp_tree, n, data_MotionPrimitive, L_nom);

        %cost function here
%         temp_tree = costFunctionFromStart(temp_tree);
%         range = temp_tree.T(end).dist_last;
%         [temp_tree,neighbor_nodes,neighbor_flag] = findPossibleNeighbors(temp_tree,range);
% 
%         if(neighbor_flag == 1)
%             if(checkNearNodeExtendable(temp_tree,neighbor_nodes))
%                 [Nearest_neighbor_node,New_near_foot] = findExtendableNodeFromNeighbors(temp_tree,neighbor_nodes);
%                 if(checkFootCenterInsideTriangle(temp_tree,Nearest_neighbor_node))
%                     plot_polygonplanning_debugging;
%                     temp_tree = addNewT_temp(temp_tree,Nearest_neighbor_node,New_near_foot,L_nom,Ground);
%                     temp_tree = addGroundNodeIndex(temp_tree);
%                     temp_tree = addGoodPose(temp_tree, n, data_MotionPrimitive, L_nom);
%                 end
%             end
%         end

        flag_Start = 1; % new triangle added if flag_Start == 1


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



        %Support polygon verification
        foot_planning_polygon_verification
        if ~isempty(find(constraint_Temp < 0, 1))
            disp('Support polygon not vaild')
            flag_Start = 0;
        end

        % Member collision check
        Ground_member_collision = foot_planning_member_col_check(temp_tree, Ground);
        if isempty(find(Ground_member_collision,1)) == 0
            disp('Member collided with the ground')
            flag_Start = 0;
        end

        if flag_Start
            StartTree = addNewT(StartTree, NN_Start, New_Node_Candidate_Start, L_nom, Ground);
            StartTree = addGroundNodeIndex(StartTree);
            StartTree = addGoodPose(StartTree, n, data_MotionPrimitive, L_nom);
%             StartTree = costFunctionFromStart(StartTree);
%             range = StartTree.T(end).dist_last;
%             [StartTree,neighbor_nodes,neighbor_flag] = findPossibleNeighbors(StartTree,range);
% 
%             if(neighbor_flag == 1)
%                 if(checkNearNodeExtendable(StartTree,neighbor_nodes))
%                     [Nearest_neighbor_node,New_near_foot] = findExtendableNodeFromNeighbors(StartTree,neighbor_nodes);
%                     if(checkFootCenterInsideTriangle(StartTree,Nearest_neighbor_node))
%                         plot_polygonplanning_debugging;
%                         StartTree = addNewT_temp(StartTree,Nearest_neighbor_node,New_near_foot,L_nom,Ground);
%                         StartTree = addGroundNodeIndex(StartTree);
%                         StartTree = addGoodPose(StartTree, n, data_MotionPrimitive, L_nom);
%                     end
%                 end
%             end
            [Check_Result_Goal,dist] = Node_Goal_Check(StartTree.T(end),param.goal);
            if Check_Result_Goal
                disp('check')
                break
            end
        end



        %         if mod(iter, maxIter) == 0
        %             disp(['iteration: ', num2str(iter)]);
        %         end
        disp(['iteration: ', num2str(iter)]);
        if iter == param.maxIter
            disp(['reach maximum iteration: ',num2str(param.maxIter)]);
        end

        iter = iter +1;
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
    Final_Path_T(i).n
    Final_Path_T(i).OptPose = StartTree.T(Final_Path_Start(No_FinalNode_Start-i)).OptPose;
    StartTree.T(Final_Path_Start(No_FinalNode_Start-i)).OptPose
    Final_Path_T(i).OptPose
end
Final_Path_T(No_FinalNode_Start).n = StartTree.T(StartTree.No_Node).n;
Final_Path_T(No_FinalNode_Start).OptPose = StartTree.T(StartTree.No_Node).OptPose;

%% Finding ground node index numbers

Final_Path_T(1).node = [1, 2, 3];
Final_Path_T = foot_GroundNodeIndex(Final_Path_T);


%% plot


if graph_plot == 1
    plot_polygonplanning
end

end
