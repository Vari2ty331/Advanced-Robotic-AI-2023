clear StartTree Final_Path_Start Final_Path_T obstacle

% Generalized support polygon planning 

%% RRT
disp('RRT planning...');

Check_Result_Goal = 0;

StartTree.stepsize = param.stepsize;

while(Check_Result_Goal == 0)
    
    % 1 step setting
    clear StartTree
    iter = 1;
    
    StartTree.T(1).n = param.Start_T;

    StartTree.T(1).Parent_Index = 0;
    StartTree.T(1).foot = [];
    StartTree.No_Node = 1;

    StartTree.T(1) = foot_gen(StartTree.T(1),L_nom,Ground);
    StartTree.reference = [];
    StartTree = makeFootReference(StartTree);
   
    while(iter <= maxIter)
        if rand(1) > mixing_factor
            Random_Node = GetRandomNode(Dim,Config_Limit);
            new_node = Random_Node;
        else
            new_node = goal';
        end
        
        % find nearest foot
        [NN_Start, New_Node_Candidate_Start] = findNearestFoot(StartTree, new_node, stepsize, Ground);

        % check constraint
        temp_tree = addNewT(StartTree, NN_Start, New_Node_Candidate_Start, L_nom, Ground);

        flag_Start = 1; % new triangle added if flag_Start == 1
        
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
        
        % Member collision check
        foot_planning_member_col_check;
        if isempty(find(Ground_collision,1)) == 0
            flag_Start = 0;
        end

        if flag_Start
            StartTree = addNewT(StartTree, NN_Start, New_Node_Candidate_Start, L_nom, Ground);
            [Check_Result_Goal,dist] = Node_Goal_Check(StartTree.T(end),goal);
            if Check_Result_Goal
                disp('check')
                break
            end
        end

%         if mod(iter, maxIter) == 0
%             disp(['iteration: ', num2str(iter)]);
%         end
%         if iter == maxIter
%             disp(['reach maximum iteration: ',num2str(maxIter)]);
%         end
        
        iter = iter +1;
    end % while end
    
end

%% get final path
disp('getting final path...');

% if IsCheckResultStart
    No_FinalNode_Start = 1;
    Current_Node = StartTree.No_Node;
    while(StartTree.T(Current_Node).Parent_Index ~= 0)
        Final_Path_Start(No_FinalNode_Start) = StartTree.T(Current_Node).Parent_Index;
        Current_Node = StartTree.T(Current_Node).Parent_Index;
        No_FinalNode_Start = No_FinalNode_Start + 1;
    end

    for i=1: No_FinalNode_Start-1
        Final_Path_T(i).n = StartTree.T(Final_Path_Start(No_FinalNode_Start-i)).n;
    end
    Final_Path_T(No_FinalNode_Start).n = StartTree.T(StartTree.No_Node).n;

    
    
%% plot

foot_GroundNodeIndex

if graph_plot == 1
    foot_planning_plot
end


