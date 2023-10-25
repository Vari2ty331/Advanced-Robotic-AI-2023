function [path, StartTree, GoalTree] = member_length_BiRRT(RRT_structure, truss, maxIter)
tic;
global fixed_index length_of_fixed_index

if nargin < 3
    maxIter = 10000000;
end


%% set param
iter = 1;
eps = 0.00001;

%% set user defined param
param = RRT_structure;
Dim = param.Dim;
stepsize = param.stepsize;
% bMixingFactor = 0;

Start_Node = param.Start_Node;
Goal_Node = param.Goal_Node;
Config_Limit = param.Config_Limit;


%% 1step setting

StartTree.Node(1).Position = Start_Node;
StartTree.Node(1).Parent_Index = 0;
StartTree.No_Node = 1;
StartTree = makeReference(StartTree);

GoalTree.Node(1).Position = Goal_Node;
GoalTree.Node(1).Parent_Index = 0;
GoalTree.No_Node = 1;
GoalTree = makeReference(GoalTree);

IsCheckResultStart = 0;
IsCheckResultGoal = 0;
% check_constraints_truss(truss, planning_pos_2_whole_pos(Start_Node, fixed_index, pos_of_fixed_index));
% check_constraints_truss(truss, planning_pos_2_whole_pos(Goal_Node, fixed_index, pos_of_fixed_index));

%% RRT
StartTree_Connect_Index = 0;
GoalTree_Connect_Index = 0;
disp('RRT planning...');

while(1)
   %% Start tree
    if rand(1) < param.mixingFactor
        Random_Node = Goal_Node;
        bMixingFactor = 1;
    else
        Random_Node = GetRandomNode(Dim, Config_Limit);
        bMixingFactor = 0;
    end
        
    
    % find nearest neighbor
    [NN_Start, New_Node_Candidate_Start] = findNearestNode(StartTree, Random_Node, stepsize);
  
    % check constraint
%     flag_Start = checkConstraint(New_Node_Candidate_Start, params);
%     flag_Start = check_constraints_truss(truss, planning_pos_2_whole_pos(New_Node_Candidate_Start, fixed_index, pos_of_fixed_index));
    flag_Start = true;
    
    % add new node
    if flag_Start == 0
%        startNode_in_constraints(:,num_startNode_in_constraints) = Random_Node;
       startNode_in_constraints(:,num_startNode_in_constraints) = New_Node_Candidate_Start;
       num_startNode_in_constraints = num_startNode_in_constraints + 1;
    elseif flag_Start == 1
        StartTree = addNewNode(StartTree, NN_Start, New_Node_Candidate_Start);
        % plot tree
%         drawRRTProgress(StartTree, h);
        % check connection
        [ Check_Result_Goal, GoalTree_Connect_Index] = Node_Tree_Distance_Check(StartTree.Node(StartTree.No_Node).Position, GoalTree, stepsize);
        if Check_Result_Goal == 1
            StartTree_Connect_Index = StartTree.No_Node;
%             break;
        end
    end
    
    %% Goal Tree
    % find nearest neighbor
    if bMixingFactor
        Random_Node = Start_Node;
    end
    
    [NN_Goal, New_Node_Candidate_Goal] = findNearestNode(GoalTree, Random_Node, stepsize);
    
    % check constraint
%     flag_Goal = checkConstraint(New_Node_Candidate_Goal, params);
%     flag_Goal = check_constraints_truss(truss, planning_pos_2_whole_pos(New_Node_Candidate_Goal, fixed_index, pos_of_fixed_index));
    flag_Goal = true;
    % add new node
    if flag_Goal == 0
%        goalNode_in_constraints(:,num_goalNode_in_constraints) = Random_Node;
       goalNode_in_constraints(:,num_goalNode_in_constraints) = New_Node_Candidate_Goal;
       num_goalNode_in_constraints = num_goalNode_in_constraints + 1;
    elseif flag_Goal == 1
        GoalTree = addNewNode(GoalTree, NN_Goal, New_Node_Candidate_Goal);
        % plot tree
%         drawRRTProgress(GoalTree, h);
        % check connection
        [ Check_Result_Start, StartTree_Connect_Index] = Node_Tree_Distance_Check(GoalTree.Node(GoalTree.No_Node).Position, StartTree, stepsize);
        if Check_Result_Start == 1
            GoalTree_Connect_Index = GoalTree.No_Node;
%             break
        end
    end
    
%     % check connection
%     [ Check_Result_Start, StartTree_Connect_Index] = Node_Tree_Distance_Check(GoalTree.Node(GoalTree.No_Node).Position, StartTree, stepsize);
%     if Check_Result_Start == 1;
%         GoalTree_Connect_Index = GoalTree.No_Node;
%         break
%     else [ Check_Result_Goal, GoalTree_Connect_Index] = Node_Tree_Distance_Check(StartTree.Node(StartTree.No_Node).Position, GoalTree, stepsize);
%         if Check_Result_Goal == 1;
%             StartTree_Connect_Index = StartTree.No_Node;
%             break;
%         end
%     end

    iter = iter +1;
    if mod(iter, 500) == 0
        disp(['iteration: ', num2str(iter)]);
    end
    if mod(iter, maxIter) == 0
%         save(['result', num2str(iter)], 'StartTree', 'GoalTree');
       break;
    end
end


%% get final path
disp('getting final path...');

% if length(StartTree_Connect_Index) ~= 1
%     StartTree_Connect_Index = StartTree_Connect_Index(1);
% end
% if length(GoalTree_Connect_Index) ~= 1
%     GoalTree_Connect_Index = GoalTree_Connect_Index(1);
% end

if StartTree_Connect_Index
    No_FinalNode_Start = 1;
    Current_Node = StartTree_Connect_Index;
    while(StartTree.Node(Current_Node).Parent_Index ~= 0)
        Final_Path_Start(No_FinalNode_Start) = StartTree.Node(Current_Node).Parent_Index;
        Current_Node = StartTree.Node(Current_Node).Parent_Index;
        No_FinalNode_Start = No_FinalNode_Start + 1;
    end

    % Final_Path_S = zeros(No_FinalNode_Start-1, Dim);
    Final_Path_S = zeros(No_FinalNode_Start, Dim);
    for i=1: No_FinalNode_Start-1
        Final_Path_S(i,:) = StartTree.Node(Final_Path_Start(No_FinalNode_Start-i)).Position';
    end
    Final_Path_S(No_FinalNode_Start,:) = StartTree.Node(StartTree_Connect_Index).Position';
    
    No_FinalNode_Goal = 1;
    Current_Node = GoalTree_Connect_Index + 1;
    GoalTree.Node(Current_Node) = StartTree.Node(StartTree_Connect_Index);
    GoalTree.Node(Current_Node).Parent_Index = GoalTree_Connect_Index;
    while(GoalTree.Node(Current_Node).Parent_Index ~= 0)
        Final_Path_Goal(No_FinalNode_Goal) = GoalTree.Node(Current_Node).Parent_Index;
        Current_Node = GoalTree.Node(Current_Node).Parent_Index;
        No_FinalNode_Goal = No_FinalNode_Goal + 1;
    end

    Final_Path_G = zeros(No_FinalNode_Goal-1, Dim);
    for i=1:No_FinalNode_Goal-1
        Final_Path_G(i,:) = GoalTree.Node(Final_Path_Goal(i)).Position';
    end
else
    Final_Path_S = [];
    Final_Path_G = [];
end


% if GoalTree_Connect_Index
%     No_FinalNode_Goal = 1;
%     Current_Node = GoalTree_Connect_Index + 1;
%     GoalTree.Node(Current_Node) = StartTree.Node(StartTree_Connect_Index);
%     GoalTree.Node(Current_Node).Parent_Index = GoalTree_Connect_Index;
%     while(GoalTree.Node(Current_Node).Parent_Index ~= 0)
%         Final_Path_Goal(No_FinalNode_Goal) = GoalTree.Node(Current_Node).Parent_Index;
%         Current_Node = GoalTree.Node(Current_Node).Parent_Index;
%         No_FinalNode_Goal = No_FinalNode_Goal + 1;
%     end
% 
%     Final_Path_G = zeros(No_FinalNode_Goal-1, Dim);
%     for i=1:No_FinalNode_Goal-1
%         Final_Path_G(i,:) = GoalTree.Node(Final_Path_Goal(i)).Position';
%     end
% else
%     Final_Path_G = [];
% end

Final_Path = [Final_Path_S; Final_Path_G];
Final_Path = Final_Path';
time = toc;
if length(Final_Path)
    Smooth_Path = GetSmoothPath(Final_Path, stepsize);
    % Smooth_Path = Smooth_Path';

    path.final_path = zeros(size(Final_Path,1) + length(fixed_index), size(Final_Path,2));
    % path.smooth_path = zeros(size(Smooth_Path,1) + length(fixed_index), size(Smooth_Path,2));

    for i=1:size(Final_Path,2)
%         path.final_path(:,i) = planning_pos_2_whole_pos(Final_Path(:,i), fixed_index, pos_of_fixed_index);
        path.final_path(:,i) = planning_mem_2_whole_mem(Final_Path(:,i), fixed_index, length_of_fixed_index);
    end
else
    path.final_path = [];
end

% for i=1:size(Smooth_Path,2)
%     path.smooth_path(:,i) = planning_pos_2_whole_pos(Smooth_Path(:,i), fixed_index, pos_of_fixed_index);
% end


% 
% %%
% A_4_length = No_FinalNode_Start + No_FinalNode_Goal + 1;
% A_3_iter = iter;
% 
% param.output.A_4_length = A_4_length;
% param.output.A_3_iter = A_3_iter;
% param.output.time = time;
% 
% param.setting.eps = eps;
% 
% 
% % result.Smooth_Path = Smooth_Path; 
% % result.Final_Path = Final_Path;
% % result.param = param;
% % result.StartTree = StartTree;
% % result.GoalTree = GoalTree;
% % Smooth_Path, Final_Path, param, StartTree, GoalTree
% path.Smooth_Path = Smooth_Path;
% path.Final_Path = Final_Path;
% path.StartTree = StartTree;
% path.GoalTree = GoalTree;














