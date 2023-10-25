function [path, StartTree, iter, dist_data] = Locomotion_retraction_RRT(RRT_structure, truss, maxIter, fixed_index, pos_of_fixed_index, MassMatrix, num_retract)
tic;

% global dist_data
dist_data = [];

if nargin < 7
    num_retract = 3;
end

%% set param
iter = 1;
eps = 0.00001;
mixing_factor = 0.2;

%% set user defined param
param = RRT_structure;
Dim = param.Dim;
stepsize = param.stepsize;

Start_Node = param.Start_Node;
Goal_ComPos = param.Goal_ComPos;
Config_Limit = param.Config_Limit;

%% 1step setting

StartTree.Node(1).Position = Start_Node;
StartTree.Node(1).Parent_Index = 0;
StartTree.No_Node = 1;
StartTree = makeReference(StartTree);

IsCheckResultStart = 0;
check_constraints_truss(truss, planning_pos_2_whole_pos(Start_Node, fixed_index, pos_of_fixed_index));

%% RRT
disp('RRT planning...');

while(1)
%     %% Start Tree
%     % random node sampling
% %     if rand(1) < param.mixingFactor
% %         Random_Node = Goal_Node;
% %         bMixingFactor = 1;
% %     else
%         Random_Node = GetRandomNode(Dim, Config_Limit);
%         bMixingFactor = 0;
% %     end
    Random_Node = GetRandomNode(Dim, Config_Limit); % generate 1 random node within Config_Limit 
    bMixingFactor = 0;
        
    % find nearest neighbor
    [NN_Start, New_Node_Candidate_Start] = findNearestNode(StartTree, Random_Node, stepsize);
  
    % check constraint
    flag_Start = check_constraints_truss(truss, planning_pos_2_whole_pos(New_Node_Candidate_Start, fixed_index, pos_of_fixed_index));
    
    if flag_Start
        StartTree = addNewNode(StartTree, NN_Start, New_Node_Candidate_Start);
        [Check_Result_Goal, dist] = Node_Goal_Distance_Check(planning_pos_2_whole_pos(StartTree.Node(StartTree.No_Node).Position, fixed_index, pos_of_fixed_index), MassMatrix, Goal_ComPos, stepsize);
        dist_data = [dist_data; dist];
        
        if Check_Result_Goal == 1
            disp('check')
            IsCheckResultStart = 1;
            break;
        end
    else
        parent_index = StartTree.Node(NN_Start).Parent_Index;
        if parent_index ~= 0
            New_Node_Candidate_Start_dir = StartTree.Node(NN_Start).Position - StartTree.Node(parent_index).Position;
            for ii = 1:num_retract
                New_Node_Candidate_Start = StartTree.Node(NN_Start).Position + ii*stepsize*New_Node_Candidate_Start_dir/norm(New_Node_Candidate_Start_dir);
                if check_constraints_truss(truss, planning_pos_2_whole_pos(New_Node_Candidate_Start, fixed_index, pos_of_fixed_index))
                    StartTree = addNewNode(StartTree, NN_Start, New_Node_Candidate_Start);
                    Check_Result_Goal =  Node_Goal_Distance_Check(planning_pos_2_whole_pos(StartTree.Node(StartTree.No_Node).Position, fixed_index, pos_of_fixed_index), MassMatrix, Goal_ComPos, stepsize);
                    dist_data = [dist_data; dist];
                    if Check_Result_Goal == 1
                        IsCheckResultStart = 1;
                        break;
                    end
                else
                    break;
                end
            end
            if IsCheckResultStart
                break;
            end
        end
    end

    iter = iter +1;
    if mod(iter, 10000) == 0
        disp(['iteration: ', num2str(iter),'   min. distance to goal: ',num2str(min(dist_data))]);
    end
    if mod(iter, maxIter) == 0
        disp(['reach maximum iteration: ',num2str(maxIter)]);
       break;
    end
end


%% get final path
disp('getting final path...');

StartTree.No_Node;

if IsCheckResultStart
    No_FinalNode_Start = 1;
    Current_Node = StartTree.No_Node;
    while(StartTree.Node(Current_Node).Parent_Index ~= 0)
        Final_Path_Start(No_FinalNode_Start) = StartTree.Node(Current_Node).Parent_Index;
        Current_Node = StartTree.Node(Current_Node).Parent_Index;
        No_FinalNode_Start = No_FinalNode_Start + 1;
    end

    Final_Path_S = zeros(No_FinalNode_Start, Dim);
    for i=1: No_FinalNode_Start-1
        Final_Path_S(i,:) = StartTree.Node(Final_Path_Start(No_FinalNode_Start-i)).Position';
    end
    Final_Path_S(No_FinalNode_Start,:) = StartTree.Node(StartTree.No_Node).Position';
   
else
    Final_Path_S = [];
end

Final_Path = [Final_Path_S];
Final_Path = Final_Path';
time = toc;
if length(Final_Path)
    path.final_path = zeros(size(Final_Path,1) + length(fixed_index), size(Final_Path,2));

    for i=1:size(Final_Path,2)
        path.final_path(:,i) = planning_pos_2_whole_pos(Final_Path(:,i), fixed_index, pos_of_fixed_index);
    end
else
    path.final_path = [];
end









