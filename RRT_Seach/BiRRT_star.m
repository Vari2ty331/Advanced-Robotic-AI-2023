function [path, StartTree, GoalTree] = BiRRT_star(RRT_structure, truss, maxIter, wayPoint)
tic;
global fixed_index pos_of_fixed_index

IsWayPoint = false;
if nargin < 3
    maxIter = 10000000;
elseif nargin == 4
    IsWayPoint = true;
end


%% set param
iter = 1;
eps = 0.00001;

%% set user defined param
param = RRT_structure;
Dim = param.Dim;
stepsize = param.stepsize;
radius_factor = 5;
max_neighbors = 7;
% bMixingFactor = 0;

Start_Node = param.Start_Node;
Goal_Node = param.Goal_Node;
Config_Limit = param.Config_Limit;


% h = param.h;
% if Dim == 2 && isempty(h) == 1
%     h = figure;
% end

%% 1step setting

StartTree.Node(1).Position = Start_Node;
StartTree.Node(1).Parent_Index = 0;
StartTree.Node(1).Cost = 0;
StartTree.No_Node = 1;
StartTree = makeReference(StartTree);

GoalTree.Node(1).Position = Goal_Node;
GoalTree.Node(1).Parent_Index = 0;
GoalTree.Node(1).Cost = 0;
GoalTree.No_Node = 1;
GoalTree = makeReference(GoalTree);

check_constraints_truss(truss, planning_pos_2_whole_pos(Start_Node, fixed_index, pos_of_fixed_index));
check_constraints_truss(truss, planning_pos_2_whole_pos(Goal_Node, fixed_index, pos_of_fixed_index));

%% RRT

disp('RRT star planning...');

while(1)
    %% Start Tree
    % random node sampling
    if rand(1) < param.mixingFactor
        Random_Node = Goal_Node;
        if IsWayPoint
            Random_Node = wayPoint;
        end
        bMixingFactor = 1;
    else
        Random_Node = GetRandomNode(Dim, Config_Limit);
        bMixingFactor = 0;
    end
    
    
    
    % find nearest neighbor
    [NN_Start_ind, New_Node_Candidate_Start] = findNearestNode(StartTree, Random_Node, stepsize);
  
    % check constraint
    flag_Start = check_constraints_truss(truss, planning_pos_2_whole_pos(New_Node_Candidate_Start, fixed_index, pos_of_fixed_index));
    num_neighbors = min(StartTree.No_Node, max_neighbors);
    if flag_Start == 1
        % add new node and update cost
        StartTree = addNewNode(StartTree, NN_Start_ind, New_Node_Candidate_Start);
        StartTree.Node(StartTree.No_Node).Cost = StartTree.Node(NN_Start_ind).Cost + Distance(New_Node_Candidate_Start, StartTree.Node(NN_Start_ind).Position);
        
        % within a radius_factor * stepsize, find at least num_neighbors exisitng nodes
        q_nearest_ind_start = [];
        q_nearest_ind_temp_start = knnsearch(New_Node_Candidate_Start', StartTree.reference, num_neighbors);
        for j = 3:length(q_nearest_ind_temp_start)
            if Distance(New_Node_Candidate_Start, StartTree.Node(q_nearest_ind_temp_start(j)).Position) <= radius_factor*stepsize
               for rr = 1:floor(radius_factor)-1
                   node_interpolated(:,rr) = rr*New_Node_Candidate_Start + (radius_factor-rr)*StartTree.Node(q_nearest_ind_temp_start(j)).Position;
                   flag_interpolated(1, rr) = check_constraints_truss(truss, planning_pos_2_whole_pos(node_interpolated(:,rr), fixed_index, pos_of_fixed_index));
               end
               if isequal(flag_interpolated, ones(1, floor(radius_factor)-1))
                   q_nearest_ind_start = [q_nearest_ind_start, q_nearest_ind_temp_start(j)];
               end  
            end
        end
        
        q_min = StartTree.Node(NN_Start_ind);
        cost_min = StartTree.Node(StartTree.No_Node).Cost;
        
        for k = 1:length(q_nearest_ind_start)
           if StartTree.Node(q_nearest_ind_start(k)).Cost + Distance(StartTree.Node(q_nearest_ind_start(k)).Position, New_Node_Candidate_Start) < cost_min
               q_min = StartTree.Node(q_nearest_ind_start(k));
               cost_min = StartTree.Node(q_nearest_ind_start(k)).Cost + Distance(StartTree.Node(q_nearest_ind_start(k)).Position, New_Node_Candidate_Start);
               nearest_parent_ind = q_nearest_ind_start(k);
               StartTree.Node(StartTree.No_Node).Parent_Index = nearest_parent_ind;
           end
        end
        


        [Check_Result_Goal, GoalTree_Connect_Index] = Node_Tree_Distance_Check(StartTree.Node(StartTree.No_Node).Position, GoalTree, radius_factor*stepsize);
        
   
        for rr = 1:floor(radius_factor)-1
            node_interpolated_btw_trees(:,rr) = rr*GoalTree.Node(GoalTree_Connect_Index).Position + (radius_factor-rr)*StartTree.Node(StartTree.No_Node).Position;
            flag_interpolated_btw_trees(1, rr) = check_constraints_truss(truss, planning_pos_2_whole_pos(node_interpolated_btw_trees(:,rr), fixed_index, pos_of_fixed_index));
        end
        if Check_Result_Goal == 1 && isequal(flag_interpolated_btw_trees, ones(1, floor(radius_factor)-1))
            StartTree_Connect_Index = StartTree.No_Node;
            break;
        end
    end
    
    %% Goal Tree
    % find nearest neighbor
    if bMixingFactor
        Random_Node = Start_Node;
    end
    
    [NN_Goal_ind, New_Node_Candidate_Goal] = findNearestNode(GoalTree, Random_Node, stepsize);
    
    % check constraint
%     flag_Goal = checkConstraint(New_Node_Candidate_Goal, params);
    flag_Goal = check_constraints_truss(truss, planning_pos_2_whole_pos(New_Node_Candidate_Goal, fixed_index, pos_of_fixed_index));
    num_neighbors = min(GoalTree.No_Node, max_neighbors);
    if flag_Goal == 1
        % add new node and update cost
        GoalTree = addNewNode(GoalTree, NN_Goal_ind, New_Node_Candidate_Goal);
        GoalTree.Node(GoalTree.No_Node).Cost = GoalTree.Node(NN_Goal_ind).Cost + Distance(New_Node_Candidate_Goal, GoalTree.Node(NN_Goal_ind).Position);
        
        % within a radius_factor * stepsize, find at least num_neighbors exisitng nodes
        q_nearest_ind_goal = [];
        q_nearest_ind_temp_goal = knnsearch(New_Node_Candidate_Goal', GoalTree.reference, num_neighbors);
        for j = 3:length(q_nearest_ind_temp_goal)
            if Distance(New_Node_Candidate_Goal, GoalTree.Node(q_nearest_ind_temp_goal(j)).Position) <= radius_factor*stepsize
               for rr = 1:floor(radius_factor)-1
                   node_interpolated(:,rr) = rr*New_Node_Candidate_Goal + (radius_factor-rr)*GoalTree.Node(q_nearest_ind_temp_goal(j)).Position;
                   flag_interpolated(1, rr) = check_constraints_truss(truss, planning_pos_2_whole_pos(node_interpolated(:,rr), fixed_index, pos_of_fixed_index));
               end
               if isequal(flag_interpolated, ones(1, floor(radius_factor)-1))
                   q_nearest_ind_goal = [q_nearest_ind_goal, q_nearest_ind_temp_goal(j)];
               end  
            end
        end
        
        q_min = GoalTree.Node(NN_Goal_ind);
        cost_min = GoalTree.Node(GoalTree.No_Node).Cost;
        
        for k = 1:length(q_nearest_ind_goal)
           if GoalTree.Node(q_nearest_ind_goal(k)).Cost + Distance(GoalTree.Node(q_nearest_ind_goal(k)).Position, New_Node_Candidate_Goal) < cost_min
               q_min = GoalTree.Node(q_nearest_ind_goal(k));
               cost_min = GoalTree.Node(q_nearest_ind_goal(k)).Cost + Distance(GoalTree.Node(q_nearest_ind_goal(k)).Position, New_Node_Candidate_Goal);
               nearest_parent_ind = q_nearest_ind_goal(k);
               GoalTree.Node(GoalTree.No_Node).Parent_Index = nearest_parent_ind;
           end
        end
        
        

        [Check_Result_Start, StartTree_Connect_Index] = Node_Tree_Distance_Check(GoalTree.Node(GoalTree.No_Node).Position, StartTree, radius_factor*stepsize);
        
   
        for rr = 1:floor(radius_factor)-1
            node_interpolated_btw_trees(:,rr) = rr*StartTree.Node(StartTree_Connect_Index).Position + (radius_factor-rr)*GoalTree.Node(GoalTree.No_Node).Position;
            flag_interpolated_btw_trees(1, rr) = check_constraints_truss(truss, planning_pos_2_whole_pos(node_interpolated_btw_trees(:,rr), fixed_index, pos_of_fixed_index));
        end
        if Check_Result_Start == 1 && isequal(flag_interpolated_btw_trees, ones(1, floor(radius_factor)-1))
            GoalTree_Connect_Index = GoalTree.No_Node;
            break;
        end
    end
    

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
Final_Path = [Final_Path_S; Final_Path_G];
Final_Path = Final_Path';
time = toc;

Smooth_Path = GetSmoothPath(Final_Path, stepsize);
% Smooth_Path = Smooth_Path';

path.final_path = zeros(size(Final_Path,1) + length(fixed_index), size(Final_Path,2));
path.smooth_path = zeros(size(Smooth_Path,1) + length(fixed_index), size(Smooth_Path,2));

for i=1:size(Final_Path,2)
    path.final_path(:,i) = planning_pos_2_whole_pos(Final_Path(:,i), fixed_index, pos_of_fixed_index);
end

for i=1:size(Smooth_Path,2)
    path.smooth_path(:,i) = planning_pos_2_whole_pos(Smooth_Path(:,i), fixed_index, pos_of_fixed_index);
end


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














