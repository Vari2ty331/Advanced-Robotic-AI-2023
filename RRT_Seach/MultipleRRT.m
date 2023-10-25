function [path, local_tree, IATT_concat] = MultipleRRT(RRT_structure, truss, maxIter, maxTrees)
tic;
global fixed_index pos_of_fixed_index

if nargin < 3
    maxIter = 10000000;
    maxTrees = 100;
elseif nargin < 4
    maxTrees = 5;
end


%% set param
iter = 1;
eps = 0.00001;

%% set user defined param
param = RRT_structure;
Dim = param.Dim;
stepsize = param.stepsize;
% bMixingFactor = 0;

Start_Node = param.Start_Node';
Goal_Node = param.Goal_Node';
Config_Limit = param.Config_Limit;

% num_startNode_in_constraints = 1;
% num_goalNode_in_constraints = 1;

% h = param.h;
% if Dim == 2 && isempty(h) == 1
%     h = figure;
% end

%% 1step setting

local_tree = struct;
for i = 1:maxTrees
    local_tree(i).valid = 0;
end

% local_tree(1) represents the start tree
local_tree(1).Position(1,:) = Start_Node;
local_tree(1).Parent_Index = 0;
local_tree(1).valid = 1;

% local_tree(2) represents the goal tree
local_tree(2).Position = Goal_Node;
local_tree(2).Parent_Index = 0;
local_tree(2).valid = 1;

numTree = 2;
assert(check_constraints_truss(truss, planning_pos_2_whole_pos(Start_Node, fixed_index, pos_of_fixed_index)));
assert(check_constraints_truss(truss, planning_pos_2_whole_pos(Goal_Node, fixed_index, pos_of_fixed_index)));

%% RRT
IATT_concat = [];
disp('RRT planning...');

comparing_node = local_tree(1).Position;
for cur_it = 1:maxIter
    
    if mod(cur_it, 100) == 0
        disp(['iteration: ', num2str(cur_it)]);
    end
    % random node sampling
    if rand(1) < param.mixingFactor
        if Distance(comparing_node, Start_Node) > Distance(comparing_node, Goal_Node)
            Random_Node = Start_Node + stepsize*randn(1,Dim);
        else
            Random_Node = Goal_Node + stepsize*randn(1,Dim);
        end
    else
        Random_Node = GetRandomNode(Dim, Config_Limit)';
    end
    
    % find the nearest neighbors of each tree
    [NN_Index, New_Node_Candidate] = multipleFindNearestNode(local_tree, Random_Node, stepsize);
    
    IsPathFound = false;
    temp_tree = local_tree;

    IsAddedToTree = zeros(maxTrees, 1);
    for i = 1:maxTrees
        if ~temp_tree(i).valid
            continue;
        end
        
        flag_constraints = check_constraints_truss(truss, planning_pos_2_whole_pos(New_Node_Candidate(i,:), fixed_index, pos_of_fixed_index));
        if flag_constraints
            temp_tree(i) = multipleAddNewNode(temp_tree(i), NN_Index(i,1), New_Node_Candidate(i,:));
            IsAddedToTree(i) = 1;
        end       
    end
    
    if sum(IsAddedToTree) > 1
        trees_added_to = find(IsAddedToTree == 1);
        parent_node = temp_tree(trees_added_to(1)).Parent_Index(end);
        
        for j = 2:numel(trees_added_to)
            if Distance(temp_tree(trees_added_to(1)).Position(end,:), temp_tree(trees_added_to(j)).Position(end,:)) > stepsize
                continue;
            end
            original_parents = temp_tree(trees_added_to(j)).Parent_Index;
            current_node_index = numel(temp_tree(trees_added_to(j)).Parent_Index);
            temp_tree(trees_added_to(j)).Parent_Index(current_node_index) = parent_node;
            
            while original_parents(current_node_index) ~= 0
                this_node_parents_index = original_parents(current_node_index);
                temp_tree(trees_added_to(j)).Parent_Index(this_node_parents_index) = current_node_index;
                current_node_index = this_node_parents_index;
            end
            
            connectedToLastNode = temp_tree(trees_added_to(1)).Parent_Index == numel(temp_tree(trees_added_to(1)).Parent_Index);
            temp_tree(trees_added_to(1)).Position = [temp_tree(trees_added_to(1)).Position(1:end-1,:); temp_tree(trees_added_to(j)).Position];
            temp_tree(trees_added_to(1)).Parent_Index = [temp_tree(trees_added_to(1)).Parent_Index(1:end-1); ...
                                                        temp_tree(trees_added_to(j)).Parent_Index(1:end-1) + numel(temp_tree(trees_added_to(1)).Parent_Index)-1;...
                                                        parent_node];
            temp_tree(trees_added_to(1)).Parent_Index(connectedToLastNode) = numel(temp_tree(trees_added_to(1)).Parent_Index);
            temp_tree(trees_added_to(j)).valid = 0;
            numTree = numTree - 1;
        end
        
        foundStart = find(sum(temp_tree(trees_added_to(1)).Position - Start_Node == zeros(1,Dim), 2) == Dim, 1);
        foundGoal = find(sum(temp_tree(trees_added_to(1)).Position - Goal_Node == zeros(1,Dim), 2) == Dim, 1);
        
%         foundStart = find(temp_tree(trees_added_to(1)).Position(:,1) == Start_Node(1) ...
%                          & temp_tree(trees_added_to(1)).Position(:,2) == Start_Node(2) ...
%                          & temp_tree(trees_added_to(1)).Position(:,3) == Start_Node(3), 1);                      
%         
%         foundGoal = find(temp_tree(trees_added_to(1)).Position(:,1) == Goal_Node(1) ...
%                          & temp_tree(trees_added_to(1)).Position(:,2) == Goal_Node(2) ...
%                          & temp_tree(trees_added_to(1)).Position(:,3) == Goal_Node(3), 1);
                     
        if ~isempty(foundStart) && ~isempty(foundGoal)
            IsPathFound = true;
        end
        
        
    elseif sum(IsAddedToTree) == 0 && numTree <= maxTrees
        for j = 1:maxTrees
            if temp_tree(j).valid == 0
                temp_tree(j).valid = 1;
                temp_tree(j).Position = New_Node_Candidate(randi(numTree),:);
                temp_tree(j).Parent_Index = 0;
                numTree = numTree + 1;
                break;
            end
        end
    end
    comparing_node = local_tree(1).Position(end,:);
    local_tree = temp_tree;
    IATT_concat = [IATT_concat, IsAddedToTree];
    
    if IsPathFound
        disp('Path is found...');
        break;
    end
end

%% get final path
disp('getting final path...');
goal_node_index = find(sum(local_tree(1).Position - Goal_Node == zeros(1,Dim), 2) == Dim, 1);
% Trace path back through tree
Final_Path = Goal_Node;
%     parent=n;
while local_tree(1).Parent_Index(goal_node_index)~=0
    parent = local_tree(1).Parent_Index(goal_node_index);
    Final_Path = [local_tree(1).Position(parent,:);Final_Path];
    goal_node_index = parent;
end

% if length(StartTree_Connect_Index) ~= 1
%     StartTree_Connect_Index = StartTree_Connect_Index(1);
% end
% if length(GoalTree_Connect_Index) ~= 1
%     GoalTree_Connect_Index = GoalTree_Connect_Index(1);
% end

Smooth_Path = GetSmoothPath(Final_Path, stepsize);
% Smooth_Path = Smooth_Path';

path.final_path = zeros(size(Final_Path,2) + length(fixed_index), size(Final_Path,1));
path.smooth_path = zeros(size(Smooth_Path,2) + length(fixed_index), size(Smooth_Path,1));

for i=1:size(Final_Path,1)
    path.final_path(:,i) = planning_pos_2_whole_pos(Final_Path(i,:), fixed_index, pos_of_fixed_index);
end

for i=1:size(Smooth_Path,1)
    path.smooth_path(:,i) = planning_pos_2_whole_pos(Smooth_Path(i,:), fixed_index, pos_of_fixed_index);
end

% global_tree = local_tree(1);
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














