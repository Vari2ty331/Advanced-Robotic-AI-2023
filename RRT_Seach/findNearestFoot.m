function [NN_Index, New_Node_Candidate] = findNearestFoot(tree, new_node, stepsize, Ground)

NN_Index = knnsearch(new_node', tree.reference);

vec_rand = new_node' - tree.reference(NN_Index,:);
New_Step_Start = stepsize*vec_rand/norm(vec_rand);

New_Node_Initial = tree.reference(NN_Index,:) + New_Step_Start;
New_Node_Candidate = New_Node_Initial;
% [New_Node_CollCheck, distance] = Ground_Collision(Ground, New_Node_Candidate);
% 
% if isempty(find(New_Node_CollCheck)) == 1               % The case that New_Node_Candidate is NOT colliding with ground meshes
%     New_Node_Candidate = New_Node_Candidate - [0, 0, -min(distance)];
%     while isempty(find(New_Node_CollCheck)) == 1
%         % Lowering the z value of New_Node_Candidate until it hits the ground meshes
%         New_Node_Candidate = New_Node_Candidate - [0, 0, 0.001];
%         New_Node_CollCheck = Ground_Collision(Ground, New_Node_Candidate);
%     end
% else                                                    % The case that New_Node_Candidate is colliding with ground meshes
%     while isempty(find(New_Node_CollCheck)) ~= 1        
%         % Lifting the z value of New_Node_Candidate until it is not collidiing with the ground meshes
%         New_Node_Candidate = New_Node_Candidate + [0, 0, 0.001];
%         New_Node_CollCheck = Ground_Collision(Ground, New_Node_Candidate);
%     end
% end
