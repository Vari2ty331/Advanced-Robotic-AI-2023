function  [Check_Result, dist] = Node_Goal_Distance_Check( WholePos, MassMatrix, GoalComPos, stepsize )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

Check_Result = 0;

WholePos = RRTPathToLocomotionPath(WholePos); % Change sequence of variable to [x1 x2 x3...]

CurrentComPos = (MassMatrix * WholePos')';

%% dist based on Com
% dist = norm(CurrentComPos - GoalComPos);
% if dist < stepsize
%     Check_Result = 1;
% end

%% dist based on desired configuration

dist = norm(CurrentComPos - GoalComPos);

if dist < stepsize
    Check_Result = 1;
end


end
