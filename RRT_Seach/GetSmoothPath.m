function [ Smooth_Path ] = GetSmoothPath( Final_Path, stepsize )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    iter = 1;
    ball_size = stepsize - 0.01;
    Smooth_Path = Final_Path;
    path_length = size(Smooth_Path);
    CurrentNode.Index = iter;
    CurrentNode.Position = Smooth_Path(CurrentNode.Index, :);
    
    while(1)
        if CurrentNode.Index > path_length(1)
            return;
        else
            CurrentNode.Position = Smooth_Path(CurrentNode.Index, :);
        end
        New_Index = GetNewConnectNode(Smooth_Path, CurrentNode, ball_size);
        if New_Index == 0
        else
            Smooth_Path = [Smooth_Path(1:iter, :,:) ; Smooth_Path(New_Index: path_length(1), :,:)];
            path_length = size(Smooth_Path);
        end
        iter = iter + 1;
        CurrentNode.Index = iter;
    end

end

