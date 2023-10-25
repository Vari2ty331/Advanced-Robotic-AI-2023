function [ New_Connect_Index ] = GetNewConnectNode( Path, Node, ball_size )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    path_length = size(Path);
    New_Connect_Index = 0;
    for i=Node.Index+1:1:path_length(1)
        dist = Distance(Path(i,:), Node.Position);
        if dist < ball_size
            New_Connect_Index = i;
        end
    end
end

