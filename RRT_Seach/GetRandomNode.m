function [ Random_Node ] = GetRandomNode(Dim, Joint_Limit)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    Random_Sample = rand(Dim, 1); 
    Random_Node = zeros(Dim, 1);
    for i=1:Dim
        Random_Node(i) = Random_Sample(i)*(Joint_Limit(i,2)-Joint_Limit(i,1))+Joint_Limit(i,1);
    end
    Random_Node(3) = 0;
end

