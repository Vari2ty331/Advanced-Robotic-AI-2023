function Final_Path = foot_GroundNodeIndex(Final_Path_T)

Polygon_Tolerence = 1e-4;

Final_Path = Final_Path_T;

% Array used for find front node
% If non-rotation node is 2, use Node_Conversion(2) to find front node.
Node_Conversion = [ 4, 5, 6, 1, 2, 3];      

for k = 2 : length(Final_Path)
    index = 1;
    for m = 1 : 3
        rotation_node_flag = 0;
        for j = 1 : 3
            if norm(Final_Path(k).n(j,:) - Final_Path(k-1).n(m,:)) <= Polygon_Tolerence
                rotation_node_flag = 1;
                Final_Path(k).node(index) = Final_Path(k-1).node(m);
                index = index + 1;
                break;
            end
        end
        if rotation_node_flag == 0
            Final_Path(k-1).rear_node = Final_Path(k-1).node(m);
            Final_Path(k).node(3) = Node_Conversion(Final_Path(k-1).node(m));
        end
    end
end

end