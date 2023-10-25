function bool = checkFootCenterInsideTriangle(tree,nearest_node)

query_center = tree.T(nearest_node).center_point;
end_center = tree.T(end).center_point;
end_foot = tree.T(end).n;
end_foot = end_foot(1:2,:);
query_foot = tree.T(nearest_node).n;
query_foot = query_foot(1:2,:);

xv = query_foot(:,1);
yv = query_foot(:,2);
xq = end_center(1);
yq = end_center(2);

[in,on] = inpolygon(xq,yq,xv,yv);



if(in ~= 1)

    bool = true;

else

    bool = false;

end


end