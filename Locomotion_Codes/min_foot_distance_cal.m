function dist_min = min_foot_distance_cal(x,dx,front_node,back_node,pos_foot)

x_new = x + dx;
n.pos = reshape(x_new,[length(x)/3, 3]);

dist = [];
for i = [front_node back_node]
    dist = [dist; norm(n.pos(i,:) - pos_foot)];
end
    
dist_min = min(dist);
