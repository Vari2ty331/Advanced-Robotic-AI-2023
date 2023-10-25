function angle = angle_cal(n)

angle = [];
adj = adj_gen(n.elist);
index = 1;
for i = 1:length(n.pos)
    for j = 1:length(adj{i})
        for k = j+1:length(adj{i})
            % node number
            p1 = i;
            p2 = adj{i}(j);
            p3 = adj{i}(k);

            % node position
            x1 = n.pos(p1,1);     y1 = n.pos(p1,2);     z1 = n.pos(p1,3);
            x2 = n.pos(p2,1);     y2 = n.pos(p2,2);     z2 = n.pos(p2,3);
            x3 = n.pos(p3,1);     y3 = n.pos(p3,2);     z3 = n.pos(p3,3);

            v1 = [x2 y2 z2] - [x1 y1 z1];
            v2 = [x3 y3 z3] - [x1 y1 z1];

            angle(index,1) = vector_angle(v1,v2);
            index = index + 1;
        end
    end
end