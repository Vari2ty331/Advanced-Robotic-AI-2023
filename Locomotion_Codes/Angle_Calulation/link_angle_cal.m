function link_angle = link_angle_cal(n)

link_connection = n.link_connection;
elist = n.elist;
pos = n.pos;

index = 1;

for i = 1:length(pos)
    for j = 1:length(link_connection{i})
        % node number
        p1 = i;
        p2 = elist(link_connection{i}(j,1),:);
        p3 = elist(link_connection{i}(j,2),:);

        p2(p2 == p1) = [];
        p3(p3 == p1) = [];

        % node position
        x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
        x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
        x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);

        v1 = [x2 y2 z2] - [x1 y1 z1];
        v2 = [x3 y3 z3] - [x1 y1 z1];
    
        link_angle(index,1) = vector_angle(v1,v2);

        index = index + 1;           
    end
end