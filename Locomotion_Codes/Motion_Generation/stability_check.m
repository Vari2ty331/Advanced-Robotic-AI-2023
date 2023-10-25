stability = 1;

rotate_node = [NaN NaN];
front_node = [];
back_node = [];
dist = [];
in_check = [];
x_cm = data.x_cm(index-1,:);

for i = 1:length(sup_polygon)-1
    X1 = [n.pos(sup_polygon(i),1:2) 0];
    X2 = [n.pos(sup_polygon(i+1),1:2) 0];
    x1 = X1(1);     y1 = X1(2);
    x2 = X2(1);     y2 = X2(2);
    
    if i <= length(sup_polygon)-2
        sp = sup_polygon(i+2);
    else
        sp = sup_polygon(2);
    end

    x3 = n.pos(sp,1);
    y3 = n.pos(sp,2);

    a = y2 - y1;
    b = x1 - x2;
    c = (x2 - x1)*y1 + (y1 - y2)*x1;
    line_fun = @(x,y) a*x + b*y + c;

    dist(i) = abs(line_fun(x_cm(1),x_cm(2))) / norm([a b]);
    if line_fun(x_cm(1),x_cm(2))*line_fun(x3,y3) > 0
        in_check(i) = 1;
    else
        in_check(i) = 0;
    end
end
    
if min(dist) < data.desired_stb_margin || sum(in_check) ~= length(sup_polygon)-1
    stability = 0;
    i = find(dist == min(dist));
    rotate_node = [sup_polygon(i) sup_polygon(i+1)];
    free_node = 1:length(n.pos);
    free_node(free_node == rotate_node(1)) = [];
    free_node(free_node == rotate_node(2)) = []; 
end
   
 
% if it is not stable, find front node
if stability == 0

    front_node = [];

    for i = 1:length(n.pos)
        x_cm1 = data.x_cm(index-2,:);
        x_cm2 = [n.pos(i,1:2) 0];

        X0 = n.pos(i,:);
        X1 = [n.pos(rotate_node(1),1:2) 0];
        X2 = [n.pos(rotate_node(2),1:2) 0];

        x1 = X1(1);     y1 = X1(2);
        x2 = X2(1);     y2 = X2(2);

        line_fun = @(x,y) (y2 - y1)*x + (x2 - x1)*y1 - (y2 - y1)*x1 - (x2 - x1)*y;
        if line_fun(x_cm1(1),x_cm1(2))*line_fun(x_cm2(1),x_cm2(2)) < 0 
            front_node = [front_node i];
        end
    end
    
    front_node(front_node == rotate_node(1)) = [];
    front_node(front_node == rotate_node(2)) = [];

    for i = 1:length(data.non_front_node)
        front_node(front_node == data.non_front_node(i)) = [];
    end

end