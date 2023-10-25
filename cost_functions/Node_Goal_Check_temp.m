function  [Check_Result, dist] = Node_Goal_Check_temp(T, goal)

Check_Result = 0;
p = [1 2 3 1 2];

for k = 1:3
    x1 = T.n(p(k),1);       y1 = T.n(p(k),2);
    x2 = T.n(p(k+1),1);     y2 = T.n(p(k+1),2);
    x3 = T.n(p(k+2),1);     y3 = T.n(p(k+2),2);
    
    a = y2 - y1;
    b = x1 - x2;
    c = (x2 - x1)*y1 + (y1 - y2)*x1;
    line_fun = @(x,y) a*x + b*y + c;

    if line_fun(goal(1),goal(2))*line_fun(x3,y3) > 0
        in_check(k) = 1;
    end
end

if sum(in_check) == 3
    Check_Result = 1;
end

dist = norm(sum(T.n)/3 - goal);

