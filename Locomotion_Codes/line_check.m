function val = line_check(X1,X2,P1,P2)
% check if the two points X1, X2 lies in same or opposite space which divided by the line connecting P1, P2

x1 = X1(1);     y1 = X1(2);
x2 = X2(1);     y2 = X2(2);

a = y2 - y1;
b = x1 - x2;
c = (x2 - x1)*y1 + (y1 - y2)*x1;
line_fun = @(x,y) a*x + b*y + c;

val = line_fun(P1(1),P1(2))*line_fun(P2(1),P2(2));
