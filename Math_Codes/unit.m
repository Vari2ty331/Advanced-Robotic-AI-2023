function u = unit( n1,n2 )

if nargin == 1
    u = n1 / norm(n1);
else
    u = (n2 - n1) / norm(n2 - n1);
end
