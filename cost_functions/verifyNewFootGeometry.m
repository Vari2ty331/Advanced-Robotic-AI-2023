function flag_Start = verifyNewFootGeometry(tree)

last_foot = tree.T(end).n;

length(1) = sqrt(sum((last_foot(1,:)-last_foot(2,:)).^2,2));
length(2) = sqrt(sum((last_foot(2,:)-last_foot(3,:)).^2,2));
length(3) = sqrt(sum((last_foot(1,:)-last_foot(3,:)).^2,2));

if any(length > 1.25 + 0.15) || any(length < 1.25 - 0.15)
    flag_Start = 0;
else
    flag_Start = 1;
end






end