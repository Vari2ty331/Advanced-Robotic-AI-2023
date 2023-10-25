function bool = wall_collision(tree,walls)
last_foot = tree.T(end).n;
last_foot = last_foot(:,1:2);
if length(last_foot) == 3
    last_foot = last_foot([1 2 3 1],:);
elseif length(last_foot) == 4
    last_foot = last_foot([1 2 3 4 1],:);
else
    list = 1:length(last_foot);
    last_foot = last_foot([list 1],:);    
end
last_foot = last_foot';
wall_starts = reshape([walls.wall.Start],[3,length([walls.wall.Start])/3])';
wall_starts = wall_starts(:,1:2)';
wall_ends = reshape([walls.wall.End],[3,length([walls.wall.End])/3])';
wall_ends = wall_ends(:,1:2)';
wall_x = [];
wall_y = [];
for i = 1:length(wall_starts)
    wall_x = [wall_x wall_starts(1,i) wall_ends(1,i) NaN];
    wall_y = [wall_y wall_starts(2,i) wall_ends(2,i) NaN];
end

[intersection_x,intersection_y] = polyxpoly(wall_x,wall_y,last_foot(1,:),last_foot(2,:));

bool = ~isempty(intersection_x) && ~isempty(intersection_y);


end