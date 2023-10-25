function tris = triangulate_polygon(polygon)
% Return all possible unique triangulations of a convex polygon
n = length(polygon);
if n < 3
    tris = {};
elseif n == 3
    tris = {polygon};
else
    tris = {};
    for i = 1:n-2
        % Triangle (i,i+1,i+2) is a candidate for a diagonal
        if is_interior_diagonal(polygon, i, i+2)
            % Recursively triangulate the two resulting polygons
            p1 = polygon(1:i,:);
            p2 = polygon(i+1:i+2,:);
            p3 = polygon(i+3:end,:);
            sub_tris1 = triangulate_polygon([p1; p2]);
            sub_tris2 = triangulate_polygon([p2; p3]);
            % Combine the triangles from the two subproblems
            for j = 1:length(sub_tris1)
                for k = 1:length(sub_tris2)
                    tris{end+1} = [sub_tris1{j}; sub_tris2{k}];
                end
            end
        end
    end
end

function result = is_interior_diagonal(polygon, i, j)
% Check if line segment i-j is an interior diagonal of polygon
% (i.e., it does not intersect any other edges)
n = length(polygon);
result = true;
for k = 1:n
    if k == i || k == j || k == mod(i+1,n) || k == mod(j+1,n)
        continue;
    end
    if mod(k+1,n) == 0
        result = false;
        if intersect(polygon(i,:), polygon(j,:), polygon(k,:), polygon(mod(k+1,n),:))
            result = false;
            return;
        end
    end
end

function result = intersect(a, b, c, d)
% Check if line segments ab and cd intersect
result = ccw(a,c,d) ~= ccw(b,c,d) && ccw(a,b,c) ~= ccw(a,b,d);

function result = ccw(a, b, c)
% Check if the three points a, b, and c are in counterclockwise order
result = (c(2)-a(2))*(b(1)-a(1)) > (b(2)-a(2))*(c(1)-a(1));