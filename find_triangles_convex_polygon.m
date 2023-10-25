function triangles = find_triangles_convex_polygon(polygon)
% polygon is a N-by-2 matrix of the polygon vertices

n = size(polygon, 1);
triangles = {};

if n <= 2
    return;
end

for i = 2:n-1
    % iterate over all possible middle vertices
    for j = i+1:n-1
        % iterate over all possible end vertices

        % check if the triangle formed by i,j,k is inside the polygon
        inside_polygon = true;
        for k = i+1:j-1
            if ~is_point_inside_triangle(polygon(i,:), polygon(j,:), polygon(k,:), polygon)
                inside_polygon = false;
                break;
            end
        end

        if inside_polygon
            triangles{end+1} = [polygon(i,:); polygon(j,:); polygon(k,:)];
        end
    end
end
end

function is_inside = is_point_inside_triangle(p1, p2, p3, polygon)
% check if the triangle formed by p1, p2, p3 is inside the polygon
% polygon is a N-by-2 matrix of the polygon vertices

x = polygon(:,1);
y = polygon(:,2);

% get the bounding box of the triangle
x_min = min([p1(1), p2(1), p3(1)]);
x_max = max([p1(1), p2(1), p3(1)]);
y_min = min([p1(2), p2(2), p3(2)]);
y_max = max([p1(2), p2(2), p3(2)]);

% check if any of the polygon edges intersect the bounding box
for i = 1:size(polygon, 1)
    p4 = polygon(i,:);
    if is_edge_intersecting_box(p1, p2, p4, [x_min, y_min, x_max, y_max]) ...
            || is_edge_intersecting_box(p2, p3, p4, [x_min, y_min, x_max, y_max]) ...
            || is_edge_intersecting_box(p3, p1, p4, [x_min, y_min, x_max, y_max])
        is_inside = false;
        return;
    end
end

% check if the triangle is inside the polygon
if inpolygon(p1(1), p1(2), x, y) ...
        && inpolygon(p2(1), p2(2), x, y) ...
        && inpolygon(p3(1), p3(2), x, y)
    is_inside = true;
else
    is_inside = false;
end
end

function is_intersecting = is_edge_intersecting_box(p1, p2, p3, bbox)
% check if the edge formed by p1 and p2 intersects with the bounding box

x_min = bbox(1);
y_min = bbox(2);
x_max = bbox(3);
y_max = bbox(4);

if (p1(1) < x_min && p2(1) < x_min) ...
        || (p1(1) > x_max && p2(1) > x_max) ...
        || (p1(2) < y_min && p2(2) < y_min) ...
        || (p1(2) > y_max && p2(2) > y_max)
    % the edge is completely outside the bounding box
    is_intersecting = false;
    return;
end

if p1(1) == p2(1)
    % the edge is vertical, so just check if p3 is inside the bbox
    if p1(1) >= x_min && p1(1) <= x_max ...
            && p1(2) >= y_min && p1(2) <= y_max
        is_intersecting = true;
    else
        is_intersecting = false;
    end
    return;
end

if p1(2) == p2(2)
    % the edge is horizontal, so just check if p3 is inside the bbox
    if p1(1) >= x_min && p1(1) <= x_max ...
            && p1(2) >= y_min && p1(2) <= y_max
        is_intersecting = true;
    else
        is_intersecting = false;
    end
    return;
end

% calculate the slope and y-intercept of the edge
m = (p2(2) - p1(2)) / (p2(1) - p1(1));
b = p1(2) - m * p1(1);

% check if p3 is above or below the line
y3 = m * p3(1) + b;
if (p3(2) < y3 && p1(2) < p2(2)) || (p3(2) > y3 && p1(2) > p2(2))
    is_intersecting = false;
    return;
end

% check if the line intersects the bbox
x_intersect = (y_min - b) / m;
if x_intersect >= x_min && x_intersect <= x_max
    is_intersecting = true;
    return;
end

x_intersect = (y_max - b) / m;
if x_intersect >= x_min && x_intersect <= x_max
    is_intersecting = true;
    return;
end

y_intersect = m * x_min + b;
if y_intersect >= y_min && y_intersect <= y_max
    is_intersecting = true;
    return;
end

y_intersect = m * x_max + b;
if y_intersect >= y_min && y_intersect <= y_max
    is_intersecting = true;
    return;
end

is_intersecting = false;
end

