function triangulations = findTriangulations(vertices)
    n = size(vertices, 1); % number of vertices

    % generate all possible combinations of vertices
    combos = nchoosek(1:n, 3);

    % initialize list of triangulations
    triangulations = {};

    % loop through each combination of vertices
    for i = 1:size(combos, 1)
        % check if this combination forms a valid triangulation
        v1 = vertices(combos(i, 1), :);
        v2 = vertices(combos(i, 2), :);
        v3 = vertices(combos(i, 3), :);
        edges = [v1 v2; v2 v3; v3 v1]; % edges of the triangle
        valid = true;

        % check if all edges are contained within the polygon
        for j = 1:size(edges, 1)
            if ~edgeInPolygon(edges(j, :), vertices)
                valid = false;
                break;
            end
        end

        % check if the diagonals do not intersect with any other edges
        if valid
            for j = 1:n
                if pointInTriangle(vertices(j, :), edges)
                    valid = false;
                    break;
                end
            end
        end

        % add the triangulation to the list if it is valid
        if valid
            triangulations{end+1} = combos(i, :);
        end
    end
end

function inPolygon = edgeInPolygon(edge, vertices)
    inPolygon = false;
    n = size(vertices, 1);
    for i = 1:n
        v1 = vertices(i, :);
        v2 = vertices(mod(i,n)+1, :);
        if segmentsIntersect(edge, [v1 v2])
            inPolygon = true;
            break;
        end
    end
end

function inTriangle = pointInTriangle(point, triangle)
    inTriangle = true;
    for i = 1:3
        v1 = triangle(i, :);
        v2 = triangle(mod(i,3)+1, :);
        if leftTurn(v1, v2, point)
            inTriangle = false;
            break;
        end
    end
end

function intersect = segmentsIntersect(s1, s2)
    p1 = s1(1, :);
    q1 = s1(2, :);
    p2 = s2(1, :);
    q2 = s2(2, :);
    if leftTurn(p1, q1, p2) ~= leftTurn(p1, q1, q2) && ...
            leftTurn(p2, q2, p1) ~= leftTurn(p2, q2, q1)
        intersect = true;
    else
        intersect = false;
    end
end

function left = leftTurn(p1, p2, p3)
    left = ((p2(1)-p1(1))*(p3(2)-p1(2))-(p3(1)-p1(1))*(p2(2)-p1(2))) >= 0;
end