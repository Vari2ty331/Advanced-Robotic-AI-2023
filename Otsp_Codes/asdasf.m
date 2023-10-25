% Define number of vertices in the hexagon
num_vertices = 5;

% Generate random 2D convex hexagon
hexagon = rand(num_vertices, 2);

% Find all possible unique sets of triangulations
triangulations = find_triangulations(hexagon);

% Define function to find all possible unique sets of triangulations
function triangulations = find_triangulations(vertices)
    % Define number of vertices in polygon
    num_vertices = size(vertices, 1);
    
    % Initialize triangulations cell array with one triangulation
    triangulations = {[1, 2, 3, 4, 5]};
    
    % Iterate over all possible diagonals
    for i = 1:num_vertices-3
        for j = i+2:num_vertices-1
            % Check if diagonal is inside polygon
            inside = true;
            for k = 1:num_vertices
                if k == i || k == j
                    continue;
                end
                if ~is_inside(vertices(i,:), vertices(j,:), vertices(k,:))
                    inside = false;
                    break;
                end
            end
            
            % If diagonal is inside polygon, add new triangulations
            if inside
                for t = 1:numel(triangulations)
                    old_triangulation = triangulations{t};
                    new_triangulation_1 = [old_triangulation(1:i) j old_triangulation(i+1:j-1) i];
                    new_triangulation_2 = [old_triangulation(1:i) j old_triangulation(j+1:end)];
                    new_triangulation_3 = [old_triangulation(1:j-1) i old_triangulation(j:end)];
                    triangulations{end+1} = new_triangulation_1;
                    triangulations{end+1} = new_triangulation_2;
                    triangulations{end+1} = new_triangulation_3;
                end
            end
        end
    end
    
    % Remove duplicates from triangulations
    triangulations = unique(cellfun(@(x) sort(x), triangulations, 'UniformOutput', false), 'stable');
end

% Define function to check if a point is inside a triangle
function inside = is_inside(a, b, c, p)
    if nargin < 4
        p = mean([a;b;c], 1);
    end
    v1 = b - a;
    v2 = c - a;
    v3 = p - a;
    inside = all(cross([v1 0],[v2 0]) * cross([v1 0],[v3 0])' >= 0) && all(cross([v2 0],[v1 0]) * cross([v2 0],[p-c 0])' >= 0) && all(cross([v3 0],[v2 0]) * cross([v3 0],[a-b 0])' >= 0);
end