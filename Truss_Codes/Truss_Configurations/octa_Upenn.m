function n = octahedron
% Octahedron
% 6 nodes, 

n( 1).adj = [2, 4, 5, 6];
n( 2).adj = [1, 3, 5, 6];
n( 3).adj = [2, 4, 5, 6];
n( 4).adj = [1, 3, 5, 6];
n( 5).adj = [1, 2, 3, 4];
n( 6).adj = [1, 2, 3, 4];

n( 1).pos = [1, 0, 0]';
n( 2).pos = [0, 1, 0]';
n( 3).pos = [-1, 0, 0]';
n( 4).pos = [0, -1, 0]';
n( 5).pos = [0, 0, 1]';
n( 6).pos = [0, 0, -1]';

end