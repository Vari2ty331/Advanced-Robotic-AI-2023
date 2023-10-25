function n = Triangular_Column_Concave

n.pos(1,:) = [sqrt(3)/3, 0, 0];
n.pos(2,:) = [-sqrt(3)/6, 1/2, 0];
n.pos(3,:) = [-sqrt(3)/6, -1/2, 0];
n.pos(4,:) = [sqrt(3)/3, 0, 1];
n.pos(5,:) = [-sqrt(3)/6, 1/2, 1];
n.pos(6,:) = [-sqrt(3)/6, -1/2, 1];
n.pos(7,:) = [0, 0, 0.5];

% n.pos = n.pos';

n.elist(1,:) = [1,7];
n.elist(2,:) = [2,7];
n.elist(3,:) = [3,7];
n.elist(4,:) = [4,7];
n.elist(5,:) = [5,7];
n.elist(6,:) = [6,7];
