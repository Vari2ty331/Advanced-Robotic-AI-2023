function n = Triangular_Column

n.pos(1,:) = [sqrt(3)/3, 0, 0];
n.pos(2,:) = [-sqrt(3)/6, 1/2, 0];
n.pos(3,:) = [-sqrt(3)/6, -1/2, 0];
n.pos(4,:) = [sqrt(3)/3, 0, 1];
n.pos(5,:) = [-sqrt(3)/6, 1/2, 1];
n.pos(6,:) = [-sqrt(3)/6, -1/2, 1];

% n.pos = n.pos';

n.elist(1,:) = [1,2];
n.elist(2,:) = [2,3];
n.elist(3,:) = [3,1];
n.elist(4,:) = [4,5];
n.elist(5,:) = [5,6];
n.elist(6,:) = [6,4];
n.elist(7,:) = [1,4];
n.elist(8,:) = [2,5];
n.elist(9,:) = [3,6];

