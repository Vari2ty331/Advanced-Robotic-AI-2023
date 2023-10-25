function n = cube26

n.pos(:,1) = [1 0 1];
n.pos(:,2) = [0 0 0];
n.pos(:,3) = [0 0 1];
n.pos(:,4) = [1 1 0];
n.pos(:,5) = [1 1 1];
n.pos(:,6) = [0 1 0];
n.pos(:,7) = [0 1 1];
n.pos(:,8) = [1 0 0];
n.pos(:,9) = [0.5 0.5 0.5];

n.pos = n.pos';

n.elist(1,:) = [8 4];
n.elist(2,:) = [2 6];
n.elist(3,:) = [1 5];
n.elist(4,:) = [3 7];
n.elist(5,:) = [8 1];
n.elist(6,:) = [2 3];
n.elist(7,:) = [4 5];
n.elist(8,:) = [6 7];
n.elist(9,:) = [8 2];
n.elist(10,:) = [4 6];
n.elist(11,:) = [1 3];
n.elist(12,:) = [5 7];
n.elist(13,:) = [8 3];
n.elist(14,:) = [3 5];
n.elist(15,:) = [5 6];
n.elist(16,:) = [8 6];
n.elist(17,:) = [8 5];
n.elist(18,:) = [3 6];
n.elist(19,:) = [8 9];
n.elist(20,:) = [1 9];
n.elist(21,:) = [2 9];
n.elist(22,:) = [3 9];
n.elist(23,:) = [4 9];
n.elist(24,:) = [5 9];
n.elist(25,:) = [6 9];
n.elist(26,:) = [7 9];


% top_add_variable;

end