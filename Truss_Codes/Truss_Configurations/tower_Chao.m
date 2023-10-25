function n = tower_Chao

n.pos(:,1) = [-1.60 -0.77 2.08];
n.pos(:,2) = [0.78 -0.76 2.08];
n.pos(:,3) = [-0.48 -2.02 0.08];
n.pos(:,4) = [-0.41 0.42 2.18];
n.pos(:,5) = [-1.61 -0.77 0.08];
n.pos(:,6) = [0.38 -0.37 0.13];
n.pos(:,7) = [-0.43 -0.96 1.23];
n.pos(:,8) = [-0.48 -2.02 2.08];
n.pos(:,9) = [0.18 -0.17 0.08];

n.pos = n.pos';


n.elist(1,:) = [0 3];
n.elist(2,:) = [0 4];
n.elist(3,:) = [0 6];
n.elist(4,:) = [0 7];
n.elist(5,:) = [1 3];
n.elist(6,:) = [1 5];
n.elist(7,:) = [1 6];
n.elist(8,:) = [1 7];
n.elist(9,:) = [2 4];
n.elist(10,:) = [2 6];
n.elist(11,:) = [2 7];
n.elist(12,:) = [2 8];
n.elist(13,:) = [3 5];
n.elist(14,:) = [3 6];
n.elist(15,:) = [3 7];
n.elist(16,:) = [4 6];
n.elist(17,:) = [4 7];
n.elist(18,:) = [4 8];
n.elist(19,:) = [5 7];
n.elist(20,:) = [6 7];
n.elist(21,:) = [6 8];

n.elist = n.elist + 1;

% n.link_connection{1} = [1 3 ; 3 7 ; 7 8 ; 8 1];
% n.link_connection{2} = [1 2 ; 2 9 ; 9 10 ; 10 1];
% n.link_connection{3} = [2 3 ; 3 12 ; 12 11 ; 11 2];
% n.link_connection{4} = [4 6 ; 6 9 ; 9 11 ; 11 4];
% n.link_connection{5} = [4 5 ; 5 7 ; 7 12 ; 12 4];  
% n.link_connection{6} = [5 6 ; 6 10 ; 10 8 ; 8 5];

end