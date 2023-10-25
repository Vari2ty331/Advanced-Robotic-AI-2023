function n = fig12_Chao

n.pos(:,1) = [0.05 0 0];
n.pos(:,2) = [0.1 1.8 0];
n.pos(:,3) = [2.1 1.9 0];
n.pos(:,4) = [2.1 0 0];
n.pos(:,5) = [0 2.1 3.1];
n.pos(:,6) = [1.95 0.9 3];
n.pos(:,7) = [0 0 2.9];

n.pos = n.pos';


n.elist(1,:) = [0 1];
n.elist(2,:) = [0 3];
n.elist(3,:) = [0 4];
n.elist(4,:) = [0 5];
n.elist(5,:) = [0 6];
n.elist(6,:) = [1 2];
n.elist(7,:) = [1 3];
n.elist(8,:) = [1 4];
n.elist(9,:) = [1 5];
n.elist(10,:) = [2 3];
n.elist(11,:) = [2 4];
n.elist(12,:) = [2 5];
n.elist(13,:) = [3 4];
n.elist(14,:) = [3 5];
n.elist(15,:) = [3 6];
n.elist(16,:) = [4 5];
n.elist(17,:) = [4 6];
n.elist(18,:) = [5 6];

n.elist = n.elist + 1;

% n.link_connection{1} = [1 3 ; 3 7 ; 7 8 ; 8 1];
% n.link_connection{2} = [1 2 ; 2 9 ; 9 10 ; 10 1];
% n.link_connection{3} = [2 3 ; 3 12 ; 12 11 ; 11 2];
% n.link_connection{4} = [4 6 ; 6 9 ; 9 11 ; 11 4];
% n.link_connection{5} = [4 5 ; 5 7 ; 7 12 ; 12 4];  
% n.link_connection{6} = [5 6 ; 6 10 ; 10 8 ; 8 5];

end