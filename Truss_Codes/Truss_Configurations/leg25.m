function n = leg25
% leg shape with 10 nodes, 25 members

addpath(genpath(pwd));

n.pos(:,1) =  [ 1  1  0  ];
n.pos(:,2) =  [-1  1  0  ];
n.pos(:,3) =  [-1 -1  0  ];
n.pos(:,4) =  [ 1 -1  0  ];
n.pos(:,5) =  [ 0  0  1];
n.pos(:,6) =  [ 0  0.8  1.5  ];
n.pos(:,7) =  [-0.8  0  1.5  ];
n.pos(:,8) =  [ 0 -0.8  1.5  ];
n.pos(:,9) =  [ 0.8  0  1.5  ];
n.pos(:,10) = [ 0  0  2];


n.elist(1,:) = [1 5];
n.elist(2,:) = [1 6];
n.elist(3,:) = [1 9];
n.elist(4,:) = [2 5];
n.elist(5,:) = [2 6];
n.elist(6,:) = [2 7];
n.elist(7,:) = [3 5];
n.elist(8,:) = [3 7];
n.elist(9,:) = [3 8];
n.elist(10,:) = [4 5];
n.elist(11,:) = [4 8];
n.elist(12,:) = [4 9];
n.elist(13,:) = [6 7];
n.elist(14,:) = [7 8];
n.elist(15,:) = [8 9];
n.elist(16,:) = [6 9];
n.elist(17,:) = [6 8];
n.elist(18,:) = [5 6];
n.elist(19,:) = [5 7];
n.elist(20,:) = [5 8];
n.elist(21,:) = [5 9];
n.elist(22,:) = [10 6];
n.elist(23,:) = [10 7];
n.elist(24,:) = [10 8];
n.elist(25,:) = [10 9];
% 
% 
% top_add_variable;


end

