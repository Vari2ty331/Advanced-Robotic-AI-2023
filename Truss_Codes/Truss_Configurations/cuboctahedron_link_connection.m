function n = cuboctahedron_link_connection

% Bottom nodes
n.pos(:,1) = [-sqrt(3)/3 0 0];
n.pos(:,2) = [sqrt(3)/6 0.5 0];
n.pos(:,3) = [sqrt(3)/6 -0.5 0];

% Middle nodes

n.pos(:,4) = [0 -1 sin(asec(-sqrt(3)))];
n.pos(:,5) = [-sqrt(3)/2 -0.5 sin(asec(-sqrt(3)))];
n.pos(:,6) = [-sqrt(3)/2 0.5 sin(asec(-sqrt(3)))];
n.pos(:,7) = [0 1 sin(asec(-sqrt(3)))];
n.pos(:,8) = [sqrt(3)/2 0.5 sin(asec(-sqrt(3)))];
n.pos(:,9) = [sqrt(3)/2 -0.5 sin(asec(-sqrt(3)))];

% Top nodes
n.pos(:,10) = [sqrt(3)/3 0 sin(asec(-sqrt(3)))*2];
n.pos(:,11) = [-sqrt(3)/6 -0.5 sin(asec(-sqrt(3)))*2];
n.pos(:,12) = [-sqrt(3)/6 0.5 sin(asec(-sqrt(3)))*2];

n.pos = n.pos';

% Bottom members
n.elist(1,:) = [1 2];
n.elist(2,:) = [2 3];
n.elist(3,:) = [3 1];

% Top members
n.elist(4,:) = [10 11];
n.elist(5,:) = [11 12];
n.elist(6,:) = [12 10];

% Side bottom members
n.elist(7,:) = [3 4];
n.elist(8,:) = [1 5];
n.elist(9,:) = [1 6];
n.elist(10,:) = [2 7];
n.elist(11,:) = [2 8];
n.elist(12,:) = [3 9];

% Side members
n.elist(13,:) = [4 5];
n.elist(14,:) = [5 6];
n.elist(15,:) = [6 7];
n.elist(16,:) = [7 8];
n.elist(17,:) = [8 9];
n.elist(18,:) = [9 4];

% Side top members

n.elist(19,:) = [8 10];
n.elist(20,:) = [9 10];
n.elist(21,:) = [4 11];
n.elist(22,:) = [5 11];
n.elist(23,:) = [6 12];
n.elist(24,:) = [7 12];


n.link_connection{1} = [1 3 ; 3 7 ; 7 8 ; 8 1];
n.link_connection{2} = [1 2 ; 2 9 ; 9 10 ; 10 1];
n.link_connection{3} = [2 3 ; 3 12 ; 12 11 ; 11 2];
n.link_connection{4} = [4 6 ; 6 9 ; 9 11 ; 11 4];
n.link_connection{5} = [4 5 ; 5 7 ; 7 12 ; 12 4];  
n.link_connection{6} = [5 6 ; 6 10 ; 10 8 ; 8 5];

end