function n = tetra6_link_connection

n.pos(:,1) = [-sqrt(3)/3 0 0];
n.pos(:,2) = [sqrt(3)/6 0.5 0];
n.pos(:,3) = [sqrt(3)/6 -0.5 0];
n.pos(:,4) = [0 0 sqrt(6)/3];

n.pos = n.pos';

n.elist = [1 2 ; 2 3 ; 3 1 ; 4 1 ; 4 2 ; 4 3];
n.link_connection{1} = [1 3 ; 3 4 ; 4 1];
n.link_connection{2} = [1 2 ; 2 5 ; 5 1];
n.link_connection{3} = [2 3 ; 3 6 ; 6 2];
n.link_connection{4} = [4 5 ; 5 6 ; 6 4];


end